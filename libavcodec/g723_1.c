#include "avcodec.h"
#define ALT_BITSTREAM_READER_LE
#include "get_bits.h"
#include "acelp_vectors.h"
#include "lsp.h"
#include "g723_1_data.h"

typedef struct g723_1_context {
    uint8_t lsp_index[LSP_BANDS];
    int16_t prev_lsp[LPC_ORDER];
    int16_t pitch_lag[2];
    int16_t prev_excitation[PITCH_MAX];
    G723_1_Subframe subframe[4];
    FrameType cur_frame_type;
    FrameType past_frame_type;
    Rate cur_rate;

    int16_t random_seed;
    int16_t interp_index;
    int16_t interp_gain;
    int16_t sid_gain;
    int16_t cur_gain;
} G723_1_Context;

static av_cold int g723_1_decode_init(AVCodecContext *avctx)
{
    G723_1_Context *p  = avctx->priv_data;

    avctx->sample_fmt  = SAMPLE_FMT_S16;
    avctx->channels    = 1;
    avctx->sample_rate = 8000;

    memcpy(p->prev_lsp, dc_lsp, LPC_ORDER * sizeof(int16_t));

    return 0;
}

/*
 * Unpack the frame into parameters.
 *
 * @param p           the context
 * @param buf         pointer to the input buffer
 * @param buf_size    size of the input buffer
 */
static int unpack_bitstream(G723_1_Context *p, const uint8_t *buf,
                            int buf_size)
{
    GetBitContext gb;
    int ad_cb_len;
    int temp, info_bits, i;

    init_get_bits(&gb, buf, buf_size * 8);

    // Extract frame type and rate info
    info_bits = get_bits(&gb, 2);

    if (info_bits == 3) {
        p->cur_frame_type = UntransmittedFrame;
        return 0;
    }

    // Extract 24 bit lsp indices, 8 bit for each band
    p->lsp_index[2] = get_bits(&gb, 8);
    p->lsp_index[1] = get_bits(&gb, 8);
    p->lsp_index[0] = get_bits(&gb, 8);

    if (info_bits == 2) {
        p->cur_frame_type = SIDFrame;
        p->subframe[0].amp_index = get_bits(&gb, 6);
        return 0;
    }

    // Extract the info common to both rates
    p->cur_rate       = info_bits ? Rate5k3 : Rate6k3;
    p->cur_frame_type = ActiveFrame;

    p->pitch_lag[0] = get_bits(&gb, 7);
    if (p->pitch_lag[0] > 123)       // test if forbidden code
        return -1;
    p->pitch_lag[0] += PITCH_MIN;
    p->subframe[1].ad_cb_lag = get_bits(&gb, 2);

    p->pitch_lag[1] = get_bits(&gb, 7);
    if (p->pitch_lag[1] > 123)
        return -1;
    p->pitch_lag[1]  += PITCH_MIN;
    p->subframe[3].ad_cb_lag  = get_bits(&gb, 2);
    p->subframe[0].ad_cb_lag  = 1;
    p->subframe[2].ad_cb_lag  = 1;

    for (i = 0; i < SUBFRAMES; i++) {
        // Extract combined gain
        temp =  get_bits(&gb, 12);
        ad_cb_len  = 170;
        p->subframe[i].trans_gain = 0;
        if (p->cur_rate == Rate6k3 && p->pitch_lag[i >> 1] < SUBFRAME_LEN - 2) {
            p->subframe[i].trans_gain = temp >> 11;
            temp &= 0x7ff;
            ad_cb_len = 85;
        }
        p->subframe[i].ad_cb_gain = FASTDIV(temp, GAIN_LEVELS);
        if (p->subframe[i].ad_cb_gain < ad_cb_len) {
            p->subframe[i].amp_index = temp - p->subframe[i].ad_cb_gain *
                                       GAIN_LEVELS;
        } else {
            return -1;
        }
    }

    for (i = 0; i < SUBFRAMES; i++)
        p->subframe[i].grid_index = get_bits(&gb, 1);

    if (p->cur_rate == Rate6k3) {
        skip_bits(&gb, 1);  // skip reserved bit

        // Compute pulse_pos index using the 13-bit combined position index
        temp = get_bits(&gb, 13);
        p->subframe[0].pulse_pos = temp / 810;

        temp -= p->subframe[0].pulse_pos * 810;
        p->subframe[1].pulse_pos = FASTDIV(temp, 90);

        temp -= p->subframe[1].pulse_pos * 90;
        p->subframe[2].pulse_pos = FASTDIV(temp, 9);
        p->subframe[3].pulse_pos = temp - p->subframe[2].pulse_pos * 9;

        p->subframe[0].pulse_pos = (p->subframe[0].pulse_pos << 16) +
                                       get_bits(&gb, 16);
        p->subframe[1].pulse_pos = (p->subframe[1].pulse_pos << 14) +
                                       get_bits(&gb, 14);
        p->subframe[2].pulse_pos = (p->subframe[2].pulse_pos << 16) +
                                       get_bits(&gb, 16);
        p->subframe[3].pulse_pos = (p->subframe[3].pulse_pos << 14) +
                                       get_bits(&gb, 14);

        p->subframe[0].pulse_sign = get_bits(&gb, 6);
        p->subframe[1].pulse_sign = get_bits(&gb, 5);
        p->subframe[2].pulse_sign = get_bits(&gb, 6);
        p->subframe[3].pulse_sign = get_bits(&gb, 5);
    } else { // Rate5k3
        for (i = 0; i < SUBFRAMES; i++)
            p->subframe[i].pulse_pos  = get_bits(&gb, 12);

        for (i = 0; i < SUBFRAMES; i++)
            p->subframe[i].pulse_sign = get_bits(&gb, 4);
    }

    return 0;
}

static inline int16_t abs_16(int16_t x)
{
    int16_t mask = x >> 15;
    int16_t v = (x + mask) ^ mask;
    return (v & INT16_MIN) ? INT16_MAX : v;
}

static inline int16_t prand(int16_t *rseed)
{
    *rseed = *rseed * 521 + 259;
    return *rseed;
}

static inline int dot_product(const int16_t *v1, const int16_t *v2, int length)
{
    int i, sum = 0;
    int64_t prod;

    for (i = 0; i < length; i++) {
        prod = av_clipl_int32((int64_t)v1[i] * v2[i] << 1);
        sum  = av_clipl_int32(sum + prod);
    }
    return sum;
}

static int16_t normalize_bits_int16(int16_t x)
{
    int16_t i = 0;

    if (x) {
        if (x == -1)
            return 15;
        if (x < 0)
            x = ~x;
        for (i = 0; x < 0x4000; i++)
            x <<= 1;
    }
    return i;
}

static int16_t normalize_bits_int32(int x)
{
    int16_t i = 0;
    if (x) {
        if (x == -1)
            return 31;
        if (x < 0)
            x = ~x;
        for (i = 0; x < 0x40000000; i++)
            x <<= 1;
    }
    return i;
}

static int16_t scale_vector(int16_t *vector, int16_t length)
{
    int16_t scale, max = 0;
    int i;

    const int16_t shift_table[16] = {
        0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,
        0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x7fff
    };

    for (i = 0; i < length; i++) {
        int16_t v = abs_16(vector[i]);
        max = FFMAX(v, max);
    }

    scale = shift_table[normalize_bits_int16(max)];

    for (i = 0; i < length; i++)
        vector[i] = (int16_t)av_clipl_int32(vector[i] * scale << 1) >> 4;

    return scale - 3;
}

/*
 * Perform inverse quantization of LSP frequencies.
 *
 * @param cur_lsp    the current LSP vector
 * @param prev_lsp   the previous LSP vector
 */
static void inverse_quant(int16_t *cur_lsp, int16_t *prev_lsp,
                          int8_t *lsp_index, int bad_frame)
{
    int min_dist, pred;
    int i, j, temp1, temp2, stable;

    // Check for frame erasure
    if (!bad_frame) {
        min_dist = 0x100;
        pred = 12288;
        lsp_index[0] *= 3;
        lsp_index[1] *= 3;
        lsp_index[2] *= 4;
    } else {
        min_dist = 0x200;
        pred = 23552;
        lsp_index[0] = lsp_index[1] = lsp_index[2] = 0;
    }

    // Get the VQ table entry corresponding to the transmitted index
    cur_lsp[0] = lsp_band0[lsp_index[0]];
    cur_lsp[1] = lsp_band0[lsp_index[0] + 1];
    cur_lsp[2] = lsp_band0[lsp_index[0] + 2];
    cur_lsp[3] = lsp_band1[lsp_index[1]];
    cur_lsp[4] = lsp_band1[lsp_index[1] + 1];
    cur_lsp[5] = lsp_band1[lsp_index[1] + 2];
    cur_lsp[6] = lsp_band2[lsp_index[2]];
    cur_lsp[7] = lsp_band2[lsp_index[2] + 1];
    cur_lsp[8] = lsp_band2[lsp_index[2] + 2];
    cur_lsp[9] = lsp_band2[lsp_index[2] + 3];

    // Add predicted vector & DC component to the previously quantized vector
    for (i = 0; i < LPC_ORDER; i++) {
        temp1      = av_clip_int16(prev_lsp[i] - dc_lsp[i]);
        temp2      = av_clip_int16((temp1 * pred + (1 << 14)) >> 15);
        cur_lsp[i] = av_clip_int16(cur_lsp[i] + temp2);
        cur_lsp[i] = av_clip_int16(cur_lsp[i] + dc_lsp[i]);
    }

    for (i = 0; i < LPC_ORDER; i++) {
        cur_lsp[0]             = FFMAX(cur_lsp[0],  0x180);
        cur_lsp[LPC_ORDER - 1] = FFMIN(cur_lsp[LPC_ORDER - 1], 0x7e00);

        // Stability check
        for (j = 1; j < LPC_ORDER; j++) {
            temp1 = av_clip_int16(min_dist + cur_lsp[j - 1]);
            temp1 = av_clip_int16(temp1 - cur_lsp[j]);
            if (temp1 > 0) {
                temp1 >>= 1;
                cur_lsp[j - 1] =  av_clip_int16(cur_lsp[j - 1] - temp1);
                cur_lsp[j] =  av_clip_int16(cur_lsp[j] + temp1);
            }
        }

        stable = 1;

        for (j = 1; j < LPC_ORDER; j++) {
            temp1 = av_clip_int16(cur_lsp[j - 1] + min_dist);
            temp1 = av_clip_int16(temp1 - 4);
            temp1 = av_clip_int16(temp1 - cur_lsp[j]);
            if (temp1 > 0) {
                stable = 0;
                break;
            }
        }

        if (stable)
            break;
    }

    if (!stable)
        memcpy(cur_lsp, prev_lsp, LPC_ORDER * sizeof(int16_t));
}

/*
 * Quantize LSP frequencies by interpolation and convert them to
 * the corresponding LPC coefficients.
 *
 * @param lpc      buffer for LPC coefficients
 * @param cur_lsp  the current LSP vector
 * @param prev_lsp the previous LSP vector
 */
static void lsp_interpolate(int16_t *lpc, int16_t *cur_lsp, int16_t *prev_lsp)
{
    int i, j;
    int16_t *lpc_ptr = lpc;

    // cur_lsp * 0.25 + prev_lsp * 0.75
    ff_acelp_weighted_vector_sum(&lpc[1], cur_lsp, prev_lsp,
                                 4096, 12288, 1 << 13, 14, LPC_ORDER);
    ff_acelp_weighted_vector_sum(&lpc[LPC_ORDER + 1], cur_lsp, prev_lsp,
                                 8192, 8192, 1 << 13, 14, LPC_ORDER);
    ff_acelp_weighted_vector_sum(&lpc[LPC_ORDER * 2 + 2], cur_lsp, prev_lsp,
                                 12288, 4096, 1 << 13, 14, LPC_ORDER);
    memcpy(&lpc[LPC_ORDER * 3 + 3], cur_lsp, LPC_ORDER * sizeof(int16_t));

    for (i = 0; i < SUBFRAMES; i++) {
        // Calculate cosine
        for (j = 1; j <= LPC_ORDER; j++) {
            int index      = lpc_ptr[j] >> 7;
            int offset     = lpc_ptr[j] & 0x7f;
            int64_t temp1  = cos_tab[index] << 16;
            int temp2      = (cos_tab[index + 1] - cos_tab[index]) *
                             ((offset << 8) + 0x80) << 1;
            lpc_ptr[j] = av_clipl_int32(((temp1 + temp2) << 1) + (1 << 15)) >> 16;
        }

        ff_acelp_lsp2lpc(lpc_ptr, lpc_ptr, LPC_ORDER >> 1, 9, 1 << 9, -1);
        lpc_ptr += LPC_ORDER + 1;
    }
}

/*
 * Generate fixed codebook excitation vector
 *
 * @param vector decoded excitation vector
 * @param index  the current subframe index
 */
static void gen_fcb_excitation(int16_t *vector, G723_1_Subframe subfrm,
                               Rate cur_rate, int16_t pitch_lag, int index)
{
    int temp, i, j;

    memset(vector, 0, SUBFRAMES * sizeof(int16_t));

    if (cur_rate == Rate6k3) {
        if (subfrm.pulse_pos >= max_pos[index])
            return;

        // Decode amplitudes and positions
        j = PULSE_MAX - pulses[index];
        temp = subfrm.pulse_pos;
        for (i = 0; i < SUBFRAME_LEN / GRID_SIZE; i++) {
            temp -= combinatorial_table[j][i];
            if (temp >= 0)
                break;
            temp += combinatorial_table[j++][i];
            if (!(subfrm.pulse_sign & (1 << (PULSE_MAX - j)))) {
                vector[subfrm.grid_index + GRID_SIZE * i] =
                                        -fixed_cb_gain[subfrm.amp_index];
            } else {
                vector[subfrm.grid_index + GRID_SIZE * i] =
                                         fixed_cb_gain[subfrm.amp_index];
            }
            if (j == PULSE_MAX)
                break;
        }
        // Generate Dirac train
        if (subfrm.trans_gain == 1) {
            int16_t temp_vector[SUBFRAME_LEN];
            memcpy(temp_vector, vector, SUBFRAME_LEN * sizeof(int16_t));
            for (i = pitch_lag; i < SUBFRAME_LEN; i += pitch_lag)
                for (j = 0; j < SUBFRAME_LEN - i; j++)
                    vector[i + j] += temp_vector[j];
        }
    } else { // Rate5k3
        int16_t cb_gain  = fixed_cb_gain[subfrm.amp_index];
        int16_t cb_shift = subfrm.grid_index;
        int16_t cb_sign  = subfrm.pulse_sign;
        int16_t cb_pos   = subfrm.pulse_pos;
        int offset, beta, lag, temp;

        for (i = 0; i < 8; i += 2) {
            offset =  ((cb_pos & 7) << 3) + cb_shift + i;
            vector[offset] = (cb_sign & 1) ? cb_gain : -cb_gain;
            cb_pos  >>= 3;
            cb_sign >>= 1;
        }

        // Enhance harmonic components
        lag  = pitch_contrib[subfrm.ad_cb_gain << 1] + pitch_lag +
               subfrm.ad_cb_lag - 1;
        beta = pitch_contrib[(subfrm.ad_cb_lag << 1) + 1];

        if (lag < SUBFRAME_LEN - 2) {
            for (i = lag; i < SUBFRAME_LEN; i++) {
                temp = av_clip_int16(beta * vector[i - lag] >> 15);
                vector[i] = av_clip_int16(vector[i] + temp);
            }
        }
    }
}

/*
 * Generate adaptive codebook excitation
 *
 * @param vector restored excitation vector
 */
static void gen_acb_excitation(int16_t *vector, int16_t *prev_excitation,
                               int16_t pitch_lag, G723_1_Subframe subfrm,
                               Rate cur_rate)
{
    int16_t residual[SUBFRAME_LEN + PITCH_ORDER - 1];
    const int16_t *cb_ptr;
    int lag = pitch_lag + subfrm.ad_cb_lag - 1;
    int temp = PITCH_MAX - PITCH_ORDER / 2 - lag;
    int i;
    int64_t sum;

    residual[0] = prev_excitation[temp];
    residual[1] = prev_excitation[temp + 1];

    for (i = 2; i < SUBFRAME_LEN + PITCH_ORDER - 1; i++)
        residual[i] = prev_excitation[temp + (i - 2) % lag];

    // Select quantization table
    if (cur_rate == Rate6k3 && pitch_lag < SUBFRAME_LEN - 2)
        cb_ptr = adaptive_cb_gain85;
    else
        cb_ptr = adaptive_cb_gain170;

    // Calculate adaptive vector
    cb_ptr += subfrm.ad_cb_gain * 20;
    for (i = 0; i < SUBFRAME_LEN; i++) {
        sum = dot_product(residual + i, cb_ptr, PITCH_ORDER);
        vector[i] = av_clipl_int32((sum << 1) + (1 << 15)) >> 16;
    }
}

/*
 * Search for the best pitch postfilter forward/backward lag
 * and compute cross-correlation.
 *
 * @param buf decoded excitation
 * @param ccr cross-correlation
 * @param dir forward(1) or backward(-1)
 */
static int16_t get_ppf_lag(int16_t *buf, int *ccr_max, int16_t pitch_lag,
                           int length, int dir)
{
    int ccr, lag = 0;
    int i;

    pitch_lag = FFMIN(PITCH_MAX - 3, pitch_lag);

    for (i = pitch_lag - 3; i <= pitch_lag + 3; i++) {
        ccr = dot_product(buf, buf + dir * i, length);

        if (ccr > *ccr_max) {
            *ccr_max = ccr;
            lag  = i;
        }
    }
    return lag;
}

/*
 * Calculate pitch postfilter optimal and scaling gains.
 *
 * @param lag     pitch postfilter forward/backward lag
 * @param ppf     pitch postfilter parameters
 * @param tgt_eng target energy
 * @param ccr     cross-correlation
 * @param res_eng residual energy
 */
static void comp_ppf_gains(int16_t lag, PPFParam *ppf, Rate cur_rate,
                           int tgt_eng, int ccr, int res_eng)
{
    int pf_residual;     // square of postfiltered residual
    int64_t temp1, temp2;

    ppf->index = lag;

    temp1 = tgt_eng * res_eng >> 1;
    temp2 = av_clipl_int32((int64_t)ccr * ccr << 1);

    if (temp2 > temp1) {
        if (ccr >= res_eng) {
            ppf->opt_gain = ppf_gain_weight[cur_rate];
        } else {
            ppf->opt_gain = av_clip_int16((ccr << 15) / res_eng);
            ppf->opt_gain = av_clip_int16(ppf->opt_gain *
                                          ppf_gain_weight[cur_rate] >> 15);
        }
        // pf_res^2 = tgt_eng + 2*ccr*gain + res_eng*gain^2
        tgt_eng     <<= 15;
        temp1       =   av_clipl_int32((int64_t)ccr * ppf->opt_gain << 1);
        pf_residual =   av_clipl_int32(tgt_eng + temp1);
        temp1       =   av_clip_int16(ppf->opt_gain * ppf->opt_gain >> 15);
        temp1       =   av_clipl_int32(temp1 * res_eng);
        pf_residual =   av_clipl_int32(pf_residual + temp1 + (1 << 15)) &
                        0xffff0000;

        if (tgt_eng >= pf_residual) {
            temp1 = 0x7fff;
        } else {
            temp1 = av_clip_int16(((int64_t)tgt_eng << 15) / pf_residual);
        }

        // scaling_gain = sqrt(tgt_eng/pf_res^2)
        ppf->sc_gain = ff_sqrt(temp1 << 15);
    } else {
        ppf->opt_gain = 0;
        ppf->sc_gain  = 0x7fff;
    }

    ppf->opt_gain = av_clip_int16(ppf->opt_gain * ppf->opt_gain >> 15);
}

/*
 * Calculate pitch postfilter parameters.
 *
 * @param buf   decoded excitation
 * @param ppf   pitch postfilter parameters
 * @param index current subframe index
 */
static void comp_ppf_coeff(int16_t *buf, int16_t pitch_lag, PPFParam *ppf,
                           Rate cur_rate)
{
    /*
     * 0 - target energy
     * 1 - forward cross-correlation
     * 2 - forward residual energy
     * 3 - backward cross-correlation
     * 4 - backward residual energy
     */
    int energy[5] = {0, 0, 0, 0, 0};

    int16_t fwd_lag  = get_ppf_lag(buf, &energy[1], pitch_lag, SUBFRAME_LEN, 1);
    int16_t back_lag = get_ppf_lag(buf, &energy[3], pitch_lag, SUBFRAME_LEN,-1);

    int16_t scale;
    int i;
    int64_t temp1, temp2;

    ppf->index    = 0;
    ppf->opt_gain = 0;
    ppf->sc_gain  = 0x7fff;

    // Case 0, Section 3.6
    if (!back_lag && !fwd_lag)
        return;

    // Compute target energy
    energy[0] = dot_product(buf, buf, SUBFRAME_LEN);

    // Compute forward residual energy
    if (fwd_lag)
        energy[2] = dot_product(buf + fwd_lag, buf + fwd_lag, SUBFRAME_LEN);

    // Compute backward residual energy
    if (back_lag)
        energy[4] = dot_product(buf - back_lag, buf - back_lag, SUBFRAME_LEN);

    // Normalize and shorten
    temp1 = 0;
    for (i = 0; i < 5; i++)
        temp1 = FFMAX(energy[i], temp1);

    scale = normalize_bits_int32(temp1);
    for (i = 0; i < 5; i++)
        energy[i] = av_clipl_int32(energy[i] << scale) >> 16;

    if (fwd_lag && !back_lag) {  // Case 1
        comp_ppf_gains(fwd_lag,  ppf, cur_rate, energy[0], energy[1],
                       energy[2]);
    } else if (!fwd_lag) {       // Case 2
        comp_ppf_gains(back_lag, ppf, cur_rate, energy[0], energy[3],
                       energy[4]);
    } else {                     // Case 3

        // Select the largest of energy[1]^2/energy[2] and energy[3]^2/energy[4]
        temp1 = av_clip_int16((energy[1] * energy[1] + (1 << 14)) >> 15);
        temp1 = av_clipl_int32(temp1 * energy[4]);
        temp2 = av_clip_int16((energy[3] * energy[3] + (1 << 14)) >> 15);
        temp2 = av_clipl_int32(temp2 * energy[2]);

        if (temp1 >= temp2) {
            comp_ppf_gains(fwd_lag,  ppf, cur_rate, energy[0], energy[1],
                           energy[2]);
        } else {
            comp_ppf_gains(back_lag, ppf, cur_rate, energy[0], energy[3],
                           energy[4]);
        }
    }
}

/*
 * Classify frames as voiced/unvoiced.
 *
 * @param buf     decoded excitation vector
 * @param exc_eng excitation energy estimation
 * @param scale   scaling factor of exc_eng
 *
 * @return residual interpolation index if voiced, 0 otherwise
 */
static int16_t comp_interp_index(int16_t *buf, int16_t pitch_lag,
                                 int16_t *exc_eng, int16_t *scale)
{
    int16_t index;
    int16_t *vector_ptr;
    int ccr, tgt_eng, best_eng;
    int temp;

    *scale     = scale_vector(buf, FRAME_LEN + PITCH_MAX);

    pitch_lag  = FFMIN(PITCH_MAX - 3, pitch_lag);
    vector_ptr = buf + PITCH_MAX + 2 * SUBFRAME_LEN;
    index      = pitch_lag;

    // Compute maximum backward cross-correlation
    ccr   = 0;
    index = get_ppf_lag(vector_ptr, &ccr, pitch_lag, SUBFRAME_LEN * 2, -1);
    ccr   = av_clipl_int32((int64_t)ccr + (1 << 15)) >> 16;

    // Compute target energy
    tgt_eng   = dot_product(vector_ptr, vector_ptr, SUBFRAME_LEN * 2);
    *exc_eng  = av_clipl_int32(tgt_eng + (1 << 15)) >> 16;

    if (ccr <= 0)
        return 0;

    // Compute best energy
    best_eng = dot_product(vector_ptr - index, vector_ptr - index,
                           SUBFRAME_LEN * 2);
    best_eng = av_clipl_int32((int64_t)best_eng + (1 << 15)) >> 16;

    temp = best_eng * tgt_eng >> 3;

    if (temp < ccr * ccr)
        return index;
    else
        return 0;
}

/*
 * Peform residual interpolation based on frame classification.
 *
 * @param buf   decoded excitation vector
 * @prarm out   output vector
 * @param lag   decoded pitch lag
 * @param gain  interpolated gain
 * @param rseed seed for random number generator
 */
static void residual_interp(int16_t *buf, int16_t *out, int16_t lag,
                            int16_t gain, int16_t *rseed)
{
    int i;
    if (lag) { // Voiced
        int16_t *vector_ptr = buf + PITCH_MAX;
        // Attenuate
        for (i = 0; i < lag; i++)
            vector_ptr[i - lag] = vector_ptr[i - lag] * 0x6000 >> 15;
        for (i = 0; i < FRAME_LEN; i++)
            vector_ptr[i] = vector_ptr[i - lag];
        memcpy(out, vector_ptr, FRAME_LEN * sizeof(int16_t));
    } else {  // Unvoiced
        for (i = 0; i < FRAME_LEN; i++)
            out[i] = gain * prand(rseed) >> 15;
        memset(buf, 0, (FRAME_LEN + PITCH_MAX) * sizeof(int16_t));
    }
}

static int g723_1_decode_frame(AVCodecContext *avctx, void *data,
                               int *data_size, AVPacket *avpkt)
{
    G723_1_Context *p  = avctx->priv_data;
    const uint8_t *buf = avpkt->data;
    int buf_size       = avpkt->size;
    int16_t *out       = data;

    PPFParam ppf[SUBFRAMES];
    int16_t cur_lsp[LPC_ORDER];
    int16_t lpc[SUBFRAMES * LPC_ORDER + 4];
    int16_t excitation[FRAME_LEN + PITCH_MAX];
    int16_t acb_vector[SUBFRAME_LEN];
    int16_t *vector_ptr;
    int bad_frame = 0, erased_frames = 0, i, j;

    if (!buf_size || buf_size < frame_size[buf[0] & 3]) {
        *data_size = 0;
        return buf_size;
    }

    if (unpack_bitstream(p, buf, buf_size) < 0) {
        bad_frame         = 1;
        p->cur_frame_type = p->past_frame_type == ActiveFrame ?
                            ActiveFrame : UntransmittedFrame;
    }

    if(p->cur_frame_type == ActiveFrame) {
        if (!bad_frame)
            erased_frames = 0;
        else if(erased_frames != 3)
            erased_frames++;

        inverse_quant(cur_lsp, p->prev_lsp, p->lsp_index, bad_frame);
        lsp_interpolate(lpc, cur_lsp, p->prev_lsp);

        // Save the lsp_vector for the next frame
        memcpy(p->prev_lsp, cur_lsp, LPC_ORDER * sizeof(int16_t));

        // Generate the excitation for the frame
        memcpy(excitation, p->prev_excitation, PITCH_MAX * sizeof(int16_t));
        vector_ptr = excitation + PITCH_MAX;
        if (!erased_frames) {
            // Update interpolation gain memory
            p->interp_gain = fixed_cb_gain[(p->subframe[2].amp_index +
                                        p->subframe[3].amp_index) >> 1];
            for (i = 0; i < SUBFRAMES; i++) {
                gen_fcb_excitation(vector_ptr, p->subframe[i], p->cur_rate,
                                   p->pitch_lag[i >> 1], i);
                gen_acb_excitation(acb_vector, &excitation[SUBFRAME_LEN * i],
                                   p->pitch_lag[i >> 1], p->subframe[i],
                                   p->cur_rate);
                // Get the total excitation
                for (j = 0; j < SUBFRAME_LEN; j++) {
                    vector_ptr[j] = av_clip_int16(vector_ptr[j] << 1);
                    vector_ptr[j] = av_clip_int16(vector_ptr[j] + acb_vector[j]);
                }
                vector_ptr += SUBFRAME_LEN;
            }
            // Save the excitation
            memcpy(out, excitation + PITCH_MAX, FRAME_LEN * sizeof(int16_t));

            p->interp_index = comp_interp_index(excitation, p->pitch_lag[1],
                                                &p->sid_gain, &p->cur_gain);

            vector_ptr = excitation + PITCH_MAX;

            for (i = 0, j = 0; i < FRAME_LEN; i += SUBFRAME_LEN, j++)
                comp_ppf_coeff(vector_ptr + i, p->pitch_lag[i >> 1],
                               ppf + j, p->cur_rate);

            // Restore the original excitation
            memcpy(excitation, p->prev_excitation, PITCH_MAX * sizeof(int16_t));
            memcpy(vector_ptr, out, FRAME_LEN * sizeof(int16_t));

            // Peform pitch postfiltering
            for (i = 0, j = 0; i < FRAME_LEN; i += SUBFRAME_LEN, j++)
                ff_acelp_weighted_vector_sum(out + i, vector_ptr + i,
                                             vector_ptr + i + ppf[j].index,
                                             ppf[j].sc_gain, ppf[j].opt_gain,
                                             1 << 14, 15, SUBFRAME_LEN);
        } else {
            p->interp_gain = (p->interp_gain * 3 + 2) >> 2;
            if (erased_frames == 3) {
                // Mute output
                memset(excitation, 0, (FRAME_LEN + PITCH_MAX) * sizeof(int16_t));
                memset(out, 0, FRAME_LEN * sizeof(int16_t));
            } else {
                // Regenerate frame
                residual_interp(excitation, out, p->interp_index,
                                p->interp_gain, &p->random_seed);
            }
        }
    }

    return frame_size[p->cur_frame_type];
}

AVCodec g723_1_decoder = {
    .name           = "g723_1",
    .type           = AVMEDIA_TYPE_AUDIO,
    .id             = CODEC_ID_G723_1,
    .priv_data_size = sizeof(G723_1_Context),
    .init           = g723_1_decode_init,
    .decode         = g723_1_decode_frame,
    .long_name      = NULL_IF_CONFIG_SMALL("G.723.1"),
    .capabilities   = CODEC_CAP_SUBFRAMES,
};
