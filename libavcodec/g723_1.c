#include "avcodec.h"
#define ALT_BITSTREAM_READER_LE
#include "get_bits.h"
#include "acelp_vectors.h"
#include "lsp.h"
#include "g723_1_data.h"

typedef struct g723_1_context {
    int8_t lsp_index[LSP_BANDS];
    int16_t prev_lsp[LPC_ORDER];
    int16_t pitch_lag[2];
    G723_1_Subframe subframe[4];
    FrameType cur_frame_type;
    FrameType past_frame_type;
    Rate cur_rate;
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
    p->lsp_index[0] = get_bits(&gb, 8);
    p->lsp_index[1] = get_bits(&gb, 8);
    p->lsp_index[2] = get_bits(&gb, 8);

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

/*
 * Perform inverse quantization of LSP frequencies.
 *
 * @param cur_lsp    the current LSP vector
 * @param prev_lsp   the previous LSP vector
 */
static void inverse_quant(int16_t *cur_lsp, int16_t *prev_lsp, int8_t *lsp_index, int bad_frame)
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
        temp2      = av_clip_int16((temp1 * pred >> 15) + 1);
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

        ff_acelp_lsp2lpc(lpc_ptr, lpc_ptr, LPC_ORDER >> 1);
        lpc_ptr += LPC_ORDER + 1;
    }
}

static int g723_1_decode_frame(AVCodecContext *avctx, void *data,
                              int *data_size, AVPacket *avpkt)
{
    G723_1_Context *p  = avctx->priv_data;
    const uint8_t *buf = avpkt->data;
    int buf_size       = avpkt->size;

    int16_t cur_lsp[LPC_ORDER];
    int16_t lpc[SUBFRAMES * LPC_ORDER + 4];
    int bad_frame, erased_frames;

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
