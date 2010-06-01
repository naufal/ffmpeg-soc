#include "avcodec.h"
#define ALT_BITSTREAM_READER_LE
#include "get_bits.h"

#define SUBFRAMES    4
#define FRAME_LEN    240
#define SUBFRAME_LEN (FRAME_LEN / SUBFRAMES)
#define PITCH_MIN    18
#define GAIN_LEVELS  24

/*
 * G723.1 frame types
 */
typedef enum {
    ActiveFrame,        ///< Active speech
    SIDFrame,           ///< Silence Insertion Descriptor frame
    UntransmittedFrame
} FrameType;

static const uint8_t frame_size[4] = {24, 20, 4, 1};

typedef enum {
    Rate6k3,
    Rate5k3
} Rate;

/*
 * G723.1 unpacked data subframe
 */
typedef struct {
    uint16_t ad_cb_lag;     ///< adaptive codebook lag
    uint16_t ad_cb_gain;
    uint16_t trans_gain;
    uint16_t pulse_pos;
    uint16_t pulse_sign;
    uint16_t grid_index;
    uint16_t amp_index;
} G723_1_Subframe;

typedef struct g723_1_context {
    uint32_t lsp_index;
    uint16_t pitch_lag[2];
    G723_1_Subframe subframe[4];
    FrameType cur_frame_type;
    Rate cur_rate;
} G723_1_Context;

static av_cold int g723_1_decode_init(AVCodecContext *avctx)
{
    avctx->sample_fmt  = SAMPLE_FMT_S16;
    avctx->channels    = 1;
    avctx->sample_rate = 8000;
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

    p->lsp_index = get_bits(&gb, 24);

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

static int g723_1_decode_frame(AVCodecContext *avctx, void *data,
                              int *data_size, AVPacket *avpkt)
{
    G723_1_Context *p  = avctx->priv_data;
    const uint8_t *buf = avpkt->data;
    int buf_size       = avpkt->size;

    if (!buf_size || buf_size < frame_size[buf[0] & 3]) {
        *data_size = 0;
        return buf_size;
    }

    if (unpack_bitstream(p, buf, buf_size) < 0) {
        av_log(avctx, AV_LOG_ERROR, "G723.1: Bad frame\n");
        return AVERROR_INVALIDDATA;
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
