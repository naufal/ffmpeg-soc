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
    int16_t ad_cb_lag;     ///< adaptive codebook lag
    int16_t ad_cb_gain;
    int16_t trans_gain;
    int16_t pulse_pos;
    int16_t pulse_sign;
    int16_t grid_index;
    int16_t amp_index;
} G723_1_Subframe;

