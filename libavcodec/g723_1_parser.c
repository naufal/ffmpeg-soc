/*
 * @file
 * G.723.1 parser
 *
 * Copyright (c) 2003 Fabrice Bellard
 * Copyright (c) 2003 Michael Niedermayer
 * Copyright (c) 2010 Martin Storsjo
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "parser.h"
#include "g723_1_data.h"

typedef struct {
    ParseContext pc;
    int frame_size;
} G723_1_ParseContext;

static int g723_1_parse(AVCodecParserContext *s1,
                        AVCodecContext *avctx,
                        const uint8_t **poutbuf, int *poutbuf_size,
                        const uint8_t *buf, int buf_size)
{
    G723_1_ParseContext *s = s1->priv_data;
    ParseContext *pc = &s->pc;
    int next = END_NOT_FOUND, i;

    for (i = 0; i < buf_size; ) {
        if (s->frame_size) {
            int inc = FFMIN(buf_size - i, s->frame_size);
            i += inc;
            s->frame_size -= inc;

            if (!s->frame_size) {
                next = i;
                break;
            }
        } else {
            s->frame_size = frame_size[buf[i] & 3];
        }
    }

    if (ff_combine_frame(pc, next, &buf, &buf_size) < 0) {
        *poutbuf = NULL;
        *poutbuf_size = 0;
        return buf_size;
    }

    *poutbuf = buf;
    *poutbuf_size = buf_size;
    return next;
}

AVCodecParser g723_1_parser = {
    {CODEC_ID_G723_1},
    sizeof(G723_1_ParseContext),
    NULL,
    g723_1_parse,
    ff_parse_close,
};
