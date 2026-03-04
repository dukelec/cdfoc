/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __ANTICOG_H__
#define __ANTICOG_H__

#ifndef ANTICOG_LUT_BITS
#define ANTICOG_LUT_BITS        12
#endif

#define ANTICOG_LUT_MASK        ((1 << ANTICOG_LUT_BITS) - 1)
#define ANTICOG_FRAC_BITS       (16 - ANTICOG_LUT_BITS)
#define ANTICOG_FRAC_MASK       ((1 << ANTICOG_FRAC_BITS) - 1)
#define ANTICOG_FRAC_SIZE       (1 << ANTICOG_FRAC_BITS)


typedef struct {
    const int8_t    *lut;       // -127 ~ 127
    int16_t         max_iq;
    int16_t         ratio_vq;
} anticog_t;


static inline void anticog_ff(const anticog_t *ac, uint16_t encoder, int16_t *iq_ff, int16_t *vq_ff)
{
    uint16_t idx_base = encoder >> ANTICOG_FRAC_BITS;
    uint16_t idx_next = (idx_base + 1) & ANTICOG_LUT_MASK;
    uint16_t frac = encoder & ANTICOG_FRAC_MASK;

    int16_t val_base = ac->lut[idx_base] << 8;
    int16_t val_next = ac->lut[idx_next] << 8;

    int32_t diff = (val_next - val_base) * frac;
    int16_t val = val_base + DIV_ROUND_CLOSEST(diff, ANTICOG_FRAC_SIZE);

    int32_t scaled = ac->max_iq * val;
    *iq_ff = DIV_ROUND_CLOSEST(scaled, 128 << 8);
    *vq_ff = DIV_ROUND_CLOSEST(*iq_ff * ac->ratio_vq, 4096);
}

#endif
