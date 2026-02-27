/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __ENCODER_LINEARIZER_H__
#define __ENCODER_LINEARIZER_H__

#ifndef ENC_LIN_LUT_BITS
#define ENC_LIN_LUT_BITS        12
#endif

#define ENC_LIN_LUT_MASK        ((1 << ENC_LIN_LUT_BITS) - 1)
#define ENC_LIN_FRAC_BITS       (16 - ENC_LIN_LUT_BITS)
#define ENC_LIN_FRAC_MASK       ((1 << ENC_LIN_FRAC_BITS) - 1)
#define ENC_LIN_FRAC_SIZE       (1 << ENC_LIN_FRAC_BITS)


typedef struct {
    const int8_t    *lut;       // -127 ~ 127
    int16_t         max_val;
} encoder_linearizer_t;


static inline uint16_t encoder_linearizer(const encoder_linearizer_t *el, uint16_t raw)
{
    uint16_t idx_base = raw >> ENC_LIN_FRAC_BITS;
    uint16_t idx_next = (idx_base + 1) & ENC_LIN_LUT_MASK;
    uint16_t frac = raw & ENC_LIN_FRAC_MASK;

    int16_t val_base = el->lut[idx_base] << 8;
    int16_t val_next = el->lut[idx_next] << 8;

    int32_t diff = (val_next - val_base) * frac;
    int16_t val = val_base + DIV_ROUND_CLOSEST(diff, ENC_LIN_FRAC_SIZE);

    int32_t scaled = el->max_val * val;
    int16_t delta = DIV_ROUND_CLOSEST(scaled, 128 << 8);
    return raw + delta;
}

#endif
