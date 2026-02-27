/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __ENCODER_FILTER_H__
#define __ENCODER_FILTER_H__


typedef struct {
    uint16_t    bias_encoder;
    int32_t     bias_pos;

    uint16_t    nob_encoder;
    int32_t     nob_pos;

    uint16_t    sen_encoder;
    float       sen_speed;      // encoder steps per sec

    int32_t     sen_pos;
    float       sen_speed_avg;
    float       sen_rpm_avg;

    // internal
    uint16_t    pos_rec[5];
    int16_t     delta_rec[5];
} encoder_filter_t;


static inline void encoder_filter(encoder_filter_t *ef, uint16_t input)
{
    int32_t pos_i32[5];

    for (int i = 0; i < 4; i++)
        ef->pos_rec[i] = ef->pos_rec[i+1];
    ef->pos_rec[4] = input;

    pos_i32[0] = ef->pos_rec[0];
    for (int i = 0; i < 4; i++) {
        int16_t dt = ef->pos_rec[i+1] - ef->pos_rec[0];
        pos_i32[i+1] = pos_i32[0] + dt;
    }

    int32_t pos_min = pos_i32[0];
    int32_t pos_max = pos_i32[0];
    int32_t pos_sum = 0;
    for (int i = 0; i < 5; i++) {
        pos_sum += pos_i32[i];
        pos_min = min(pos_min, pos_i32[i]);
        pos_max = max(pos_max, pos_i32[i]);
    }

    uint16_t pos_avg = DIV_ROUND_CLOSEST(pos_sum - pos_min - pos_max, 3);
    int16_t delta_encoder = pos_avg - ef->nob_encoder;
    ef->nob_encoder = pos_avg;

    for (int i = 0; i < 4; i++)
        ef->delta_rec[i] = ef->delta_rec[i+1];
    ef->delta_rec[4] = delta_encoder;

    int32_t delta_sum = 0;
    for (int i = 0; i < 5; i++)
        delta_sum += ef->delta_rec[i];

    ef->sen_speed = delta_sum * (CURRENT_LOOP_FREQ / 5.0f);

    ef->sen_speed_avg += (ef->sen_speed - ef->sen_speed_avg) * 0.1f;
    ef->sen_rpm_avg += (ef->sen_speed_avg / 0x10000 * 60 - ef->sen_rpm_avg) * 0.01f;

    // position compensation: step/sec * (2 loop period + 0.000019 sec)
    int16_t pos_comp = lroundf(ef->sen_speed_avg * (2.0f / CURRENT_LOOP_FREQ + 0.000019f));
    ef->sen_encoder = ef->nob_encoder - ef->bias_encoder + pos_comp;

    ef->nob_pos += (int16_t)(ef->sen_encoder - (uint16_t)ef->nob_pos);
    ef->sen_pos = ef->nob_pos - ef->bias_pos;
}

#endif
