/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "trap_planner.h"


void trap_planner_update(trap_planner_t *tp, int32_t pos_real)
{
    if (tp->state < 0) {
        tp->pos_out = pos_real;
        tp->state = 1;
    }

    if (!tp->state) {
        tp->vel_out = 0;
        tp->acc_brake = 0;
        tp->pos_frac = 0;
        return;
    }

    float v_step = tp->acc_tgt * tp->dt;

    if (tp->pos_tgt != tp->pos_out) {
        // t = (v1 - v2) / a; s = ((v1 + v2) / 2) * t; a =>
        tp->acc_brake = ((/* vel_end + */ tp->vel_out) / 2.0f) * (/* vel_end */ - tp->vel_out) / (tp->pos_tgt - tp->pos_out);
        tp->acc_brake = min(fabsf(tp->acc_brake), tp->acc_tgt * 1.2f);
    } else {
        tp->acc_brake = tp->acc_tgt * 1.2f;
    }

    if (tp->acc_brake >= tp->acc_tgt) {
        float delta_v = tp->acc_brake * tp->dt;
        tp->vel_out += -sign(tp->vel_out) * delta_v;
    } else {
        float target_v = ((tp->pos_tgt >= tp->pos_out) ? 1 : -1) * (float)tp->vel_tgt;
        float delta_v = ((target_v >= tp->vel_out) ? 1 : -1) * min(v_step, fabsf(target_v - tp->vel_out));
        tp->vel_out += delta_v;
    }

    float dt_pos = tp->vel_out * tp->dt;
    tp->pos_frac += dt_pos;
    int32_t tmp = lroundf(tp->pos_frac);
    tp->pos_frac -= tmp;
    int32_t pos_next = tp->pos_out + tmp;

    if (tp->max_err && abs(pos_real - pos_next) > tp->max_err)
        pos_next = clip(pos_next, pos_real - tp->max_err, pos_real + tp->max_err);

    if (fabsf(tp->vel_out) <= v_step * 4.4f) {
        // avoid overshooting
        tp->pos_out = (tp->pos_tgt >= tp->pos_out) ? min(pos_next, tp->pos_tgt) : max(pos_next, tp->pos_tgt);
        if (tp->pos_out == tp->pos_tgt) {
            tp->state = 0;
            tp->vel_out = 0;
            tp->acc_brake = 0;
            tp->pos_frac = 0;
        }
    } else {
        tp->pos_out = pos_next;
    }
}


void trap_planner_reset(trap_planner_t *tp, int32_t pos_real, float vel_real_avg)
{
    tp->pos_tgt = pos_real;
    tp->pos_out = pos_real;
    tp->state = -1;
    tp->vel_out = vel_real_avg;
    tp->acc_brake = 0;
    tp->pos_frac = 0;
}
