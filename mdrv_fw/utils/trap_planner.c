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
        tp->pos_frac = 0;
        return;
    }

    float dt = tp->dt;
    float dist = (float)(tp->pos_tgt - tp->pos_out) - tp->pos_frac;
    bool rev = dist < 0;
    float d = rev ? -dist : dist;               // remaining distance, non-negative
    float v = rev ? -tp->vel_out : tp->vel_out; // velocity component toward target

    // Stepping down by acc_tgt*dt per tick from velocity v covers
    // v^2/(2a) + v*dt/2, not v^2/(2a); the half-step term makes the
    // envelope exact in discrete time. Invert for the max velocity
    // that can still stop within d:
    float h = 0.5f * tp->acc_tgt * dt;
    float v_stop = sqrtf(h * h + 2.0f * tp->acc_tgt * d) - h;

    float v_des = min((float)tp->vel_tgt, v_stop);
    if (v < v_des)
        v = min(v + tp->acc_tgt * dt, v_des);
    else
        v = max(v - tp->acc_tgt * dt, v_des);

    // v <= v_stop rules out fly-by when over-speed (planner then passes
    // the target and comes back, never exceeding acc_tgt)
    bool landing = (v <= v_stop && v * dt >= d);
    if (landing)
        v = d / dt; // land exactly on target this tick

    tp->vel_out = rev ? -v : v;

    tp->pos_frac += tp->vel_out * dt;
    int32_t tmp = lroundf(tp->pos_frac);
    tp->pos_frac -= tmp;
    int32_t pos_next = tp->pos_out + tmp;

    if (tp->max_err && abs(pos_real - pos_next) > tp->max_err)
        pos_next = clip(pos_next, pos_real - tp->max_err, pos_real + tp->max_err);

    tp->pos_out = pos_next;

    // keep the final vel_out (<= acc_tgt * dt) this tick so downstream
    // feed-forward ramps down smoothly; the !state branch clears it next tick
    if (landing && tp->pos_out == tp->pos_tgt)
        tp->state = 0;
}


void trap_planner_reset(trap_planner_t *tp, int32_t pos_real, float vel_real_avg)
{
    tp->pos_tgt = pos_real;
    tp->pos_out = pos_real;
    tp->state = -1;
    tp->vel_out = vel_real_avg;
    tp->pos_frac = 0;
}
