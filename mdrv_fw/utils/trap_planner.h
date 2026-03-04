/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __TRAP_PLANNER_H__
#define __TRAP_PLANNER_H__

#include <math.h>
#include "cd_utils.h"

typedef struct {
    float       dt;             // planner update time step (s)
    uint16_t    max_err;        // maximum allowed position error

    // input
    int32_t     pos_tgt;        // target position
    uint32_t    vel_tgt;        // target velocity
    uint32_t    acc_tgt;        // target acceleration

    // output
    int8_t      state;          // -1: disable, 0: idle, 1: planning
    int32_t     pos_out;        // current planned position
    float       vel_out;        // current planned velocity

    // internal
    float       acc_brake;      // required braking acceleration
    float       pos_frac;       // fractional part of pos_out
} trap_planner_t;


void trap_planner_update(trap_planner_t *tp, int32_t pos_real);
void trap_planner_reset(trap_planner_t *tp, int32_t pos_real, float vel_real_avg);

#endif
