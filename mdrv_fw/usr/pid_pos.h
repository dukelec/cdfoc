/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2016, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 *
 * Notes: _i is int input version
 */

#ifndef __PID_POS_H__
#define __PID_POS_H__

typedef struct {
    // configuration
    float kp, ki, kd;
    float out_min, out_max;
    float period;

    int target;

    // runtime and internal
    float i_term;
    int _reserved0;
    float _ki, _kd;
    int _reserved1[6];
} pid_i_t;

float pid_i_compute(pid_i_t *pid, int input);

inline void pid_i_set_target(pid_i_t *pid, int target)
{
    pid->target = target;
}

void pid_i_reset(pid_i_t *pid, int input, float output);

// invoke after ki or kd changed
void pid_i_init(pid_i_t *pid, bool reset);

#endif
