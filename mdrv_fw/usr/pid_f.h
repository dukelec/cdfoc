/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2016, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 *
 * Notes: _f is float input version
 */

#ifndef __PID_F_H__
#define __PID_F_H__

typedef struct {
    // configuration
    float kp, ki;
    float _reserved0;
    float out_min, out_max;
    float period;

    float target;

    // runtime and internal
    float i_term;
    float _reserved1;
    float _ki;
    float _reserved2[7];
} pid_f_t;


float pid_f_compute(pid_f_t *pid, float input4p, float input4i);

inline void pid_f_set_target(pid_f_t *pid, float target)
{
    pid->target = target;
}

void pid_f_reset(pid_f_t *pid, float input, float output);

// invoke after ki or kd changed
void pid_f_init(pid_f_t *pid, bool reset);

#endif
