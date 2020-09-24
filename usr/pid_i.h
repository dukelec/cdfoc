/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2016, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 * Modified by: Duke Fong <duke@dukelec.com>
 */

#ifndef __PID_I_H__
#define __PID_I_H__

typedef struct {
    // configuration
    float kp, ki, kd;
    float out_min, out_max;
    float period;

    int target;

    // runtime and internal
    float i_term;
    int last_input;
    float _ki, _kd;
} pid_i_t;

float pid_i_compute(pid_i_t *pid, int input);

float pid_i_compute_no_d(pid_i_t *pid, int input);

inline void pid_i_set_target(pid_i_t *pid, int target)
{
    pid->target = target;
}

void pid_i_reset(pid_i_t *pid, int input, float output);

// invoke after ki or kd changed
void pid_i_init(pid_i_t *pid, bool reset);

#endif
