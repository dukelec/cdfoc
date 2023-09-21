/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2016, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 * Modified by: Duke Fong <d@d-l.io>
 *
 * Notes: _f is float type version
 */

#ifndef __PID_F_H__
#define __PID_F_H__

typedef struct {
    // configuration
    float kp, ki, kd;
    float out_min, out_max;
    float period;

    float target;

    // runtime and internal
    float i_term, last_input;
    float _ki, _kd;
    
    int filter_len;
    float filter_hist[5];
} pid_f_t;

float pid_f_compute(pid_f_t *pid, float input);

float pid_f_compute_no_d(pid_f_t *pid, float input);

inline void pid_f_set_target(pid_f_t *pid, float target)
{
    pid->target = target;
}

void pid_f_reset(pid_f_t *pid, float input, float output);

// invoke after ki or kd changed
void pid_f_init(pid_f_t *pid, bool reset);

#endif
