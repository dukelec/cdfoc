/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2016, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 * Modified by: Duke Fong <duke@dukelec.com>
 */

#ifndef __PID_M_H__
#define __PID_M_H__

typedef struct {
    // configuration
    float kp, ki, kd;
    float out_min, out_max;
    float period;

    float target;

    // runtime and internal
    float i_term, last_input;
    float _ki, _kd;
} pid_m_t;

float pid_compute(pid_m_t *pid, float input);

float pid_compute_no_d(pid_m_t *pid, float input);

inline void pid_set_target(pid_m_t *pid, float target)
{
    pid->target = target;
}

void pid_reset(pid_m_t *pid, float input, float output);

// invoke after ki or kd changed
void pid_init(pid_m_t *pid, bool reset);

#endif
