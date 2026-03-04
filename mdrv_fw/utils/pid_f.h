/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
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
    float kp, ki, _reserved;
    float out_min, out_max;
    float dt;

    // runtime
    float target;
    float i_term;
} pid_f_t;


float pid_f_update(pid_f_t *pid, float input_p, float input_i);


static inline void pid_f_set_target(pid_f_t *pid, float target)
{
    pid->target = target;
}

static inline void pid_f_reset(pid_f_t *pid, float output)
{
    pid->i_term = clip(output, pid->out_min, pid->out_max);
}

#endif
