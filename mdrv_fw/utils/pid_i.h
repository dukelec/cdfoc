/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 *
 * Notes: _i is int32_t input version
 */

#ifndef __PID_I_H__
#define __PID_I_H__

typedef struct {
    // configuration
    float kp, ki, _reserved;
    float out_min, out_max;
    float dt;

    // runtime
    int32_t target;
    float i_term;
} pid_i_t;


float pid_i_update(pid_i_t *pid, int32_t input_p, int32_t input_i);
float pid_i_update_p_only(pid_i_t *pid, int32_t input_p);


static inline void pid_i_set_target(pid_i_t *pid, int32_t target)
{
    pid->target = target;
}

static inline void pid_i_reset(pid_i_t *pid, float output)
{
    pid->i_term = clip(output, pid->out_min, pid->out_max);
}

#endif
