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

#include "cd_utils.h"
#include "pid_f.h"

float pid_f_compute(pid_f_t *pid, float input4p, float input4i)
{
    float error4p = pid->target - input4p;
    float error4i = pid->target - input4i;
    float i_del = pid->_ki * error4i;
    float output = pid->kp * error4p + pid->i_term;

    if (output >= pid->out_max) {
        if (i_del < 0)
            pid->i_term += i_del;
    } else if (output <= pid->out_min) {
        if (i_del > 0)
            pid->i_term += i_del;
    } else {
        pid->i_term += i_del;
        output += i_del;
    }
    pid->i_term = clip(pid->i_term, pid->out_min, pid->out_max);
    return clip(output, pid->out_min, pid->out_max);
}


void pid_f_reset(pid_f_t *pid, float input, float output)
{
    pid->i_term = clip(output, pid->out_min, pid->out_max);
}

void pid_f_init(pid_f_t *pid, bool reset)
{
    pid->_ki = pid->ki * pid->period;

    if (reset)
        pid_f_reset(pid, 0, 0);
}
