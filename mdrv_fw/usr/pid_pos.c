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

#include "cd_utils.h"
#include "pid_pos.h"
#include "app_main.h"


float pid_i_compute(pid_i_t *pid, int input)
{
    int error = pid->target - input;
    float i_del = pid->_ki * error;
    float output = pid->kp * error + pid->i_term; // old i_term

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


void pid_i_reset(pid_i_t *pid, int input, float output)
{
    pid->i_term = clip(output, pid->out_min, pid->out_max);
}

void pid_i_init(pid_i_t *pid, bool reset)
{
    pid->_ki = pid->ki * pid->period;
    //pid->_kd = pid->kd / pid->period;

    if (reset)
        pid_i_reset(pid, 0, 0);
}
