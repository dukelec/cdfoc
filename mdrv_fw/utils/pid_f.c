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

#include "cd_utils.h"
#include "pid_f.h"


float pid_f_update(pid_f_t *pid, float input_p, float input_i)
{
    float error_p = pid->target - input_p;
    float error_i = pid->target - input_i;
    float i_del = pid->ki * error_i * pid->dt;
    float output = pid->kp * error_p + pid->i_term;

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

