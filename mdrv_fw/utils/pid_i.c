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

#include "cd_utils.h"
#include "pid_i.h"


float pid_i_update(pid_i_t *pid, int32_t input_p, int32_t input_i)
{
    int32_t error_p = pid->target - input_p;
    int32_t error_i = pid->target - input_i;
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


float pid_i_update_p_only(pid_i_t *pid, int32_t input_p)
{
    int32_t error_p = pid->target - input_p;
    float output = pid->kp * error_p;
    return clip(output, pid->out_min, pid->out_max);
}

