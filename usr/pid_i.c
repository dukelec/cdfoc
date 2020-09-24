/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2016, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 * Modified by: Duke Fong <duke@dukelec.com>
 */

#include "cd_utils.h"
#include "pid_i.h"

float pid_i_compute(pid_i_t *pid, int input)
{
    int error, delta_input;
    float output;

    error = pid->target - input;

    pid->i_term += pid->_ki * error;
    pid->i_term = clip(pid->i_term, pid->out_min, pid->out_max);

    delta_input = input - pid->last_input; // delta_input = -delta_error
    pid->last_input = input;

    output = pid->kp * error + pid->i_term - pid->_kd * delta_input;
    output = clip(output, pid->out_min, pid->out_max);
    return output;
}

float pid_i_compute_no_d(pid_i_t *pid, int input)
{
    int error;
    float output;

    error = pid->target - input;

    pid->i_term += pid->_ki * error;
    pid->i_term = clip(pid->i_term, pid->out_min, pid->out_max);

    output = pid->kp * error + pid->i_term;
    output = clip(output, pid->out_min, pid->out_max);
    return output;
}


void pid_i_reset(pid_i_t *pid, int input, float output)
{
    pid->last_input = input;
    pid->i_term = clip(output, pid->out_min, pid->out_max);
}

void pid_i_init(pid_i_t *pid, bool reset)
{
    pid->_ki = pid->ki * pid->period;
    pid->_kd = pid->kd / pid->period;

    if (reset)
        pid_i_reset(pid, 0, 0);
}
