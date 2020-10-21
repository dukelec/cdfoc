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
#include "app_main.h"

#define HIST_LEN 3
static int32_t hist[HIST_LEN] = { 0 };
static float di_avg = 0;

float pid_i_compute(pid_i_t *pid, int input)
{
    int error, delta_input;
    float output;

    error = pid->target - input;

    pid->i_term += pid->_ki * error;
    //pid->i_term = clip(pid->i_term, pid->out_min, pid->out_max);
    //                                                            %         max_out         %
    float i_lim = (float)min(abs(csa.tc_pos - csa.cal_pos) + 60, 20000) * pid->out_max / 20000.0f;
    pid->i_term = clip(pid->i_term, -i_lim, i_lim);

    //float kp_term = (csa.tc_state == 0 ? (pid->kp / 2.0f) : pid->kp) * error;
    //float kp_term = (csa.tc_state == 0 ? (pid->kp * 0.6f) : pid->kp) * error;
    float kp_term = pid->kp * error;
    //                                                           %      max_out     %
    //float kp_term = (min(abs(csa.tc_pos - csa.cal_pos) + 5000, 20000) * pid->kp / 20000) * error;


    delta_input = input - pid->last_input; // delta_input = -delta_error
    pid->last_input = input;

    for (int i = 0; i < HIST_LEN; i++)
        di_avg += hist[i];
    di_avg = di_avg / (float)HIST_LEN;

    //for (int i = 0; i < HIST_LEN; i++)
    //    di_avg += hist[i];
    //di_avg = DIV_ROUND_CLOSEST(di_avg, HIST_LEN);

    output = kp_term + pid->i_term - pid->_kd * di_avg;
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
    pid->last_input = input;

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
