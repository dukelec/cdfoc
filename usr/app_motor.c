/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#include "math.h"
#include "app_main.h"

static uint16_t last_encoder;


void app_motor_init(void)
{
    pid_f_init(&csa.pid_cur, true);
    pid_f_init(&csa.pid_speed, true);
    pid_i_init(&csa.pid_pos, true);

    last_encoder = encoder_read();
}

static inline void position_loop_compute(void)
{
    if (csa.state < ST_POSITION) {
        csa.tc_run = false;
        pid_i_reset(&csa.pid_pos, csa.sen_pos, 0);
        csa.cal_pos = csa.sen_pos;
        csa.tc_pos = csa.sen_pos;
        return;
    }

    if (!csa.tc_run) {
        csa.cal_pos = csa.tc_pos;
        pid_i_set_target(&csa.pid_pos, csa.cal_pos);

    } else if (csa.tc_cnt >= csa.tc_steps) {
        csa.tc_run = false;
        csa.cal_pos = csa.tc_pos;
        pid_i_set_target(&csa.pid_pos, csa.cal_pos);

    } else {
        t_curve_step(csa.tc_s_seg, csa.tc_t_seg, csa.tc_a_seg,
                csa.tc_v_s, 25.0 / CURRENT_LOOP_FREQ * csa.tc_cnt, // i * period
                &csa.tc_v_cur, &csa.tc_s_cur);
        csa.tc_cnt++;
        pid_i_set_target(&csa.pid_pos, lroundf(csa.tc_s_s + csa.tc_s_cur));
    }

    csa.cal_speed = lroundf(pid_i_compute(&csa.pid_pos, csa.sen_pos));
}

static inline void speed_loop_compute(void)
{
    static int sub_cnt = 0;
    const float coeffs[5] = {0.1, 0.15, 0.2, 0.25, 0.3};
    static float speed_avg = 0.0;
    static int speed_filt_cnt = 0;

    float speed = csa.delta_encoder * CURRENT_LOOP_FREQ; // encoder steps per sec
    speed_avg += speed * coeffs[speed_filt_cnt++];

    if (speed_filt_cnt == 5) {

        csa.sen_speed = lroundf(speed_avg);
        speed_avg = 0;
        speed_filt_cnt = 0;

        if (csa.state < ST_CONST_SPEED) {
            pid_f_reset(&csa.pid_speed, csa.sen_speed, 0);
            csa.cal_speed = csa.sen_speed;
        } else {
            pid_f_set_target(&csa.pid_speed, csa.cal_speed); // TODO: only set once after modified
            csa.cal_current = -lroundf(pid_f_compute_no_d(&csa.pid_speed, csa.sen_speed));
        }

        if (++sub_cnt == 5) {
            sub_cnt = 0;
            position_loop_compute();
        }
    }
}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    static int skip_cnt = 0;
    static float ia_idle, ib_idle;
    float sin_sen_angle_elec, cos_sen_angle_elec; // reduce the amount of calculations
    int16_t out_pwm_u = 0, out_pwm_v = 0, out_pwm_w = 0;
    bool dbg_str = (csa.loop_cnt & csa.dbg_str_msk) == 0;
    //dbg_str = 0;

    if (csa.loop_cnt == 0xffff) {
        csa.state = ST_CALI; //////////////////////////////
        //csa.cal_current = 600;
        //csa.cal_speed = 1000;
    }

    csa.ori_encoder = encoder_read();
    csa.noc_encoder = csa.ori_encoder - csa.bias_encoder;
    int16_t delta_enc = csa.noc_encoder - last_encoder;

#if 0
    if (abs(delta_enc) < 1000 || skip_cnt != 0) { // a good value?
        if (dbg_str && abs(delta_enc) >= 1000)
            d_debug("rst! big delta_enc: %d (%x - %x) | %d\n",
                    delta_enc, csa.noc_encoder, last_encoder, csa.delta_encoder);
        csa.delta_encoder = delta_enc;
        // (4Bytes 5.25M, 1/8 CURRENT_LOOP) was wrong
        // encoder lock data at start of first byte
        csa.sen_encoder = csa.noc_encoder + DIV_ROUND_CLOSEST(delta_enc * 1, 8);
        last_encoder = csa.noc_encoder;
        skip_cnt = 0;
    } else { // skip wrong sensor value
        //if (dbg_str)
        //    d_debug("skip big delta_enc: %d (%x - %x) | %d\n",
        //            delta_enc, csa.noc_encoder, last_encoder, csa.delta_encoder);
        csa.sen_encoder += csa.delta_encoder;
        last_encoder += csa.delta_encoder;
        skip_cnt++;
    }
#else
    csa.sen_encoder = csa.noc_encoder;
    csa.delta_encoder = delta_enc;
    last_encoder = csa.noc_encoder;
#endif

    if (abs((uint16_t)(csa.ori_pos & 0xffff) - csa.sen_encoder) > 0x10000/2) {
        if (csa.sen_encoder < 0x10000/2)
            csa.ori_pos += 0x10000;
        else
            csa.ori_pos -= 0x10000;
    }
    csa.ori_pos = (csa.ori_pos & 0xffff0000) | csa.sen_encoder;
    csa.sen_pos = csa.ori_pos - csa.bias_pos;

    if (dbg_str)
            d_debug("%04x %d %08x", csa.ori_encoder, delta_enc, csa.sen_pos);

    float angle_mech = csa.sen_encoder*((float)M_PI*2/0x10000);
    uint16_t encoder_sub = csa.sen_encoder % lroundf(0x10000/21.0f);
    csa.sen_angle_elec = encoder_sub*((float)M_PI*2/(0x10000/21.0f));
    sin_sen_angle_elec = sinf(csa.sen_angle_elec);
    cos_sen_angle_elec = cosf(csa.sen_angle_elec);

    if (dbg_str)
        d_debug_c(" a %d.%.2d %d.%.2d %d.%.2d", P_2F(angle_mech), P_2F(csa.sen_angle_elec), P_2F(csa.cali_angle_elec));

    if (csa.state != ST_STOP) {
        int32_t ia = HAL_ADCEx_InjectedGetValue(&hadc1, 1) - ia_idle;
        int32_t ib = HAL_ADCEx_InjectedGetValue(&hadc2, 1) - ib_idle;
        int32_t ic = -ia - ib;
        HAL_ADCEx_InjectedGetValue(&hadc3, 1); // remove this

        float i_alpha = ia;
        float i_beta = (ia + ib * 2) / 1.732050808f; // √3

        csa.sen_current = -i_alpha * sin_sen_angle_elec + i_beta * cos_sen_angle_elec; // i_sq
        float i_sd = i_alpha * cos_sen_angle_elec + i_beta * sin_sen_angle_elec; // i_sd

        /*
        if (in_current > peak_cur_threshold) {
            if (++peak_cur_cnt >= peak_cur_duration) {
                error_flag = ERR_CUR_PROTECT;
                control_mode = M_STOP;
                dprintf(DBG_ERROR, "err: current protected, in_cur: %.3f, threshold: %.3f",
                        in_current, peak_cur_threshold);
            }
        } else
            peak_cur_cnt = 0;
        */
        if (dbg_str)
            d_debug_c(", in %5d %5d %5d", ia, ib, ic);
        if (dbg_str)
            d_debug_c(", i %5d.%.2d, %5d.%.2d", P_2F(csa.sen_current), P_2F(i_sd));

    } else { // state: stop
        static int i_cnt = 0;
        static int32_t ia_sum = 0;
        static int32_t ib_sum = 0;
        ia_sum += HAL_ADCEx_InjectedGetValue(&hadc1, 1);
        ib_sum += HAL_ADCEx_InjectedGetValue(&hadc2, 1);
        HAL_ADCEx_InjectedGetValue(&hadc3, 1);

        if (++i_cnt == 5) {
            ia_idle = DIV_ROUND_CLOSEST(ia_sum, 5);
            ib_idle = DIV_ROUND_CLOSEST(ib_sum, 5);
            ia_sum = ib_sum = 0;
            i_cnt = 0;
        }
        if (dbg_str)
            d_debug_c(", in %5d.%.2d %5d.%.2d", P_2F(ia_idle), P_2F(ib_idle));
    }

    // current --> pwm
    if (csa.state != ST_STOP) {
        float i_sq_out, i_alpha, i_beta;

        // calculate angle
        if (csa.state != ST_CALI) {
            pid_f_set_target(&csa.pid_cur, csa.cal_current);
            i_sq_out = pid_f_compute_no_d(&csa.pid_cur, csa.sen_current);
            i_alpha = -i_sq_out * sin_sen_angle_elec;
            i_beta =  i_sq_out * cos_sen_angle_elec;

        } else {
            csa.cali_angle_elec += csa.cali_angle_step;
            if (csa.cali_angle_elec >= (float)M_PI*2)
                csa.cali_angle_elec -= (float)M_PI*2;

            i_sq_out = csa.cali_current;
            i_alpha = -i_sq_out * sinf(csa.cali_angle_elec);
            i_beta =  i_sq_out * cosf(csa.cali_angle_elec);
        }

        out_pwm_u = lroundf(i_alpha);
        out_pwm_v = lroundf(-i_alpha / 2 + i_beta * 0.866025404f); // (√3÷2)
        out_pwm_w = -out_pwm_u - out_pwm_v;

        /*
        append_dprintf(DBG_CURRENT, "u:%.2f %.2f %d ", pid_cur_u.target, pid_cur_u.i_term, out_pwm_u);
        append_dprintf(DBG_CURRENT, "v:%.2f %.2f %d ", pid_cur_v.target, pid_cur_v.i_term, out_pwm_v);
        append_dprintf(DBG_CURRENT, "w:%d", out_pwm_w);
        */
        if (dbg_str)
            d_debug_c(", o %5d.%.2d %5d %5d %5d\n", P_2F(i_sq_out), out_pwm_u, out_pwm_v, out_pwm_w);
    } else {
        pid_f_reset(&csa.pid_cur, csa.sen_current, 0.0);
        csa.cal_current = 0;

        //peak_cur_cnt = 0;
        if (dbg_str)
            d_debug_c("\n");
    }

    //gpio_set_value(&led_r, !gpio_get_value(&led_r)); // debug for hw config

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, DRV_PWM_HALF - out_pwm_u); // TIM1_CH3: A
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, DRV_PWM_HALF - out_pwm_v); // TIM1_CH2: B
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, DRV_PWM_HALF - out_pwm_w); // TIM1_CH1: C

    speed_loop_compute();
    csa.loop_cnt++;
}
