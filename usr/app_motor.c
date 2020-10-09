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

static cdn_sock_t pos_sock = { .port = 20, .ns = &dft_ns };
static cdn_sock_t speed_sock = { .port = 21, .ns = &dft_ns };

void app_motor_init(void)
{
    pid_f_init(&csa.pid_cur, true);
    pid_f_init(&csa.pid_speed, true);
    pid_i_init(&csa.pid_pos, true);
}


static void pos_mode_service_routine(void)
{
}

static void speed_mode_service_routine(void)
{
}


void app_motor(void)
{
    pos_mode_service_routine();
    speed_mode_service_routine();

}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{

    static float ia_idle, ib_idle, ic_idle;
    float sin_sen_angle_elec, cos_sen_angle_elec; // reduce the amount of calculations
    int16_t out_pwm_u = 0, out_pwm_v = 0, out_pwm_w = 0;
    bool dbg_str = (csa.loop_cnt & csa.dbg_str_msk) == 0;

    if (csa.loop_cnt == 0xffff)
        csa.state = ST_CONST_SPEED;

    // bit mask for 2048: 0x7ff
    // 2048 × 7 = 14336

    //in_pos = EQep1Regs.QPOSCNT;
    //float angle_elec = 2.0 * PI * ((in_pos & 0x7ff) / 2048.0);

    csa.ori_encoder = encoder_read();
    csa.sen_encoder = csa.ori_encoder - csa.bias_encoder;

    if (abs((uint16_t)(csa.ori_pos & 0xffff) - csa.sen_encoder) > 0x10000/2) {
        if (csa.sen_encoder < 0x10000/2)
            csa.ori_pos += 0x10000;
        else
            csa.ori_pos -= 0x10000;
    }
    csa.ori_pos = (csa.ori_pos & 0xffff0000) | csa.sen_encoder;
    csa.sen_pos = csa.ori_pos - csa.bias_pos;

    if (dbg_str)
            d_debug("%04x %08x", csa.ori_encoder, csa.sen_pos);


    uint16_t encoder_sub = csa.sen_encoder % lroundf(0x10000/21.0f);

    float angle_mech = csa.sen_encoder*((float)M_PI*2/0x10000);
    csa.sen_angle_elec = encoder_sub*((float)M_PI*2/(0x10000/21.0f));
    sin_sen_angle_elec = sinf(csa.sen_angle_elec);
    cos_sen_angle_elec = cosf(csa.sen_angle_elec);

    if (dbg_str)
        d_debug_c(" a %d.%.2d %d.%.2d %d.%.2d", P_2F(angle_mech), P_2F(csa.sen_angle_elec), P_2F(csa.cali_angle_elec));

    if (csa.state != ST_STOP) {
        float ia = HAL_ADCEx_InjectedGetValue(&hadc1, 1) - ia_idle;
        float ib = HAL_ADCEx_InjectedGetValue(&hadc2, 1) - ib_idle;
        float ic = HAL_ADCEx_InjectedGetValue(&hadc3, 1) - ic_idle;
        float ic_ = -ia - ib;

        float i_alpha = ia;
        float i_beta = (ia + ib * 2) / 1.732050808f; // √3

        /*
        if (csa.state != ST_CALI)
            csa.cali_angle_elec = csa.sen_angle_elec;
        */


        /*
        //if (dbg_str) {
            csa.cali_angle_elec += 0.002f;
            if (csa.cali_angle_elec >= (float)M_PI*2)
                csa.cali_angle_elec -= (float)M_PI*2;
            //csa.cali_angle_elec = 0;
        //}
         *
         */


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
            d_debug_c(", in %5d.%.2d %5d.%.2d %5d.%.2d %5d.%.2d", P_2F(ia), P_2F(ib), P_2F(ic), P_2F(ic_));
        if (dbg_str)
            d_debug_c(", i %5d.%.2d, %5d.%.2d", P_2F(csa.sen_current), P_2F(i_sd));
    } else {
        // TODO: add filter
        ia_idle = HAL_ADCEx_InjectedGetValue(&hadc1, 1);
        ib_idle = HAL_ADCEx_InjectedGetValue(&hadc2, 1);
        ic_idle = HAL_ADCEx_InjectedGetValue(&hadc3, 1);
        if (dbg_str)
            d_debug_c(", in %5d.%.2d %5d.%.2d %5d.%.2d", P_2F(ia_idle), P_2F(ib_idle), P_2F(ic_idle));
    }

    // current --> pwm
    if (csa.state != ST_STOP) {
        float i_sq_out, i_alpha, i_beta;

        // calculate angle
        if (csa.state != ST_CALI) {
            pid_f_set_target(&csa.pid_cur, csa.cal_current); // TODO: only set once after modified
            i_sq_out = pid_f_compute_no_d(&csa.pid_cur, csa.sen_current);
            i_alpha = -i_sq_out * sin_sen_angle_elec;
            i_beta =  i_sq_out * cos_sen_angle_elec;
        } else {
            i_sq_out = 200; ///
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
        // TODO: not only init in STOP
#if 0
        pid_reset(&pid_cur, 0.0, 0.0);
        pid_reset(&pid_speed, 0.0, 0.0);
        pid_reset(&pid_pos, csa.sen_pos, 0);
        peak_cur_cnt = 0;
#endif
        if (dbg_str)
            d_debug_c("\n");
    }

    gpio_set_value(&led_r, !gpio_get_value(&led_r));

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, DRV_PWM_HALF - out_pwm_u); // TIM1_CH3: A
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, DRV_PWM_HALF - out_pwm_v); // TIM1_CH2: B
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, DRV_PWM_HALF - out_pwm_w); // TIM1_CH1: C

#if 1
    // speed filter, output in_speed for speed loop
    {
        const float coeffs[5] = {0.1, 0.15, 0.2, 0.25, 0.3};
        static float speed_avg = 0.0;
        static int speed_filt_cnt = 0;
        static int sen_pos_last = 0;

        float speed = (sen_pos_last - csa.sen_pos) * CURRENT_LOOP_FREQ * 60 / 0x10000;
        sen_pos_last = csa.sen_pos;

        speed_avg += speed * coeffs[speed_filt_cnt++];

        if (speed_filt_cnt == 5) {
            csa.sen_speed = speed_avg;
            speed_avg = 0;
            speed_filt_cnt = 0;

            csa.cal_speed = csa.state == ST_CONST_SPEED ? -1 : 0;

            pid_f_set_target(&csa.pid_speed, csa.cal_speed); // TODO: only set once after modified
            csa.cal_current = pid_f_compute_no_d(&csa.pid_speed, csa.sen_speed);
        }
    }
#endif
    csa.loop_cnt++;
}
