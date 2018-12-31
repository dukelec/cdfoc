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

static cdnet_socket_t pos_sock = { .port = 20 };
static cdnet_socket_t speed_sock = { .port = 21 };

void app_motor_init(void)
{
    pid_init(&csa.pid_cur, true);
    pid_init(&csa.pid_speed, true);
    pid_init(&csa.pid_pos, true);
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
    float sin_angle_elec_in, cos_angle_elec_in; // reduce the amount of calculations
    int16_t out_pwm_u = 0, out_pwm_v = 0, out_pwm_w = 0;
    bool dbg_en = (csa.loop_cnt & csa.loop_msk) == 0;

    if (csa.loop_cnt == 0xffff)
        csa.state = ST_CONST_CURRENT;

    // bit mask for 2048: 0x7ff
    // 2048 × 7 = 14336

    //in_pos = EQep1Regs.QPOSCNT;
    //float angle_elec = 2.0 * PI * ((in_pos & 0x7ff) / 2048.0);

    uint16_t encoder_ori = encoder_read() << 1;
    csa.encoder_val = 0xfffe - (encoder_ori - 0x58d8);

    static int32_t pos = 0;

    if (abs((uint16_t)(pos & 0xffff) - csa.encoder_val) > 0x10000/2) {
        if (csa.encoder_val < 0x10000/2)
            pos += 0x10000;
        else
            pos -= 0x10000;
    }
    pos  = (pos & 0xffff0000) | csa.encoder_val;

    if (dbg_en)
            d_debug("%04x %08x", encoder_ori, pos);


    uint16_t encoder_sub = csa.encoder_val % lroundf(0x10000/11.0);

    float angle_mech = csa.encoder_val*((float)M_PI*2/0x10000);
    csa.angle_elec_in = encoder_sub*((float)M_PI*2/(0x10000/11.0f));
    sin_angle_elec_in = sinf(csa.angle_elec_in);
    cos_angle_elec_in = cosf(csa.angle_elec_in);

    if (dbg_en)
        d_debug_c(" a %d.%.2d %d.%.2d %d.%.2d",
                //csa.encoder_val, encoder_sub,
                P_2F(angle_mech), P_2F(csa.angle_elec_in), P_2F(csa.angle_elec_out));

    if (csa.state != ST_STOP) {
        float ia = HAL_ADCEx_InjectedGetValue(&hadc1, 1) - ia_idle;
        float ib = HAL_ADCEx_InjectedGetValue(&hadc2, 1) - ib_idle;
        float ic = HAL_ADCEx_InjectedGetValue(&hadc3, 1) - ic_idle;
        float ic_ = -ia - ib;

        float i_alpha = ia;
        float i_beta = (ia + ib * 2) / 1.732050808f; // √3

        /*
        if (csa.state != ST_CALIBRATION)
            csa.angle_elec_out = csa.angle_elec_in;
        if (dbg_en) {
            csa.angle_elec_out += 0.01f;
            if (csa.angle_elec_out >= (float)M_PI*2)
                csa.angle_elec_out -= (float)M_PI*2;
        }*/

        csa.current_in = -i_alpha * sin_angle_elec_in + i_beta * cos_angle_elec_in; // i_sq
        float i_sd = i_alpha * cos_angle_elec_in + i_beta * sin_angle_elec_in; // i_sd

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
        if (dbg_en)
            d_debug_c(", in %5d.%.2d %5d.%.2d %5d.%.2d %5d.%.2d",
                    P_2F(ia), P_2F(ib), P_2F(ic), P_2F(ic_));
        if (dbg_en)
            d_debug_c(", i %5d.%.2d, %5d.%.2d", P_2F(csa.current_in), P_2F(i_sd));
    } else {
        // TODO: add filter
        ia_idle = HAL_ADCEx_InjectedGetValue(&hadc1, 1);
        ib_idle = HAL_ADCEx_InjectedGetValue(&hadc2, 1);
        ic_idle = HAL_ADCEx_InjectedGetValue(&hadc3, 1);
        if (dbg_en)
            d_debug_c(", in %5d.%.2d %5d.%.2d %5d.%.2d",
                    P_2F(ia_idle), P_2F(ib_idle), P_2F(ic_idle));
    }

    // current --> pwm
    if (csa.state != ST_STOP) {
        float i_sq_out, i_alpha, i_beta;

        // calculate angle
        if (csa.state != ST_CALIBRATION) {
            pid_set_target(&csa.pid_cur, 100); // TODO: only set once after modified
            i_sq_out = pid_compute_no_d(&csa.pid_cur, csa.current_in);
            i_alpha = -i_sq_out * sin_angle_elec_in;
            i_beta =  i_sq_out * cos_angle_elec_in;
        } else {
            i_sq_out = 500; ///
            i_alpha = -i_sq_out * sinf(csa.angle_elec_out);
            i_beta =  i_sq_out * cosf(csa.angle_elec_out);
        }

        out_pwm_u = lroundf(i_alpha);
        out_pwm_v = lroundf(-i_alpha / 2 + i_beta * 0.866025404f); // (√3÷2)
        out_pwm_w = -out_pwm_u - out_pwm_v;

        /*
        append_dprintf(DBG_CURRENT, "u:%.2f %.2f %d ", pid_cur_u.target, pid_cur_u.i_term, out_pwm_u);
        append_dprintf(DBG_CURRENT, "v:%.2f %.2f %d ", pid_cur_v.target, pid_cur_v.i_term, out_pwm_v);
        append_dprintf(DBG_CURRENT, "w:%d", out_pwm_w);
        */
        if (dbg_en)
            d_debug_c(", o %5d.%.2d %5d %5d %5d\n", P_2F(i_sq_out), out_pwm_u, out_pwm_v, out_pwm_w);
    } else {
        // TODO: not only init in STOP
#if 0
        pid_reset(&pid_cur, 0.0, 0.0);
        pid_reset(&pid_speed, 0.0, 0.0);
        pid_reset(&pid_pos, in_pos, 0);
        peak_cur_cnt = 0;
#endif
        if (dbg_en)
            d_debug_c("\n");
    }

    gpio_set_value(&led_r, !gpio_get_value(&led_r));

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, DRV_PWM_HALF - out_pwm_u); // TIM1_CH3: A
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, DRV_PWM_HALF - out_pwm_v); // TIM1_CH2: B
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, DRV_PWM_HALF - out_pwm_w); // TIM1_CH1: C

#if 0
    // speed filter, output in_speed for speed loop
    {
        const float coeffs[5] = {0.1, 0.15, 0.2, 0.25, 0.3};
        static float speed_avg = 0.0;
        static int speed_filt_cnt = 0;
        static int in_pos_last = 0;

        float speed = (in_pos_last - in_pos) * CURRENT_LOOP_FREQ / 14336.0;
        in_pos_last = in_pos;

        speed_avg += speed * coeffs[speed_filt_cnt++];

        if (speed_filt_cnt == 5) {
            in_speed = speed_avg;
            speed_filt_cnt = 0;
            speed_avg = 0;
            speed_loop_compute();
        }
    }
#endif
    csa.loop_cnt++;
}
