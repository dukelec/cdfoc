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
    int16_t out_pwm_u = 0, out_pwm_v = 0, out_pwm_w = 0;
    bool dbg_en = (csa.loop_cnt & csa.loop_msk) == 0;

    if (csa.loop_cnt == 0xffff)
        csa.state = ST_CALIBRATION;

    // bit mask for 2048: 0x7ff
    // 2048 × 7 = 14336

    //in_pos = EQep1Regs.QPOSCNT;
    //float angle_elec = 2.0 * PI * ((in_pos & 0x7ff) / 2048.0);

    csa.encoder_val = encoder_read() - 11602;
    if (csa.encoder_val >= 0x7fff)
        csa.encoder_val -= 0x7fff;

    uint16_t encoder_sub = csa.encoder_val % lroundf(0x7fff/11.0);

    float angle_mech = csa.encoder_val*((float)M_PI*2/0x7fff);
    csa.angle_elec_in = encoder_sub*((float)M_PI*2/(0x7fff/11.0f));

    if (dbg_en)
        d_debug("a %d.%.2d %d.%.2d %d.%.2d",
                //csa.encoder_val, encoder_sub,
                P_2F(angle_mech), P_2F(csa.angle_elec_in), P_2F(csa.angle_elec_out));

    //append_dprintf(DBG_POSITION, "pos:%08lx %08lx ", in_pos, in_pos & 0x7ff);

#if 0
    // speed filter, output in_speed for speed loop
    {
        const float coeffs[5] = {0.1, 0.15, 0.2, 0.25, 0.3};
        static float speed_avg = 0.0;
        static int speed_filt_cnt = 0;
        static int32 in_pos_last = 0;

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

    if (csa.state != ST_STOP) {
        float ia = ia_idle - HAL_ADCEx_InjectedGetValue(&hadc1, 1); // 1, 2
        float ib = ib_idle - HAL_ADCEx_InjectedGetValue(&hadc3, 1); // 3, 1
        float ic = ic_idle - HAL_ADCEx_InjectedGetValue(&hadc2, 1);
        float ic_ = -ia - ib;

        float i_alpha = ia;
        float i_beta = (ia + ib * 2) / 1.732050808f; // √3

        if (csa.state != ST_CALIBRATION)
            csa.angle_elec_out = csa.angle_elec_in;
        if (dbg_en) {
            csa.angle_elec_out += 0.01f;
            if (csa.angle_elec_out >= (float)M_PI*2)
                csa.angle_elec_out -= (float)M_PI*2;
        }

        csa.current_in = -i_alpha * sinf(csa.angle_elec_out) + i_beta * cosf(csa.angle_elec_out); // i_sq
        float i_sd = i_alpha * cosf(csa.angle_elec_out) + i_beta * sinf(csa.angle_elec_out); // i_sd

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
            d_debug_c(", in %d.%.2d %d.%.2d %d.%.2d %d.%.2d",
                    P_2F(ia), P_2F(ib), P_2F(ic), P_2F(ic_));
        if (dbg_en)
            d_debug_c(", i %d.%.2d, %d.%.2d", P_2F(csa.current_in), P_2F(i_sd));
    } else {
        // TODO: add filter
        ia_idle = HAL_ADCEx_InjectedGetValue(&hadc1, 1);
        ib_idle = HAL_ADCEx_InjectedGetValue(&hadc2, 1);
        ic_idle = HAL_ADCEx_InjectedGetValue(&hadc3, 1);
        if (dbg_en)
            d_debug_c(", in %d.%.2d %d.%.2d %d.%.2d",
                    P_2F(ia_idle), P_2F(ib_idle), P_2F(ic_idle));
    }

    // current --> pwm
    if (csa.state != ST_STOP) {
#if 0
        float angle;

        // calculate angle
        if (control_mode != M_CALIBRATION) {
            angle = angle_elec;
            append_dprintf(DBG_CURRENT, "a:%.2f ", angle);
        } else {
            angle = sub_angle_manual; // should be 0 during calibrate
            append_dprintf(DBG_CURRENT, "a_m:%.2f ", angle);
        }

        pid_set_target(&pid_cur, out_current); // TODO: only set once after modified
        float out_voltage = pid_compute_skip_d_term(&pid_cur, in_current);

        debug_current_raw(in_current, out_voltage);
        append_dprintf(DBG_CURRENT, "vo:%.2f ", out_voltage);
#endif

        float out_voltage = 300;

        float v_alpha = -out_voltage * sinf(csa.angle_elec_out);
        float v_beta =  out_voltage * cosf(csa.angle_elec_out);

        out_pwm_u = lroundf(v_alpha);
        out_pwm_v = lroundf(-v_alpha / 2 + v_beta * 0.866025404f); // (√3÷2)
        out_pwm_w = -out_pwm_u - out_pwm_v;

        /*
        append_dprintf(DBG_CURRENT, "u:%.2f %.2f %d ", pid_cur_u.target, pid_cur_u.i_term, out_pwm_u);
        append_dprintf(DBG_CURRENT, "v:%.2f %.2f %d ", pid_cur_v.target, pid_cur_v.i_term, out_pwm_v);
        append_dprintf(DBG_CURRENT, "w:%d", out_pwm_w);
        */
        if (dbg_en)
            d_debug_c(", o %d %d %d\n", out_pwm_u, out_pwm_v, out_pwm_w);

        //append_dprintf(DBG_CURRENT, "uvw:%d %d %d", out_pwm_u, out_pwm_v, out_pwm_w);
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


    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, DRV_PWM_HALF - out_pwm_u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, DRV_PWM_HALF - out_pwm_v);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, DRV_PWM_HALF - out_pwm_w);

    csa.loop_cnt++;
}
