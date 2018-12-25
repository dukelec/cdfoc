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

    static float in_idle_pid_cur_a, in_idle_pid_cur_b;
    int16_t out_pwm_u = 0, out_pwm_v = 0, out_pwm_w = 0;


    // bit mask for 2048: 0x7ff
    // 2048 × 7 = 14336

    //in_pos = EQep1Regs.QPOSCNT;
    //float angle_elec = 2.0 * PI * ((in_pos & 0x7ff) / 2048.0);

    csa.encoder_val = encoder_read() - 11602;
    if (csa.encoder_val >= 0x7fff)
        csa.encoder_val -= 0x7fff;

    uint16_t encoder_sub = csa.encoder_val % (uint16_t)(0x7fff/11.0+0.5);

    float angle_mech = csa.encoder_val*360.0f/0x7fff;
    csa.angle_elec = encoder_sub*360.0f/(0x7fff/11.0f);

    if ((csa.loop_cnt & csa.loop_msk) == 0) {
        d_debug("a: %d, %d | %d.%.2d,  %d.%.2d\n",
                csa.encoder_val, encoder_sub,
                (int)(angle_mech), (int)((angle_mech - (int)angle_mech)*100),
                (int)(csa.angle_elec), (int)((csa.angle_elec - (int)csa.angle_elec)*100));
    }

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

#if 0
    if (control_mode != M_STOP) {
        float ia = (float)AdcResult.ADCRESULT2 * ADC2MA - in_idle_pid_cur_a;
        float ib = (float)AdcResult.ADCRESULT1 * ADC2MA - in_idle_pid_cur_b;
        float ic = -ia - ib;

        float i_alpha = ia;
        float i_beta = (ia + ib * 2) / 1.732050808; // √3

        if (control_mode == M_CALIBRATION)
            angle_elec = sub_angle_manual;

        in_current = -i_alpha * sin(angle_elec) + i_beta * cos(angle_elec); // i_sq

        if (in_current > peak_cur_threshold) {
            if (++peak_cur_cnt >= peak_cur_duration) {
                error_flag = ERR_CUR_PROTECT;
                control_mode = M_STOP;
                dprintf(DBG_ERROR, "err: current protected, in_cur: %.3f, threshold: %.3f",
                        in_current, peak_cur_threshold);
            }
        } else
            peak_cur_cnt = 0;

        append_dprintf(DBG_CURRENT, "ad:%.2f %.2f %.2f %.2f ", ia, ib, ic, in_current);
    } else {
        // TODO: add filter
        in_idle_pid_cur_a = (float)AdcResult.ADCRESULT2 * ADC2MA;
        in_idle_pid_cur_b = (float)AdcResult.ADCRESULT1 * ADC2MA;
        //append_dprintf(DBG_CURRENT, "adc idle: %.2f %.2f", in_idle_pid_cur_u, in_idle_pid_cur_v);
    }

#endif

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
        static float angle = 0;

        if ((csa.loop_cnt & csa.loop_msk) == 0) {
            angle += (float)(M_PI / 180.0 * 0.5);
        }

        float v_alpha = -out_voltage * sinf(angle);
        float v_beta =  out_voltage * cosf(angle);

        out_pwm_u = (int16_t)(v_alpha + 0.5f);
        out_pwm_v = (int16_t)(-v_alpha / 2 + v_beta * 0.866025404f + 0.5f); // (√3÷2)
        out_pwm_w = -out_pwm_u - out_pwm_v;

        /*
        append_dprintf(DBG_CURRENT, "u:%.2f %.2f %d ", pid_cur_u.target, pid_cur_u.i_term, out_pwm_u);
        append_dprintf(DBG_CURRENT, "v:%.2f %.2f %d ", pid_cur_v.target, pid_cur_v.i_term, out_pwm_v);
        append_dprintf(DBG_CURRENT, "w:%d", out_pwm_w);
        */
        if ((csa.loop_cnt & csa.loop_msk) == 0) {
            d_debug("a: %d, uvw: %d %d %d\n",
                    (int)(angle*180.0f/(float)M_PI),
                    out_pwm_u, out_pwm_v, out_pwm_w);
        }

        //append_dprintf(DBG_CURRENT, "uvw:%d %d %d", out_pwm_u, out_pwm_v, out_pwm_w);
    } else {
        // TODO: not only init in STOP
#if 0
        pid_reset(&pid_cur, 0.0, 0.0);
        pid_reset(&pid_speed, 0.0, 0.0);
        pid_reset(&pid_pos, in_pos, 0);
        peak_cur_cnt = 0;
#endif
    }

    uint32_t v1 = HAL_ADCEx_InjectedGetValue(&hadc1, 1);
    uint32_t v2 = HAL_ADCEx_InjectedGetValue(&hadc2, 1);
    uint32_t v3 = HAL_ADCEx_InjectedGetValue(&hadc3, 1);
    if ((csa.loop_cnt & csa.loop_msk) == 0)
        d_debug("adc: %p %d %d %d\n", hadc, v1, v2, v3);

    //HAL_ADCEx_InjectedStart_IT(&hadc1);
    //HAL_ADCEx_InjectedStart_IT(&hadc2);
    //HAL_ADCEx_InjectedStart_IT(&hadc3);
    gpio_set_value(&led_r, !gpio_get_value(&led_r));


    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, DRV_PWM_HALF - out_pwm_u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, DRV_PWM_HALF - out_pwm_v);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, DRV_PWM_HALF - out_pwm_w);

    csa.loop_cnt++;
}
