/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __MISC_H__
#define __MISC_H__

void raw_dbg(int idx);
void cali_elec_angle(void);


static inline void encoder_isr(void)
{
    __HAL_DMA_ENABLE(hspi1.hdmatx);
}


static inline void motor_pwm_output(int16_t *pwm_uvw, bool motor_wire_swap)
{
    // pwm_uvw: +/-16376 (32768 counts = vcc), compare range: [1, 4095]
    if (motor_wire_swap) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, DRV_PWM_HALF - DIV_ROUND_POW2(pwm_uvw[1], 3));
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, DRV_PWM_HALF - DIV_ROUND_POW2(pwm_uvw[0], 3));
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, DRV_PWM_HALF - DIV_ROUND_POW2(pwm_uvw[0], 3)); // TIM1_CH3: A
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, DRV_PWM_HALF - DIV_ROUND_POW2(pwm_uvw[1], 3)); // TIM1_CH2: B
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, DRV_PWM_HALF - DIV_ROUND_POW2(pwm_uvw[2], 3));     // TIM1_CH1: C
}

#endif
