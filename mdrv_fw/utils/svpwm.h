/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __SVPWM_H__
#define __SVPWM_H__

#define SVPWM_FULL              16376   // 2047 << 3, 32768 counts = vcc
#define SVPWM_MAX_DUTY_PCT      92      // 92% pwm max duty
#define SVPWM_B_MARGIN          320     // bottom margin: 1.95% (320/16376)
#define SVPWM_MAX_MAG           ((SVPWM_FULL * 115 * SVPWM_MAX_DUTY_PCT) / 10000)

#define DEADTIME_PWM_DUTY       160     // (1÷41504Hz)÷32768×160: 118ns
#define DEADTIME_CUR_THRESHOLD  800


static inline void svpwm_deadtime_compensate(int16_t *pwm_uvw, const int16_t *sen_i)
{
    for (int n = 0; n < 3; n++) {
        int16_t i = sen_i[n];
        int16_t ai = min(abs(i), DEADTIME_CUR_THRESHOLD);
        int16_t comp = DIV_ROUND_CLOSEST(DEADTIME_PWM_DUTY * ai, DEADTIME_CUR_THRESHOLD);
        pwm_uvw[n] += i >= 0 ? comp : -comp;
    }
}


static inline float svpwm(float v_alpha, float v_beta, int16_t *pwm_uvw, int16_t *pwm_dbg, const int16_t *sen_i)
{
    // limit vector magnitude
    float mag = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    if (mag > SVPWM_MAX_MAG) {
        v_alpha *= SVPWM_MAX_MAG / mag;
        v_beta *= SVPWM_MAX_MAG / mag;
    }

    pwm_uvw[0] = lroundf(v_alpha);
    pwm_uvw[1] = lroundf(-v_alpha / 2 + v_beta * 0.866025404f); // (√3÷2)
    pwm_uvw[2] = -pwm_uvw[0] - pwm_uvw[1];

    memcpy(pwm_dbg, pwm_uvw, 2 * 3);
    if (sen_i) {
        svpwm_deadtime_compensate(pwm_uvw, sen_i);
        memcpy(pwm_dbg + 3, pwm_uvw, 2 * 3);
    }

    // increase the current sensing window
    int16_t out_min = min(pwm_uvw[0], min(pwm_uvw[1], pwm_uvw[2]));
    int16_t out_ofs = -out_min - SVPWM_FULL + SVPWM_B_MARGIN;
    pwm_uvw[0] = clip(pwm_uvw[0] + out_ofs, -SVPWM_FULL, SVPWM_FULL);
    pwm_uvw[1] = clip(pwm_uvw[1] + out_ofs, -SVPWM_FULL, SVPWM_FULL);
    pwm_uvw[2] = clip(pwm_uvw[2] + out_ofs, -SVPWM_FULL, SVPWM_FULL);

    return mag;
}

#endif
