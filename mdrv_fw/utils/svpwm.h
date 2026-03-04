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

#define SVPWM_MAX_DUTY          0.92f   // 92% pwm max duty
#define SVPWM_B_MARGIN          40      // bottom margin: 1.95% (40/2047)
#define SVPWM_MAX_MAG           (2047 * 1.15f * SVPWM_MAX_DUTY) // deliver 15% more power by svpwm

#define DEADTIME_PWM_DUTY       20      // (1÷41504Hz)÷4096×20: 118ns
#define DEADTIME_CUR_THRESHOLD  100


static inline void svpwm_deadtime_compensate(int16_t *pwm_uvw, const int16_t *sen_i)
{
    for (int n = 0; n < 3; n++) {
        int16_t i = sen_i[n];
        int16_t comp = DEADTIME_PWM_DUTY * fminf((float)abs(i) / DEADTIME_CUR_THRESHOLD, 1.0f) + 0.5f;
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
    int16_t out_ofs = -out_min - 2047 + SVPWM_B_MARGIN;
    pwm_uvw[0] = clip(pwm_uvw[0] + out_ofs, -2047, 2047);
    pwm_uvw[1] = clip(pwm_uvw[1] + out_ofs, -2047, 2047);
    pwm_uvw[2] = clip(pwm_uvw[2] + out_ofs, -2047, 2047);

    return mag;
}

#endif
