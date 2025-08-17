/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2025, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include <math.h>
#include "app_main.h"
#include "sensorless.h"

static float sat(float x, float eps)
{
    if (x > eps)
        return 1.0f;
    else if (x < -eps)
        return -1.0f;
    else
        return x / eps;
}

void smo_init(smo_t *smo, bool reset)
{
    smo->_f = 1.0f - (smo->r * smo->delta_t / smo->l);
    smo->_g = smo->delta_t / smo->l;

    if (reset) {
        smo->i_alpha = 0.0f;
        smo->i_beta = 0.0f;
        smo->e_alpha = 0.0f;
        smo->e_beta = 0.0f;
        smo->v_alpha_real = 0;
        smo->v_beta_real = 0;
    }
}

void smo_update(smo_t *smo)
{
    smo->i_alpha = smo->_f * smo->i_alpha + smo->_g * (smo->v_alpha_real - smo->e_alpha);
    smo->i_beta  = smo->_f * smo->i_beta  + smo->_g * (smo->v_beta_real  - smo->e_beta);

    float err_alpha = smo->i_alpha - smo->i_alpha_real;
    float err_beta  = smo->i_beta - smo->i_beta_real;

    smo->e_alpha += smo->gamma * sat(err_alpha, smo->eps) * smo->delta_t;
    smo->e_beta  += smo->gamma * sat(err_beta, smo->eps)  * smo->delta_t;
}


void pll_init(pll_t *pll, bool reset)
{
    pll->_ki = pll->ki * pll->delta_t;

    if (reset) {
        pll->theta = 0.0f;
        pll->omega = 0.0f;
        pll->i_term = 0.0f;
    }
}

void pll_update(pll_t *pll, float e_alpha, float e_beta)
{
    float sin_theta = sinf(pll->theta);
    float cos_theta = cosf(pll->theta);

    if (csa.sl_start < 0) { // ccw
        e_alpha *= -1;
        e_beta *= -1;
    }

    // Δe = -Eα·cosθ - Eβ·sinθ
    float err = -e_alpha * cos_theta - e_beta * sin_theta;

    pll->i_term += err * pll->_ki;
    pll->omega = pll->kp * err + pll->i_term;

    // θ: integral(ω)
    pll->theta += pll->omega * pll->delta_t;

    if (pll->theta >= 2 * M_PIf)
        pll->theta -= 2 * M_PIf;
    else if (pll->theta < 0)
        pll->theta += 2 * M_PIf;

    pll->_atan2 = atan2f(-e_alpha, e_beta); // dbg
    if (pll->_atan2 < 0)
        pll->_atan2 += 2 * M_PIf;
}


void sl_routine(void)
{
    float theta_err = csa.pll.theta - csa.cali_angle_elec;

    if (csa.state == ST_STOP && csa.sl_state) {
        csa.sl_state = 0;
        csa.sl_start = 0;
        d_info("sl: stop\n");
        return;
    }

    if (!csa.sl_state && csa.sl_start) {
        csa.cali_angle_speed_tgt = 800 * csa.sl_start;
        csa.cali_current = 400 * csa.sl_start;
        state_w_hook_before(0, 0, (uint8_t []){ST_CALI});
        csa.state = ST_CALI;
        csa.sl_state = 1; // speed inc
        d_info("sl: inc speed...\n");
        return;
    }

    if (csa.sl_state == 1) {
        if (fabsf(csa.cali_angle_speed - csa.cali_angle_speed_tgt) < 0.001f) {
            csa.cali_current = 10 * csa.sl_start;
            csa.sl_state = 2; // current dec
            d_info("sl: dec speed...\n");
        }
        return;
    }

    if (csa.sl_state == 2) {
        if (fabsf(theta_err) < (10 / 180.0f) * M_PIf) {
            csa.sl_state = 3;
            csa.cal_current = 500 * csa.sl_start;
            csa.state = ST_CURRENT; // or ST_SPEED
            d_info("sl: closeloop...\n");
        }
    }
}

