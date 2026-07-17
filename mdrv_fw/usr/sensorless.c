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

static float sl_angle = 0;
static float sl_speed = 0;

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


void sl_maintain(void)
{
    float theta_err = remainderf(csa.pll_theta - sl_angle, 2 * M_PIf);

    if (csa.state == ST_STOP && csa.sl_state) {
        csa.sl_state = 0;
        csa.sl_start = 0;
        sl_angle = 0;
        sl_speed = 0;
        d_info("sl: stop\n");
        return;
    }

    if (!csa.sl_state && csa.sl_start) {
        sl_angle = csa.meas_elec_angle / 65536.0f * (2 * M_PIf);
        sl_speed = 0;
        state_w_hook_before(0, 0, (uint8_t []){ST_CURRENT});
        csa.state = ST_CURRENT;
        csa.sl_state = 1; // speed inc
        d_info("sl: inc speed...\n");
        csa.tgt_id = 3200; // drag current, direction comes from sl_angle ramp
        csa.tgt_iq = 0;
        return;
    }

    if (csa.sl_state == 1) {
        if (fabsf(sl_speed - 800 * csa.sl_start) < 0.001f) {
            csa.tgt_id = 800; // lower id to reduce smo angle bias (∝ ΔR·i) before handoff
            csa.sl_state = 2; // current dec
            d_info("sl: dec current...\n");
        }
        return;
    }

    if (csa.sl_state == 2) {
        if (fabsf(theta_err) < (20 / 180.0f) * M_PIf
                && fabsf(csa.pll_omega - 800 * csa.sl_start) < 80) {
            csa.sl_state = 3;
            csa.tgt_id = 0;
            csa.tgt_iq = 4000 * csa.sl_start;
            //csa.state = ST_SPEED;
            d_info("sl: closeloop...\n");
        }
    }
}

void sl_angle_update(void)
{
    if (csa.sl_state != 1 && csa.sl_state != 2)
        return;

    float speed_tgt = 800 * csa.sl_start;
    if (fabsf(sl_speed - speed_tgt) >= 0.01f)
        sl_speed += sl_speed < speed_tgt ? 0.01f : -0.01f;
    else
        sl_speed = speed_tgt;

    sl_angle += sl_speed / CURRENT_LOOP_FREQ;
    if (sl_angle >= 2 * M_PIf)
        sl_angle -= 2 * M_PIf;
    else if (sl_angle < 0)
        sl_angle += 2 * M_PIf;

    csa.tgt_elec_angle = lroundf(sl_angle / (2 * M_PIf) * 0x10000);
}
