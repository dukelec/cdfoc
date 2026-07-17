/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __CSA_SYNC_H__
#define __CSA_SYNC_H__


// adc_samp

static inline void adc_samp2csa(adc_samp_t *as)
{
    memcpy(csa.meas_i, as->meas_i, sizeof(csa.meas_i));
}


// encoder_filter

static inline void csa2encoder_filter_mt(encoder_filter_t *ef)
{
    ef->bias_encoder = csa.bias_encoder;
    ef->bias_pos = csa.bias_pos;
}

static inline void encoder_filter2csa(encoder_filter_t *ef)
{
    csa.nob_encoder = ef->nob_encoder;
    csa.nob_pos = ef->nob_pos;
    csa.meas_encoder = ef->meas_encoder;
    csa.meas_speed = lroundf(ef->meas_speed);
    csa.meas_pos = ef->meas_pos;
    csa.meas_speed_avg = lroundf(ef->meas_speed_avg);
    csa.meas_rpm_avg = ef->meas_rpm_avg;
}


// encoder_linearize

static inline void csa2encoder_linearizer_mt(encoder_linearizer_t *el)
{
    el->max_val = csa.enc_linear_max;
}


// anticog

static inline void csa2anticog_mt(anticog_t *ac)
{
    ac->max_iq = csa.anticog_max_iq;
    ac->ratio_vq = csa.anticog_ratio_vq;
}


// pid

static inline void csa2pid_mt(pid_i_t *pos, pid_f_t *speed, pid_f_t *iq, pid_f_t *id)
{
    pos->kp = csa.pid_pos_kp;
    pos->out_min = csa.pid_pos_out_min;
    pos->out_max = csa.pid_pos_out_max;
    speed->kp = csa.pid_speed_kp;
    speed->ki = csa.pid_speed_ki;
    speed->out_min = csa.pid_speed_out_min;
    speed->out_max = csa.pid_speed_out_max;
    iq->kp = csa.pid_iq_kp;
    iq->ki = csa.pid_iq_ki;
    iq->out_min = csa.pid_iq_out_min;
    iq->out_max = csa.pid_iq_out_max;
    id->kp = csa.pid_id_kp;
    id->ki = csa.pid_id_ki;
    id->out_min = csa.pid_id_out_min;
    id->out_max = csa.pid_id_out_max;
}


// sensorless observer

static inline void csa2smo_mt(smo_t *smo)
{
    smo->l = csa.smo_l;
    smo->r = csa.smo_r;
    smo->gamma = csa.smo_gamma;
    smo->eps = csa.smo_eps;
    smo_init(smo, false);
}

static inline void csa2smo(smo_t *smo)
{
    smo->v_alpha_real = csa.smo_v_alpha_real;
    smo->v_beta_real = csa.smo_v_beta_real;
    smo->i_alpha_real = csa.smo_i_alpha_real;
    smo->i_beta_real = csa.smo_i_beta_real;
}

static inline void smo2csa(smo_t *smo)
{
    csa.smo_i_alpha = smo->i_alpha;
    csa.smo_i_beta = smo->i_beta;
    csa.smo_e_alpha = smo->e_alpha;
    csa.smo_e_beta = smo->e_beta;
}

static inline void csa2pll_mt(pll_t *pll)
{
    pll->kp = csa.pll_kp;
    pll->ki = csa.pll_ki;
    pll_init(pll, false);
}

static inline void pll2csa(pll_t *pll)
{
    csa.pll_theta = pll->theta;
    csa.pll_omega = pll->omega;
    csa.pll_i_term = pll->i_term;
    csa.pll_atan2 = pll->_atan2;
}


// trap_planner

static inline void csa2trap_planner_mt(trap_planner_t *tp)
{
    tp->max_err = csa.tp_max_err;
}

static inline void csa2trap_planner(trap_planner_t *tp)
{
    tp->pos_tgt = csa.tp_pos;
    tp->vel_tgt = csa.tp_speed;
    tp->acc_tgt = csa.tp_accel;
}

static inline void trap_planner2csa_rst(trap_planner_t *tp)
{
    csa.tp_pos = tp->pos_tgt;

    csa.tp_state = tp->state;
    csa.tp_vel_out = lroundf(tp->vel_out);
}

static inline void trap_planner2csa(trap_planner_t *tp)
{
    csa.tgt_pos = tp->pos_out;

    csa.tp_state = tp->state;
    csa.tp_vel_out = lroundf(tp->vel_out);
}

#endif
