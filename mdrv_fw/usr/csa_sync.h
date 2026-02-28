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
    memcpy(csa.sen_i, as->sen_i, sizeof(csa.sen_i));
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
    memcpy(&csa.nob_pos, &ef->nob_pos,
            offsetof(encoder_filter_t, sen_rpm_avg) - offsetof(encoder_filter_t, nob_pos) + 4);
}


// encoder_linearize

static inline void csa2encoder_linearizer_mt(encoder_linearizer_t *el)
{
    el->max_val = csa.encoder_linearizer_max;
}


// anticog

static inline void csa2anticog_mt(anticog_t *ac)
{
    ac->max_iq = csa.anticog_max_iq;
    ac->ratio_vq = csa.anticog_ratio_vq;
}


// trap_planner

static inline void csa2trap_planner_mt(trap_planner_t *tp)
{
    tp->max_err = csa.tp_max_err;
}

static inline void csa2trap_planner(trap_planner_t *tp)
{
    memcpy(&tp->pos_tgt, &csa.tp_pos, 4 * 3);
}

static inline void trap_planner2csa_rst(trap_planner_t *tp)
{
    csa.tp_pos = tp->pos_tgt;

    csa.tp_state = tp->state;
    csa.tp_vel_out = tp->vel_out;
    csa.tp_acc_brake = tp->acc_brake;
}

static inline void trap_planner2csa(trap_planner_t *tp)
{
    csa.cal_pos = tp->pos_out;

    csa.tp_state = tp->state;
    csa.tp_vel_out = tp->vel_out;
    csa.tp_acc_brake = tp->acc_brake;
}

#endif
