/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2025, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __SENSORLESS_H__
#define __SENSORLESS_H__

typedef struct {
    float v_alpha_real; // [V]
    float v_beta_real;
    float i_alpha_real; // [A]
    float i_beta_real;
    float i_alpha;
    float i_beta;
    float e_alpha;  // Back-EMF
    float e_beta;
    float l;        // [H]
    float r;        // [Ohm]
    float gamma;
    float eps;
    float delta_t;
    float _f;
    float _g;
    int32_t _reserved[9];
} smo_t;

void smo_init(smo_t *smo, bool reset);
void smo_update(smo_t *smo);


typedef struct {
    float theta;   // [rad]
    float omega;   // [rad/s]
    float i_term;
    float kp;
    float ki;
    float delta_t; // control period [s]
    float _ki;
    float _atan2;  // debug
    int32_t _reserved[4];
} pll_t;

void pll_init(pll_t *pll, bool reset);
void pll_update(pll_t *pll, float e_alpha, float e_beta);

#endif
