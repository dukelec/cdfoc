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

static cdn_sock_t sock8 = { .port = 8, .ns = &dft_ns }; // raw debug
static list_head_t raw_pend = { 0 };


void app_motor_init(void)
{
    pid_f_init(&csa.pid_cur, true);
    pid_f_init(&csa.pid_speed, true);
    pid_i_init(&csa.pid_pos, true);

    csa.sen_encoder = encoder_read(); // init last value
    cdn_sock_bind(&sock8);
}

void app_motor_routine(void)
{
    if (frame_free_head.len > 1) {
        cdn_pkt_t *pkt = cdn_pkt_get(&raw_pend);
        if (pkt)
            cdn_sock_sendto(&sock8, pkt);
    }
}

static void raw_dbg(int idx)
{
    static cdn_pkt_t *pkt_raw[4] = { NULL };
    static uint8_t skip_cnt[4] = { 0 };
    static bool pkt_less = false;

    if (!(csa.dbg_raw_msk & (1 << idx))) {
        skip_cnt[idx] = 0;
        if (pkt_raw[idx]) {
            list_put(&dft_ns.free_pkts, &pkt_raw[idx]->node);
            pkt_raw[idx] = NULL;
        }
        return;
    }

    if (++skip_cnt[idx] >= csa.dbg_raw_skip[idx])
        skip_cnt[idx] = 0;
    if (skip_cnt[idx] != 0)
        return;

    if (pkt_less) {
        if (raw_pend.len == 0) {
            pkt_less = false;
        }
    }

    if (!pkt_less && !pkt_raw[idx]) {
        if (dft_ns.free_pkts.len < 5) {
            pkt_less = true;
            return;

        } else {
            pkt_raw[idx] = cdn_pkt_get(&dft_ns.free_pkts);
            pkt_raw[idx]->dst = csa.dbg_raw_dst;
            pkt_raw[idx]->dat[0] = 0x40 | idx;
            *(uint32_t *)(pkt_raw[idx]->dat + 1) = csa.loop_cnt;
            pkt_raw[idx]->dat[5] = csa.dbg_raw_skip[idx];
            pkt_raw[idx]->len = 6;
        }
    }
    if (!pkt_raw[idx])
        return;

    for (int i = 0; i < 10; i++) {
        regr_t *regr = &csa.dbg_raw[idx][i];
        if (!regr->size)
            break;
        uint8_t *dst_dat = pkt_raw[idx]->dat + pkt_raw[idx]->len;
        memcpy(dst_dat, ((void *) &csa) + regr->offset, regr->size);
        pkt_raw[idx]->len += regr->size;
    }

    if (pkt_raw[idx]->len >= csa.dbg_raw_th) {
        list_put(&raw_pend, &pkt_raw[idx]->node);
        pkt_raw[idx] = NULL;
    }
}


static inline void position_loop_compute(void)
{
    if (csa.state < ST_POSITION) {
        csa.tc_run = false;
        pid_i_reset(&csa.pid_pos, csa.sen_pos, 0);
        pid_i_set_target(&csa.pid_pos, csa.sen_pos);
        csa.cal_pos = csa.sen_pos;
        csa.tc_pos = csa.sen_pos;
        if (csa.state == ST_STOP) {
            csa.cal_speed = 0;
        }
        return;
    }

    if (!csa.tc_run) {
        csa.cal_pos = csa.tc_pos;
        pid_i_set_target(&csa.pid_pos, csa.cal_pos);

    } else if (csa.tc_cnt >= csa.tc_steps) {
        csa.tc_run = false;
        csa.cal_pos = csa.tc_pos;
        pid_i_set_target(&csa.pid_pos, csa.cal_pos);

    } else {
        t_curve_step(csa.tc_s_seg, csa.tc_t_seg, csa.tc_a_seg,
                csa.tc_v_s, 25.0 / CURRENT_LOOP_FREQ * csa.tc_cnt, // i * period
                &csa.tc_v_cur, &csa.tc_s_cur);
        csa.tc_cnt++;
        pid_i_set_target(&csa.pid_pos, lroundf(csa.tc_s_s + csa.tc_s_cur));
    }

    csa.cal_speed = lroundf(pid_i_compute(&csa.pid_pos, csa.sen_pos));
}

static inline void speed_loop_compute(void)
{
    static int sub_cnt = 0;
    const float coeffs[5] = {0.1, 0.15, 0.2, 0.25, 0.3};
    static float speed_avg = 0.0;
    static int speed_filt_cnt = 0;

    float speed = csa.delta_encoder * CURRENT_LOOP_FREQ; // encoder steps per sec
    speed_avg += speed * coeffs[speed_filt_cnt++];

    if (speed_filt_cnt == 5) {

        csa.sen_speed = lroundf(speed_avg);
        speed_avg = 0;
        speed_filt_cnt = 0;


        if (csa.state < ST_CONST_SPEED) {
            pid_f_reset(&csa.pid_speed, 0, 0);
            if (csa.state == ST_STOP) {
                csa.cal_current = 0;
                pid_f_set_target(&csa.pid_speed, 0);
            }
        } else {
            pid_f_set_target(&csa.pid_speed, csa.cal_speed); // TODO: only set once after modified
            csa.cal_current = -lroundf(pid_f_compute_no_d(&csa.pid_speed, csa.sen_speed));
        }

        if (++sub_cnt == 5) {
            sub_cnt = 0;
            position_loop_compute();
            raw_dbg(2);
            raw_dbg(3);
        }
        raw_dbg(1);
    }
}


#define ENC_MAX_DELTA   40 // > 21
static int enc_err_cnt = 0;

#define HIST_RM 3
#define HIST_LEN 10
static uint16_t hist[HIST_LEN] = { 0 };


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    static float ia_idle, ib_idle;
    float sin_sen_angle_elec, cos_sen_angle_elec; // reduce the amount of calculations
    int16_t out_pwm_u = 0, out_pwm_v = 0, out_pwm_w = 0;
    bool dbg_str = csa.dbg_str_msk && (csa.loop_cnt % csa.dbg_str_skip) == 0;

    csa.ori_encoder = encoder_read();
    csa.noc_encoder = csa.ori_encoder - csa.bias_encoder;
    int16_t delta_enc = csa.noc_encoder - csa.sen_encoder; // sen_encoder is previous value

    uint16_t hist_dm[HIST_LEN] = { 0 }; // delta_max
    bool hist_rm[HIST_LEN] = { 0 };

    for (int i = 0; i < HIST_LEN - 1; i++)
        hist[i] = hist[i + 1];
    hist[HIST_LEN - 1] = csa.noc_encoder;

    hist_dm[0] = abs(hist[1] - hist[0]);
    hist_dm[HIST_LEN - 1] = abs(hist[HIST_LEN - 1] - hist[HIST_LEN - 2]) * 2;
    for (int i = 1; i < HIST_LEN - 2; i++)
        hist_dm[i] = (abs(hist[i] - hist[i - 1]) + abs(hist[i + 1] - hist[i]));

    for (int c = 0; c < HIST_RM; c++) {
        int idx = -1;
        int max_val = 0;
        for (int i = 0; i < HIST_LEN; i++) {
            if (!hist_rm[i] && (hist_dm[i] > max_val || idx == -1)) {
                max_val = hist_dm[i];
                idx = i;
            }
        }
        hist_rm[idx] = true;
    }

    int first = -1;
    int delta_sum = 0;
    int delta_cnt = 0;
    for (int i = 0; i < HIST_LEN - 1; i++) {
        if (!(hist_rm[i] || hist_rm[i + 1])) {
            if (first == -1)
                first = i;
            delta_sum += (int16_t)(hist[i + 1] - hist[i]);
            delta_cnt ++;
        }
    }

    csa.delta_encoder = DIV_ROUND_CLOSEST(delta_sum, delta_cnt);
    csa.sen_encoder = hist[first] + (HIST_LEN - first - 1) * csa.delta_encoder;


#if 0
    int16_t delta = hist[i + 1] - hist[i];
    if (abs(delta) <= ENC_MAX_DELTA) {

    }



    if (abs(delta_enc) > ENC_MAX_DELTA) {
        if (++enc_err_cnt > 20) {
            d_debug("enc: rst!\n");
            enc_err_cnt = 0;
            csa.sen_encoder = csa.noc_encoder;
            csa.delta_encoder = delta_enc;
        } else {
            csa.sen_encoder += csa.delta_encoder;
        }
    } else {
        enc_err_cnt = 0;
        //csa.sen_encoder = DIV_ROUND_CLOSEST((int16_t)csa.sen_encoder + (int16_t)csa.noc_encoder, 2); 32768!!
        csa.sen_encoder += delta_enc / 2;
        //csa.delta_encoder = DIV_ROUND_CLOSEST(csa.delta_encoder + delta_enc, 2);
        csa.delta_encoder = (csa.delta_encoder + delta_enc) / 2;
    }

#endif

#if 0
    if (abs(delta_enc) < 1000 || skip_cnt != 0) { // a good value?
        if (dbg_str && abs(delta_enc) >= 1000)
            d_debug("rst! big delta_enc: %d (%x - %x) | %d\n",
                    delta_enc, csa.noc_encoder, last_encoder, csa.delta_encoder);
        csa.delta_encoder = delta_enc;
        // (4Bytes 5.25M, 1/8 CURRENT_LOOP) was wrong
        // encoder lock data at start of first byte
        csa.sen_encoder = csa.noc_encoder + DIV_ROUND_CLOSEST(delta_enc * 1, 8);
        last_encoder = csa.noc_encoder;
        skip_cnt = 0;
    } else { // skip wrong sensor value
        //if (dbg_str)
        //    d_debug("skip big delta_enc: %d (%x - %x) | %d\n",
        //            delta_enc, csa.noc_encoder, last_encoder, csa.delta_encoder);
        csa.sen_encoder += csa.delta_encoder;
        last_encoder += csa.delta_encoder;
        skip_cnt++;
    }
//#else
    csa.sen_encoder = csa.noc_encoder;
    csa.delta_encoder = delta_enc;
    last_encoder = csa.noc_encoder;
#endif

    if (abs((uint16_t)(csa.ori_pos & 0xffff) - csa.sen_encoder) > 0x10000/2) {
        if (csa.sen_encoder < 0x10000/2)
            csa.ori_pos += 0x10000;
        else
            csa.ori_pos -= 0x10000;
    }
    csa.ori_pos = (csa.ori_pos & 0xffff0000) | csa.sen_encoder;
    csa.sen_pos = csa.ori_pos - csa.bias_pos;

    if (dbg_str)
            d_debug("%04x %d %08x", csa.ori_encoder, delta_enc, csa.sen_pos);

    float angle_mech = csa.sen_encoder*((float)M_PI*2/0x10000);
    uint16_t encoder_sub = csa.sen_encoder % lroundf(0x10000/21.0f);
    csa.sen_angle_elec = encoder_sub*((float)M_PI*2/(0x10000/21.0f));
    sin_sen_angle_elec = sinf(csa.sen_angle_elec);
    cos_sen_angle_elec = cosf(csa.sen_angle_elec);

    if (dbg_str)
        d_debug_c(" a %d.%.2d %d.%.2d %d.%.2d", P_2F(angle_mech), P_2F(csa.sen_angle_elec), P_2F(csa.cali_angle_elec));

    if (csa.state != ST_STOP) {
        int32_t ia = HAL_ADCEx_InjectedGetValue(&hadc1, 1) - ia_idle;
        int32_t ib = HAL_ADCEx_InjectedGetValue(&hadc2, 1) - ib_idle;
        int32_t ic = -ia - ib;
        HAL_ADCEx_InjectedGetValue(&hadc3, 1); // remove this

        float i_alpha = ia;
        float i_beta = (ia + ib * 2) / 1.732050808f; // √3

        csa.sen_current = -i_alpha * sin_sen_angle_elec + i_beta * cos_sen_angle_elec; // i_sq
        float i_sd = i_alpha * cos_sen_angle_elec + i_beta * sin_sen_angle_elec; // i_sd

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
        if (dbg_str)
            d_debug_c(", in %5d %5d %5d", ia, ib, ic);
        if (dbg_str)
            d_debug_c(", i %5d.%.2d, %5d.%.2d", P_2F(csa.sen_current), P_2F(i_sd));

    } else { // state: stop
        static int i_cnt = 0;
        static int32_t ia_sum = 0;
        static int32_t ib_sum = 0;
        ia_sum += HAL_ADCEx_InjectedGetValue(&hadc1, 1);
        ib_sum += HAL_ADCEx_InjectedGetValue(&hadc2, 1);
        HAL_ADCEx_InjectedGetValue(&hadc3, 1);

        if (++i_cnt == 5) {
            ia_idle = DIV_ROUND_CLOSEST(ia_sum, 5);
            ib_idle = DIV_ROUND_CLOSEST(ib_sum, 5);
            ia_sum = ib_sum = 0;
            i_cnt = 0;
        }
        if (dbg_str)
            d_debug_c(", in %5d.%.2d %5d.%.2d", P_2F(ia_idle), P_2F(ib_idle));

        csa.sen_current = 0;
    }

    // current --> pwm
    if (csa.state != ST_STOP) {
        float i_alpha, i_beta;

        // calculate angle
        if (csa.state != ST_CALI) {
            pid_f_set_target(&csa.pid_cur, csa.cal_current);
            csa.cal_i_sq = pid_f_compute_no_d(&csa.pid_cur, csa.sen_current);
            i_alpha = -csa.cal_i_sq * sin_sen_angle_elec;
            i_beta =  csa.cal_i_sq * cos_sen_angle_elec;

        } else {
            csa.cali_angle_elec += csa.cali_angle_step;
            if (csa.cali_angle_elec >= (float)M_PI*2)
                csa.cali_angle_elec -= (float)M_PI*2;

            csa.cal_i_sq = csa.cali_current;
            i_alpha = -csa.cal_i_sq * sinf(csa.cali_angle_elec);
            i_beta =  csa.cal_i_sq * cosf(csa.cali_angle_elec);
        }

        out_pwm_u = lroundf(i_alpha);
        out_pwm_v = lroundf(-i_alpha / 2 + i_beta * 0.866025404f); // (√3÷2)
        out_pwm_w = -out_pwm_u - out_pwm_v;

        /*
        append_dprintf(DBG_CURRENT, "u:%.2f %.2f %d ", pid_cur_u.target, pid_cur_u.i_term, out_pwm_u);
        append_dprintf(DBG_CURRENT, "v:%.2f %.2f %d ", pid_cur_v.target, pid_cur_v.i_term, out_pwm_v);
        append_dprintf(DBG_CURRENT, "w:%d", out_pwm_w);
        */
        if (dbg_str)
            d_debug_c(", o %5d.%.2d %5d %5d %5d\n", P_2F(csa.cal_i_sq), out_pwm_u, out_pwm_v, out_pwm_w);
    } else {
        csa.cal_i_sq = 0;
        pid_f_set_target(&csa.pid_cur, 0);
        pid_f_reset(&csa.pid_cur, 0, 0);

        //peak_cur_cnt = 0;
        if (dbg_str)
            d_debug_c("\n");
    }

    //gpio_set_value(&led_r, !gpio_get_value(&led_r)); // debug for hw config

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, DRV_PWM_HALF - out_pwm_u); // TIM1_CH3: A
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, DRV_PWM_HALF - out_pwm_v); // TIM1_CH2: B
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, DRV_PWM_HALF - out_pwm_w); // TIM1_CH1: C

    speed_loop_compute();
    raw_dbg(0);
    csa.loop_cnt++;
}
