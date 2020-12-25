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

static cdn_sock_t sock_raw_dbg = { .port = 0xa, .ns = &dft_ns }; // raw debug
static list_head_t raw_pend = { 0 };

static float tto_last; // encoder last value


void selection_sort(int32_t arr[], int len, int32_t order[])
{
    for (int i = 0 ; i < len - 1 ; i++) {
        int min = i;
        for (int j = i + 1; j < len; j++) {
            if (arr[j] < arr[min])
                min = j;
        }
        if (min != i) {
            swap(arr[min], arr[i]);
            if (order)
                swap(order[min], order[i]);
        }
    }
}

void selection_sort_u(uint32_t arr[], int len, uint32_t order[])
{
    for (int i = 0 ; i < len - 1 ; i++) {
        int min = i;
        for (int j = i + 1; j < len; j++) {
            if (arr[j] < arr[min])
                min = j;
        }
        if (min != i) {
            swap(arr[min], arr[i]);
            if (order)
                swap(order[min], order[i]);
        }
    }
}


void app_motor_init(void)
{
    pid_f_init(&csa.pid_cur, true);
    pid_f_init(&csa.pid_speed, true);
    pid_i_init(&csa.pid_pos, true);

    csa.sen_encoder = encoder_read(); // init last value
    tto_last = csa.sen_encoder;
    cdn_sock_bind(&sock_raw_dbg);
}

void app_motor_routine(void)
{
    if (frame_free_head.len > 1) {
        cdn_pkt_t *pkt = cdn_pkt_get(&raw_pend);
        if (pkt)
            cdn_sock_sendto(&sock_raw_dbg, pkt);
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


static inline void t_curve_compute(void)
{
    if (csa.state != ST_POS_TC) {
        csa.tc_state = 0;
        csa.tc_vc = 0;
        csa.tc_pos = csa.sen_pos;
        return;
    }

    if (csa.tc_state) {

        if (csa.tc_pos != csa.cal_pos && (csa.tc_pos - csa.cal_pos >= 0) != (csa.tc_vc >= 0.0f)) { // different direction

            csa.tc_ac = sign(csa.tc_pos - csa.cal_pos) * min((float)csa.tc_accel, fabsf(csa.tc_vc));
            csa.tc_state = 1;

        } else {
            csa.tc_ac = sign(csa.tc_pos - csa.cal_pos) * (float)csa.tc_accel;

            if (csa.tc_pos != csa.cal_pos)
                csa.tc_ac = ((/* tc_ve + */ csa.tc_vc) / 2.0f) * (/* tc_ve */ - csa.tc_vc) / (csa.tc_pos - csa.cal_pos);

            // Slightly more than allowed, such as 1.1 times.
            csa.tc_ac = sign(csa.tc_ac) * min(fabsf(csa.tc_ac), (float)csa.tc_accel * 1.1f);
        }

        if (fabsf(csa.tc_ac) < csa.tc_accel) {
            if (csa.tc_state == 1) {
                csa.tc_vc += sign(csa.tc_pos - csa.cal_pos) * ((float)csa.tc_accel / (CURRENT_LOOP_FREQ / 25.0f));
                csa.tc_vc = clip(csa.tc_vc, -(float)csa.tc_speed, (float)csa.tc_speed);
            }
        } else {
            csa.tc_state = 2;
            csa.tc_vc += csa.tc_ac / (CURRENT_LOOP_FREQ / 25.0f);
        }

        //if (fabsf(csa.tc_vc) < (float)csa.tc_speed_min)
        //    csa.tc_vc = sign(csa.tc_pos - csa.cal_pos) * (float)csa.tc_speed_min;

        int32_t new_pos = lroundf((float)csa.cal_pos + csa.tc_vc / (CURRENT_LOOP_FREQ / 25.0f));
        csa.cal_pos = new_pos;
    }

    if (abs(csa.cal_pos - csa.tc_pos) <= 1 /*&& fabsf(csa.tc_vc) <= (float)csa.tc_speed_min*/) {
        csa.tc_state = 0;
        csa.tc_vc = 0;
        csa.tc_ac = 0;
        csa.cal_pos = csa.tc_pos;
    }
    if (!csa.tc_state)
        csa.tc_vc = 0;
}

static inline void position_loop_compute(void)
{
    if (csa.state < ST_POSITION) {
        pid_i_reset(&csa.pid_pos, csa.sen_pos, 0);
        pid_i_set_target(&csa.pid_pos, csa.sen_pos);
        csa.cal_pos = csa.sen_pos;
        if (csa.state == ST_STOP)
            csa.cal_speed = 0;
        return;
    }

    t_curve_compute();
    pid_i_set_target(&csa.pid_pos, csa.cal_pos);
    csa.cal_speed = pid_i_compute(&csa.pid_pos, csa.sen_pos);

    raw_dbg(3);
}


#define S_HIST_LEN 5

static inline void speed_loop_compute(void)
{
    static int sub_cnt = 0;
    static float s_avg = 0;
    static int s_filt_cnt = 0;

    s_avg += csa.delta_encoder * (float)CURRENT_LOOP_FREQ; // encoder steps per sec

    if (++s_filt_cnt == 5) {
        csa.sen_speed = s_avg / 5.0f;
        s_avg = 0;
        s_filt_cnt = 0;

        if (++sub_cnt == 5) {
            sub_cnt = 0;
            position_loop_compute();
            raw_dbg(2);
        }

        if (csa.state < ST_CONST_SPEED) {
            pid_f_reset(&csa.pid_speed, 0, 0);
            pid_f_set_target(&csa.pid_speed, 0);
            if (csa.state == ST_STOP)
                csa.cal_current = 0;
        } else {
            pid_f_set_target(&csa.pid_speed, csa.cal_speed); // TODO: only set once after modified
            csa.cal_current = lroundf(pid_f_compute_no_d(&csa.pid_speed, csa.sen_speed));
        }

        raw_dbg(1);
    }
}


#define HIST_LEN 10
static uint16_t hist[HIST_LEN] = { 0 };
static int8_t hist_err = -2;

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    gpio_set_value(&led_r, 1);

    static float ia_idle, ib_idle;
    float sin_sen_angle_elec, cos_sen_angle_elec; // reduce the amount of calculations
    int16_t out_pwm_u = 0, out_pwm_v = 0, out_pwm_w = 0;
    bool dbg_str = csa.dbg_str_msk && (csa.loop_cnt % csa.dbg_str_skip) == 0;

    csa.ori_encoder = encoder_read();
    csa.noc_encoder = csa.ori_encoder - csa.bias_encoder;
    int16_t delta_enc = csa.noc_encoder - csa.sen_encoder; // sen_encoder is previous value

    gpio_set_value(&dbg_out, 1);

#if 0
    // v--- Encoder Filter: ---v

    for (int i = 0; i < HIST_LEN - 1; i++)
        hist[i] = hist[i + 1];
    hist[HIST_LEN - 1] = csa.noc_encoder;

    uint32_t order[HIST_LEN];
    uint32_t hist32[HIST_LEN];
    for (int i = 0; i < HIST_LEN; i++) {
        hist32[i] = hist[i];
        order[i] = i;
    }                                          // hist: 1, 2, 9, 8 -> hist32: 1, 2, 8, 9
    selection_sort_u(hist32, HIST_LEN, order); // order 0, 1, 2, 3    order:  0, 1, 3, 2

    int longest_idx = 0;
    uint32_t longest_dt = hist32[0] + 0x10000 - hist32[HIST_LEN - 1]; // 1+10-9=2
    for (int i = 0; i < HIST_LEN - 1; i++) {
        if (hist32[i + 1] - hist32[i] > longest_dt) {
            longest_dt = hist32[i + 1] - hist32[i];
            longest_idx = i + 1; // idx: 2, val: 8
        }
    }
    for (int i = 0; i < longest_idx; i++)      // hist32: 1, 2, 8, 9 -> hist32: 11, 12, 8, 9 (range: [0,9])
            hist32[i] += 0x10000;              // order:  0, 1, 3, 2    order:   0,  1, 3, 2

                                               // hist32: 11, 12, 8, 9 -> hist32: 8, 9, 11, 12
    selection_sort_u(hist32, HIST_LEN, order); // order:   0,  1, 3, 2    order:  3, 2,  0,  1

    selection_sort_u(order+3, HIST_LEN-6, hist32+3); // hist32: 8, 9, 11, 12 -> hist32: 8, 11, 9, 12
                                                     // order:  3, 2,  0,  1    order:  3,  0, 2,  1

    float iavg = 0;
    for (int i = 3; i < HIST_LEN - 3; i++)
        iavg += order[i];
    iavg = iavg / (HIST_LEN - 6); // (0 + 2) / 2 = 1

    float davg = 0;
    for (int i = 3; i < HIST_LEN - 3 - 1; i++) { // (hist[2] - hist[0]) / (2 - 0) => (9 - 1) / 2
        //davg += (float)((int16_t)(hist[order[i+1]] - hist[order[i]])) / (float)(order[i+1] - order[i]);
        int v_delta = (int)hist32[order[i+1]] - (int)hist32[order[i]]; // (9 - 11) = -2
        int v_distant = order[i+1] - order[i];
        davg += (float)v_delta / (float)v_distant; // -2 / 2 = -1
    }
    davg = davg / (HIST_LEN - 6 - 1);

    uint32_t ttt = 0;
    for (int i = 3; i < HIST_LEN - 3; i++)
        ttt += hist32[i];
    float tto = (float)ttt / (float)(HIST_LEN - 6); // (11 + 9) / 2 = 10

    if (tto >= 0x10000)
        tto = tto - (float)0x10000; // 10 - 10 = 0

    csa.sen_encoder = lroundf(tto + davg * (HIST_LEN /*- 1*/ - iavg));
    csa.delta_encoder = tto - tto_last;
    tto_last = tto;

    if (csa.delta_encoder > 0x8000)
        csa.delta_encoder -= 0x10000;
    else if (csa.delta_encoder < -0x8000)
        csa.delta_encoder += 0x10000;

    // csa.sen_encoder = tto + csa.delta_encoder * 4;

    // ^--- END of Encoder Filter ---^
#else

    if (hist_err < 0 || abs(hist[1] - hist[0]) > 500) {
        csa.sen_encoder = csa.noc_encoder;
        csa.delta_encoder = delta_enc;
        if (hist_err < 0)
            hist_err++;
    } else {
        uint32_t hist_raised[2] = {hist[0], hist[1]};
        uint32_t sen_raised = csa.noc_encoder;

        // empty in range: 1/3 to 2/3
        if ((hist[0] < 0x10000/3 || hist[0] >= 0x10000*2/3) && (hist[1] < 0x10000/3 || hist[1] >= 0x10000*2/3)) {
            if (hist[0] < 0x10000/3 && hist[1] < 0x10000/3) {
                if (csa.noc_encoder < 0x10000*2/3)
                    sen_raised += 0x10000;
            } else if (csa.noc_encoder < 0x10000/3) {
                    sen_raised += 0x10000;
            }
            if (hist[0] < 0x10000/3)
                hist_raised[0] += 0x10000;
            if (hist[1] < 0x10000/3)
                hist_raised[1] += 0x10000;
        }
        int16_t hist_delta = lroundf((int16_t)(hist_raised[1] - hist_raised[0]) * 0.8f);
        uint32_t estimate = hist_raised[1] + hist_delta;
        if (abs(sen_raised - estimate) > 500) {
            csa.sen_encoder = estimate >= 0x10000 ? (estimate - 0x10000) : estimate;
            csa.delta_encoder = hist_delta;
            if(++hist_err > 2)
                hist_err = -2;
        } else {
            csa.sen_encoder = csa.noc_encoder;
            csa.delta_encoder = delta_enc;
            hist_err = 0;
        }
    }

    //csa.sen_encoder = csa.noc_encoder;
    //csa.delta_encoder = delta_enc;

    hist[0] = hist[1];
    hist[1] = csa.sen_encoder;
#endif

    gpio_set_value(&dbg_out, 0);


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
    uint16_t encoder_sub = csa.sen_encoder % lroundf((float)0x10000/csa.motor_poles);
    csa.sen_angle_elec = encoder_sub*((float)M_PI*2/((float)0x10000/csa.motor_poles));
    sin_sen_angle_elec = sinf(csa.sen_angle_elec);
    cos_sen_angle_elec = cosf(csa.sen_angle_elec);

    if (dbg_str)
        d_debug_c(" a %d.%.2d %d.%.2d %d.%.2d", P_2F(angle_mech), P_2F(csa.sen_angle_elec), P_2F(csa.cali_angle_elec));

    if (csa.state != ST_STOP) {
        int32_t ia = HAL_ADCEx_InjectedGetValue(&hadc1, 1) - ia_idle;
        int32_t ib = HAL_ADCEx_InjectedGetValue(&hadc2, 1) - ib_idle;
        int32_t ic = -ia - ib;

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

            csa.cal_i_sq = -csa.cali_current; // sign select direction
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
    gpio_set_value(&led_r, 0);
}
