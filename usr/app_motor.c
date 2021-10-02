/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "math.h"
#include "app_main.h"

#define CUR_AVG_WINDOW  18 // moving average window sample size is 2^18
#define CUR_OFFSET_MAX  2200 // current offset max limit (middle 2048)
#define CUR_OFFSET_MIN  1900 // current offset min limit (middle 2048)

static cdn_sock_t sock_tc_rpt = { .port = 0x10, .ns = &dft_ns, .tx_only = true };
static cdn_sock_t sock_raw_dbg = { .port = 0xa, .ns = &dft_ns, .tx_only = true }; // raw debug
static list_head_t raw_pend = { 0 };

uint8_t state_w_hook_before(uint16_t sub_offset, uint8_t len, uint8_t *dat)
{
    if (*dat == ST_STOP) {
        gpio_set_value(&drv_en, 0);

    } else if (csa.state == ST_STOP && *dat != ST_STOP) {
        gpio_set_value(&drv_en, 1);
        delay_systick(50);

        d_debug("drv 02: %04x\n", drv_read_reg(0x02));
        drv_write_reg(0x02, drv_read_reg(0x02) | 0x1 << 5);
        d_debug("drv 02: %04x\n", drv_read_reg(0x02));

        d_debug("drv 03: %04x\n", drv_read_reg(0x03));
        d_debug("drv 04: %04x\n", drv_read_reg(0x04));

        drv_write_reg(0x03, 0x0344); // 550mA, 1100mA
        drv_write_reg(0x04, 0x0544); // 550mA, 1100mA, 1000-ns peak gate-current
        d_debug("drv 03: %04x\n", drv_read_reg(0x03));
        d_debug("drv 04: %04x\n", drv_read_reg(0x04));
    }
    return 0;
}

uint8_t motor_w_hook_after(uint16_t sub_offset, uint8_t len, uint8_t *dat)
{
    uint32_t flags;

    if (csa.state == ST_POS_TC) {
        local_irq_save(flags);
        if (csa.cal_pos != csa.tc_pos) {
            csa.tc_state = 1; // restart t_curve
        }
        local_irq_restore(flags);
    }
    return 0;
}


void app_motor_init(void)
{
    pid_f_init(&csa.pid_i_sq, true);
    pid_f_init(&csa.pid_i_sd, true);
    pid_f_init(&csa.pid_speed, true);
    pid_i_init(&csa.pid_pos, true);

    csa.sen_encoder = encoder_read(); // init last value
    cdn_sock_bind(&sock_raw_dbg);
}

void app_motor_routine(void)
{
    // update _ki, _kd
    pid_f_init(&csa.pid_i_sq, false);
    pid_f_init(&csa.pid_i_sd, false);
    pid_f_init(&csa.pid_speed, false);
    pid_i_init(&csa.pid_pos, false);

    if (frame_free_head.len > 1) {
        cdn_pkt_t *pkt = cdn_pkt_get(&raw_pend);
        if (pkt)
            cdn_sock_sendto(&sock_raw_dbg, pkt);
    }

    static uint8_t tc_state_old = 0;
    if (csa.tc_rpt_end) {
        if (csa.tc_state == 0 && tc_state_old != 0) {
            cdn_pkt_t *pkt = cdn_pkt_get(&dft_ns.free_pkts);
            if (pkt) {
                cdn_init_pkt(pkt);
                pkt->dst = csa.tc_rpt_dst;
                pkt->dat[0] = 0x40;
                pkt->len = 1;
                cdn_sock_sendto(&sock_tc_rpt, pkt);
            }
        }
    }
    tc_state_old = csa.tc_state;
}

static void raw_dbg(int idx)
{
    static cdn_pkt_t *pkt_raw[4] = { NULL };
    static bool pkt_less = false;

    if (!(csa.dbg_raw_msk & (1 << idx))) {
        if (pkt_raw[idx]) {
            list_put(&dft_ns.free_pkts, &pkt_raw[idx]->node);
            pkt_raw[idx] = NULL;
        }
        return;
    }

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
            cdn_init_pkt(pkt_raw[idx]);
            pkt_raw[idx]->dst = csa.dbg_raw_dst;
            pkt_raw[idx]->dat[0] = 0x40 | idx;
            *(uint32_t *)(pkt_raw[idx]->dat + 1) = csa.loop_cnt;
            pkt_raw[idx]->len = 5;
        }
    }
    if (!pkt_raw[idx])
        return;

    for (int i = 0; i < 6; i++) {
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
    static double p64f = (double)INFINITY;

    if (csa.state != ST_POS_TC) {
        csa.tc_state = 0;
        csa.tc_vc = 0;
        csa.tc_ac = 0;
        csa.tc_pos = csa.sen_pos;
        p64f = (double)INFINITY;
        return;
    } else if (p64f == (double)INFINITY) {
        p64f = csa.cal_pos;
    }

    if (csa.tc_state) {
        if (csa.tc_pos != csa.cal_pos && (csa.tc_pos - csa.cal_pos >= 0) != (csa.tc_vc >= 0.0f)) { // different direction
            csa.tc_ac = sign(csa.tc_pos - csa.cal_pos) * min((float)csa.tc_accel, fabsf(csa.tc_vc));
            csa.tc_state = 1;
        } else {
            if (csa.tc_pos != csa.cal_pos) {
                csa.tc_ac = ((/* tc_ve + */ csa.tc_vc) / 2.0f) * (/* tc_ve */ - csa.tc_vc) / (csa.tc_pos - csa.cal_pos);
                csa.tc_ac = sign(csa.tc_ac) * min(fabsf(csa.tc_ac), (float)csa.tc_accel);
            } else {
                csa.tc_ac = -sign(csa.tc_vc) * (float)csa.tc_accel;
            }
        }

        if (fabsf(csa.tc_ac) * 1.1f < csa.tc_accel) {
            if (csa.tc_state == 1) {
                csa.tc_vc += sign(csa.tc_pos - csa.cal_pos) * ((float)csa.tc_accel / (CURRENT_LOOP_FREQ / 25.0f));
                csa.tc_vc = clip(csa.tc_vc, -(float)csa.tc_speed, (float)csa.tc_speed);
            }
        } else {
            csa.tc_state = 2;
            // Slightly more than allowed, such as 1.1 times.
            csa.tc_vc += csa.tc_ac * 1.1f / (CURRENT_LOOP_FREQ / 25.0f);
        }

        float v_step = (float)csa.tc_accel / (CURRENT_LOOP_FREQ / 25.0f);
        float dt_pos = csa.tc_vc / (CURRENT_LOOP_FREQ / 25.0f);
        if (fabsf(dt_pos) < min(v_step, 1.0f))
            dt_pos = sign(csa.tc_pos - csa.cal_pos) * min(v_step, 1.0f);
        p64f += (double)dt_pos;
        int32_t p32i = lround(p64f);

        if (fabsf(csa.tc_vc) <= v_step * 4.4f) { // avoid exceeding
            csa.cal_pos = (csa.tc_pos >= csa.cal_pos) ? min(p32i, csa.tc_pos) : max(p32i, csa.tc_pos);
            if (csa.cal_pos == csa.tc_pos) {
                csa.tc_state = 0;
                csa.tc_vc = 0;
                csa.tc_ac = 0;
            }
        } else {
            csa.cal_pos = p32i;
        }

    } else {
        csa.tc_vc = 0;
        csa.tc_ac = 0;
    }
}

static inline void position_loop_compute(void)
{
    if (csa.state < ST_POSITION) {
        pid_i_reset(&csa.pid_pos, csa.sen_pos, 0);
        pid_i_set_target(&csa.pid_pos, csa.sen_pos);
        csa.cal_pos = csa.sen_pos;
        if (csa.state == ST_STOP)
            csa.cal_speed = 0;
        t_curve_compute(); // reset t_curve
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


// TODO: read sensor at same time of adc sampling

#define HIST_LEN 2
static uint16_t hist[HIST_LEN] = { 0 };
static int8_t hist_err = -2;

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    gpio_set_value(&dbg_out1, 1);

    float sin_sen_angle_elec, cos_sen_angle_elec; // reduce the amount of calculations
    int16_t out_pwm_u = 0, out_pwm_v = 0, out_pwm_w = 0;
    bool dbg_str = csa.dbg_str_msk && (csa.loop_cnt % csa.dbg_str_skip) == 0;

    uint16_t noc_encoder_bk = csa.noc_encoder;
    csa.ori_encoder = encoder_read();
    csa.noc_encoder = csa.ori_encoder - csa.bias_encoder;
    int16_t delta_enc = csa.noc_encoder - noc_encoder_bk;

    //gpio_set_value(&dbg_out2, 1);

#if 1
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

    hist[0] = hist[1];
    hist[1] = csa.sen_encoder;
#else
    csa.sen_encoder = csa.noc_encoder;
    csa.delta_encoder = delta_enc;
#endif

    // 9us lag steps = step/sec * 0.000009 sec
    csa.sen_encoder += delta_enc * ((float)CURRENT_LOOP_FREQ * 0.000019f); // 0.000009f

    //gpio_set_value(&dbg_out2, 0);


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

    {
        static uint32_t cumulative_a, cumulative_b, offset_a, offset_b;

        int32_t adc_a = HAL_ADCEx_InjectedGetValue(&hadc1, 1); // change ADC_SMPR1 sample time register
        int32_t adc_b = HAL_ADCEx_InjectedGetValue(&hadc2, 1);

        if (csa.state == ST_STOP) {
            cumulative_a = adc_a << CUR_AVG_WINDOW;
            cumulative_b = adc_b << CUR_AVG_WINDOW;
            offset_a = adc_a;
            offset_b = adc_b;
        }

        cumulative_a  += adc_a - offset_a;
        cumulative_b  += adc_b - offset_b;
        offset_a = clip(cumulative_a >> CUR_AVG_WINDOW, CUR_OFFSET_MIN, CUR_OFFSET_MAX);
        offset_b = clip(cumulative_b >> CUR_AVG_WINDOW, CUR_OFFSET_MIN, CUR_OFFSET_MAX);
        int32_t ia = adc_a - offset_a;
        int32_t ib = adc_b - offset_b;
        int32_t ic = -ia - ib;

        float i_alpha = ia;
        float i_beta = (ia + ib * 2) / 1.732050808f; // √3

        csa.sen_i_sq = -i_alpha * sin_sen_angle_elec + i_beta * cos_sen_angle_elec;
        csa.sen_i_sd = i_alpha * cos_sen_angle_elec + i_beta * sin_sen_angle_elec;

        if (dbg_str)
            //d_debug_c(", i %5d %5d %5d", ia, ib, ic);
            d_debug_c(", i %5d %5d", adc_a, adc_b);
        if (dbg_str)
            d_debug_c(" (q %5d.%.2d, d %5d.%.2d)", P_2F(csa.sen_i_sq), P_2F(csa.sen_i_sd));
    }

    // current --> pwm
    if (csa.state != ST_STOP) {
        float i_alpha, i_beta;

        // calculate angle
        if (csa.state != ST_CALI) {
            pid_f_set_target(&csa.pid_i_sq, csa.cal_current);
            csa.cal_i_sq = pid_f_compute_no_d(&csa.pid_i_sq, csa.sen_i_sq);
            csa.cal_i_sd = pid_f_compute_no_d(&csa.pid_i_sd, csa.sen_i_sd); // target default 0
            i_alpha = csa.cal_i_sd * cos_sen_angle_elec - csa.cal_i_sq * sin_sen_angle_elec;
            i_beta =  csa.cal_i_sd * sin_sen_angle_elec + csa.cal_i_sq * cos_sen_angle_elec;

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
        pid_f_set_target(&csa.pid_i_sq, 0);
        pid_f_set_target(&csa.pid_i_sd, 0);
        pid_f_reset(&csa.pid_i_sq, 0, 0);
        pid_f_reset(&csa.pid_i_sd, 0, 0);

        //peak_cur_cnt = 0;
        if (dbg_str)
            d_debug_c("\n");
    }

    //gpio_set_value(&led_r, !gpio_get_value(&led_r)); // debug for hw config

    // write 4095 to all pwm channel for brake (set A, B, C to zero)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, DRV_PWM_HALF - out_pwm_u); // TIM1_CH3: A
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, DRV_PWM_HALF - out_pwm_v); // TIM1_CH2: B
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, DRV_PWM_HALF - out_pwm_w); // TIM1_CH1: C

    speed_loop_compute();
    raw_dbg(0);
    csa.loop_cnt++;

#if 1
    if (!LL_ADC_REG_IsConversionOngoing(hadc1.Instance)) {

        //HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        int32_t adc_temperature = HAL_ADC_GetValue(&hadc1);

        //HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
        int32_t adc_dc = HAL_ADC_GetValue(&hadc2);

#if 0
        static uint32_t t_last = 0;
        if (get_systick() - t_last > 1000) {
            t_last = get_systick();
            d_info("temperature: %d, dc: %d\n", adc_temperature, adc_dc);
        }
#endif

        LL_ADC_REG_StartConversion(hadc1.Instance);
//          delay_systick(10);

    }
#endif

    uint16_t enc_check = encoder_read();
    if (enc_check != csa.ori_encoder)
        d_warn("encoder dat late\n");

    gpio_set_value(&dbg_out1, 0);
}
