/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include <math.h>
#include "app_main.h"

#define ADC_CALI_LEN 50
static int adc_cali_cnt;
static uint32_t adc_ofs[3][2];
static volatile int adc_cali_st = 0;

static int vector_over_limit = 0;

static uint16_t pos_rec[5] = {0};
static int32_t pos_tmp[5];
static float speed_rec[5] = {0};

static uint8_t pos_loop_cnt = 0;
static uint8_t speed_loop_cnt = 0;
static float cal_speed_bk = 0;
static int32_t cal_current_bk = 0;

static volatile bool adc_has_new;
static volatile uint32_t adc_temperature;
static volatile uint32_t adc_dc;



uint8_t state_w_hook_before(uint16_t sub_offset, uint8_t len, uint8_t *dat)
{
    if (*dat == ST_STOP) {
        gpio_set_val(&drv_en, 0);
        gpio_set_val(&led_r, 0);
        csa.adc_sel = 0;

    } else if (csa.state == ST_STOP && *dat != ST_STOP) {
        gpio_set_val(&drv_en, 1);
        csa.adc_sel = 0;
        delay_systick(50);

        d_debug("drv 05: %04x\n", drv_read_reg(0x05)); // default 0x016d
        drv_write_reg(0x05, 0x036d); // 400ns dead time
        d_debug("drv 05: %04x\n", drv_read_reg(0x05));

        d_debug("drv 03: %04x\n", drv_read_reg(0x03)); // default 0x03ff
        d_debug("drv 04: %04x\n", drv_read_reg(0x04)); // default 0x07ff
        drv_write_reg(0x03, 0x0300); // 10mA, 20mA
        drv_write_reg(0x04, 0x0700); // 10mA, 20mA
        d_debug("drv 03: %04x\n", drv_read_reg(0x03));
        d_debug("drv 04: %04x\n", drv_read_reg(0x04));

        d_debug("drv 02: %04x\n", drv_read_reg(0x02)); // default 0x0000
        drv_write_reg(0x02, 0x1 << 2); // COAST mode
        d_debug("drv 02: %04x\n", drv_read_reg(0x02));

        delay_systick(5);
        csa.adc_sel = 0;
        memset(adc_ofs, 0, sizeof(adc_ofs));
        adc_cali_cnt = 0;
        adc_cali_st = 1; // start adc_cali
        while (adc_cali_st);
        delay_systick(5);
        d_debug("adc cali: ab %lu %lu, ca %lu %lu, cb %lu %lu\n",
                adc_ofs[0][0], adc_ofs[0][1], adc_ofs[1][0], adc_ofs[1][1], adc_ofs[2][0], adc_ofs[2][1]);

        drv_write_reg(0x02, (1 << 10) | (1 << 7) | (1 << 5)); // shutdown all on err, otw err, 3x pwm mode
        d_debug("drv 02: %04x\n", drv_read_reg(0x02));
    }
    return 0;
}

uint8_t motor_w_hook_after(uint16_t sub_offset, uint8_t len, uint8_t *dat)
{
    uint32_t flags;

    if (csa.state == ST_POS_TC) {
        local_irq_save(flags);
        if (csa.cal_pos != csa.tc_pos)
            csa.tc_state = 1; // restart t_curve
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
    csa.bus_voltage = csa.nominal_voltage;
}

void app_motor_routine(void)
{
    static uint32_t t_last = 0;
    if (vector_over_limit && (t_last == 0 || get_systick() - t_last > 500)) {
        d_debug("\n\n!~!~ %d ~!~!\n\n", vector_over_limit);
        vector_over_limit = 0;
        t_last = get_systick();
    }
    // update _ki, _kd
    pid_f_init(&csa.pid_i_sq, false);
    pid_f_init(&csa.pid_i_sd, false);
    pid_f_init(&csa.pid_speed, false);
    pid_i_init(&csa.pid_pos, false);

    if (adc_has_new) {
        float v_dc = (adc_dc / 4095.0f * 3.3f) / 4.7f * (4.7f + 75);
        csa.bus_voltage += (v_dc - csa.bus_voltage) * 0.001f;

        #define _B 3970
        float r_ntc = (10000.0f * adc_temperature) / (4095 - adc_temperature);
        //    pull-up: 10K                              r_ntc = 100K  ->          @ 25°C
        float temperature = (1.0f / ((1.0f / _B) * logf(r_ntc / 100000) + (1.0f / (25 + 273.15f))) - 273.15f);
        csa.temperature += (temperature - csa.temperature) * 0.001f;
        adc_has_new = false;
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
        float v_step = (float)csa.tc_accel / (CURRENT_LOOP_FREQ / 25.0f);

        if (csa.tc_pos != csa.cal_pos) {
            // t = (v1 - v2) / a; s = ((v1 + v2) / 2) * t; a =>
            csa.tc_ac = ((/* tc_ve + */ csa.tc_vc) / 2.0f) * (/* tc_ve */ - csa.tc_vc) / (csa.tc_pos - csa.cal_pos);
            csa.tc_ac = min(fabsf(csa.tc_ac), csa.tc_accel * 1.2f);
        } else {
            csa.tc_ac = csa.tc_accel * 1.2f;
        }

        if (csa.tc_ac >= csa.tc_accel) {
            float delta_v = csa.tc_ac / (CURRENT_LOOP_FREQ / 25.0f);
            csa.tc_vc += -sign(csa.tc_vc) * delta_v;
        } else {
            float target_v = ((csa.tc_pos >= csa.cal_pos) ? 1 : -1) * (float)csa.tc_speed;
            float delta_v = ((target_v >= csa.tc_vc) ? 1 : -1) * min(v_step, fabsf(target_v - csa.tc_vc));
            csa.tc_vc += delta_v;
        }

        float dt_pos = csa.tc_vc / (CURRENT_LOOP_FREQ / 25.0f);
        p64f += (double)dt_pos;
        int32_t p32i = lround(p64f);

        if (csa.tc_max_err && abs(csa.sen_pos - p32i) > csa.tc_max_err) {
            p32i = clip(p32i, csa.sen_pos - csa.tc_max_err, csa.sen_pos + csa.tc_max_err);
            p64f = p32i;
            // todo: set warning flag
        }

        if (fabsf(csa.tc_vc) <= v_step * 4.4f) { // avoid exceeding
            csa.cal_pos = (csa.tc_pos >= csa.cal_pos) ? min(p32i, csa.tc_pos) : max(p32i, csa.tc_pos);
            if (csa.cal_pos == csa.tc_pos) {
                csa.tc_state = 0;
                csa.tc_vc = 0;
                csa.tc_ac = 0;
                p64f = csa.cal_pos;
            }
        } else {
            csa.cal_pos = p32i;
        }

    } else {
        csa.tc_vc = 0;
        csa.tc_ac = 0;
    }
    csa.tc_vc_avg += (csa.tc_vc - csa.tc_vc_avg) * 0.1f;
}


static inline void position_loop_compute(void)
{
    if (++pos_loop_cnt < 5)
        return;
    pos_loop_cnt = 0;

    if (csa.state < ST_POSITION) {
        pid_i_reset(&csa.pid_pos, csa.sen_pos, 0);
        pid_i_set_target(&csa.pid_pos, csa.sen_pos);
        csa.cal_pos = csa.sen_pos;
        if (csa.state == ST_STOP) {
            csa.cal_speed = 0;
            cal_speed_bk = 0;
        }
        t_curve_compute(); // reset t_curve
        return;
    }

    t_curve_compute();
    csa.pid_pos.out_min = csa.tc_vc_avg - 65536;
    csa.pid_pos.out_max = csa.tc_vc_avg + 65536;
    pid_i_set_target(&csa.pid_pos, csa.cal_pos);
    cal_speed_bk = csa.cal_speed;
    csa.cal_speed = pid_i_compute(&csa.pid_pos, csa.sen_pos);

    raw_dbg(2);
    raw_dbg(3);
}


static inline void speed_loop_compute(void)
{
    if (++speed_loop_cnt < 5)
        return;
    speed_loop_cnt = 0;
    position_loop_compute();

    if (csa.state < ST_SPEED) {
        pid_f_reset(&csa.pid_speed, 0, 0);
        pid_f_set_target(&csa.pid_speed, 0);
        if (csa.state == ST_STOP) {
            csa.cal_current = 0;
            cal_current_bk = 0;
            csa.cal_speed = 0;
            cal_speed_bk = 0;
        }
    } else {
        if (csa.state == ST_SPEED) {
            float v_step = (float)csa.tc_accel / (CURRENT_LOOP_FREQ / 5.0f);
            float speed = csa.pid_speed.target <= csa.cal_speed ?
                    min(csa.pid_speed.target + v_step, csa.cal_speed) : max(csa.pid_speed.target - v_step, csa.cal_speed);
            pid_f_set_target(&csa.pid_speed, speed);
        } else {
            float speed = cal_speed_bk + (csa.cal_speed - cal_speed_bk) / 5 * (pos_loop_cnt + 1);
            pid_f_set_target(&csa.pid_speed, speed);
        }
        cal_current_bk = csa.cal_current;
        csa.cal_current = lroundf(pid_f_compute(&csa.pid_speed, csa.sen_speed_avg, csa.sen_speed));
    }

    raw_dbg(1);
}


void adc_isr(void)
{
    gpio_set_val(&dbg_out1, 1);
    gpio_set_val(&s_cs, 1);

    uint16_t tbl_idx = 0;
    uint16_t tbl_idx_next = 0;
    float tbl_percent = 0;
    float voltage_ratio = csa.bus_voltage / csa.nominal_voltage;

    float sin_tmp_angle_elec, cos_tmp_angle_elec; // reduce the amount of calculations
    int16_t out_pwm_u = 0, out_pwm_v = 0, out_pwm_w = 0;
    bool dbg_str = csa.dbg_str_msk && (csa.loop_cnt % csa.dbg_str_skip) == 0;

    csa.ori_encoder = encoder_read();
    uint16_t cali_encoder = csa.ori_encoder;
    if (csa.cali_encoder_en) {
        uint16_t idx = csa.ori_encoder >> 4;
        uint16_t idx_next = (idx == 4095) ? 0 : (idx + 1);
        tbl_percent = (float)(csa.ori_encoder & 0xf) / 0x10;
        uint16_t idx_val = *(uint16_t *)(CALI_ENCODER_TBL + idx * 2);
        uint32_t idx_next_val = *(uint16_t *)(CALI_ENCODER_TBL + idx_next * 2);
        if (idx == 4095)
            idx_next_val += 0x10000;
        cali_encoder = lroundf(idx_val + (idx_next_val - idx_val) * tbl_percent);
    }

    //gpio_set_val(&dbg_out2, 1);

    for (int i = 0; i < 4; i++)
        pos_rec[i] = pos_rec[i+1];
    pos_rec[4] = cali_encoder;
    pos_tmp[0] = pos_rec[0];
    for (int i = 0; i < 4; i++) {
        int16_t dt = pos_rec[i+1] - pos_rec[0];
        pos_tmp[i+1] = pos_tmp[0] + dt;
    }
    int32_t pos_min = pos_tmp[0];
    int32_t pos_max = pos_tmp[0];
    int32_t pos_sum = 0;
    for (int i = 0; i < 5; i++) {
        pos_sum += pos_tmp[i];
        pos_min = min(pos_min, pos_tmp[i]);
        pos_max = max(pos_max, pos_tmp[i]);
    }
    uint16_t pos_avg = DIV_ROUND_CLOSEST(pos_sum - pos_min - pos_max, 3);
    csa.delta_encoder = (int16_t)(pos_avg - csa.nob_encoder);
    csa.nob_encoder = pos_avg;

    for (int i = 0; i < 4; i++)
        speed_rec[i] = speed_rec[i+1];
    speed_rec[4] = csa.delta_encoder * CURRENT_LOOP_FREQ; // encoder steps per sec
    float speed_sum = 0;
    for (int i = 0; i < 5; i++)
        speed_sum += speed_rec[i];
    csa.sen_speed = speed_sum / 5;
    csa.sen_speed_avg += (csa.sen_speed - csa.sen_speed_avg) * 0.1f;
    csa.sen_rpm_avg += (csa.sen_speed_avg / 0x10000 * 60 - csa.sen_rpm_avg) * 0.01f;

    // position compensation: step/sec * (2 loop period + 0.000019 sec)
    int16_t pos_speed_comp = lroundf(csa.sen_speed_avg * (2.0f / CURRENT_LOOP_FREQ + 0.000019f));
    csa.sen_encoder = csa.nob_encoder - csa.bias_encoder + pos_speed_comp;

    //gpio_set_val(&dbg_out2, 0);

    csa.ori_pos += (int16_t)(csa.sen_encoder - (uint16_t)csa.ori_pos);
    csa.sen_pos = csa.ori_pos - csa.bias_pos;

    if (dbg_str)
            d_debug("%04x %04x %08lx", csa.ori_encoder, csa.sen_encoder, csa.sen_pos);
    speed_loop_compute();

    float angle_mech = csa.sen_encoder*((float)M_PI*2/0x10000);
    uint16_t encoder_sub = csa.sen_encoder % lroundf((float)0x10000/csa.motor_poles);
    csa.sen_angle_elec = encoder_sub*((float)M_PI*2/((float)0x10000/csa.motor_poles));
    if (csa.state == ST_CALI) {
        csa.cali_angle_elec += csa.cali_angle_step;
        if (csa.cali_angle_elec >= (float)M_PI*2)
            csa.cali_angle_elec -= (float)M_PI*2;
        sin_tmp_angle_elec = sinf(csa.cali_angle_elec);
        cos_tmp_angle_elec = cosf(csa.cali_angle_elec);
    } else {
        sin_tmp_angle_elec = sinf(csa.sen_angle_elec);
        cos_tmp_angle_elec = cosf(csa.sen_angle_elec);
    }

    if (dbg_str)
        d_debug_c(" a %d.%.2d %d.%.2d %d.%.2d", P_2F(angle_mech), P_2F(csa.sen_angle_elec), P_2F(csa.cali_angle_elec));

    {
        int32_t adc1_val = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1); // change ADC_SMPR1 sample time register
        int32_t adc2_val = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        int32_t ia, ib, ic;

        if (csa.adc_sel == 0) {
            ia = adc1_val - adc_ofs[0][0];
            ib = adc2_val - adc_ofs[0][1];
            ic = -ia - ib;
        } else if (csa.adc_sel == 1) {
            ic = adc1_val - adc_ofs[1][0];
            ia = adc2_val - adc_ofs[1][1];
            ib = -ia - ic;
        } else {
            ic = adc1_val - adc_ofs[2][0];
            ib = adc2_val - adc_ofs[2][1];
            ia = -ic - ib;
        }

        if (csa.state == ST_STOP && adc_cali_st) {
            adc_ofs[csa.adc_sel][0] += adc1_val;
            adc_ofs[csa.adc_sel][1] += adc2_val;
            if (++adc_cali_cnt >= ADC_CALI_LEN) {
                adc_cali_cnt = 0;
                adc_ofs[csa.adc_sel][0] = DIV_ROUND_CLOSEST(adc_ofs[csa.adc_sel][0], ADC_CALI_LEN);
                adc_ofs[csa.adc_sel][1] = DIV_ROUND_CLOSEST(adc_ofs[csa.adc_sel][1], ADC_CALI_LEN);
                if (++csa.adc_sel == 3) {
                    csa.adc_sel = 0;
                    adc_cali_st = 0;
                }
            }
        }

        csa.dbg_ia = ia;
        csa.dbg_ib = ib;
        if (csa.motor_wire_swap)
            swap(ia, ib);

        float i_alpha = ia;
        float i_beta = (ia + ib * 2) / 1.732050808f; // √3

        csa.sen_i_sq = -i_alpha * sin_tmp_angle_elec + i_beta * cos_tmp_angle_elec;
        csa.sen_i_sd = i_alpha * cos_tmp_angle_elec + i_beta * sin_tmp_angle_elec;

        if (csa.anticogging_en) {
            uint16_t pos = csa.nob_encoder + pos_speed_comp;
            tbl_idx = pos >> 4;
            tbl_idx_next = (tbl_idx == 4095) ? 0 : (tbl_idx + 1);
            tbl_percent = (float)(pos & 0xf) / 0x10;
            int8_t idx_val = *(int8_t *)(ANTICOGGING_TBL + tbl_idx * 2);
            int8_t idx_next_val = *(int8_t *)(CALI_ENCODER_TBL + tbl_idx_next * 2);
            float val = idx_val + (idx_next_val - idx_val) * tbl_percent;
            csa.sen_i_sq -= csa.anticogging_max_val[0] * val / 100.0f;
        }
        float err_i_sq = csa.sen_i_sq - csa.sen_i_sq_avg;
        csa.sen_i_sq_avg += err_i_sq * 0.001f;

        if (dbg_str)
            //d_debug_c(", i %5d %5d %5d", ia, ib, ic);
            d_debug_c(", i %5ld %5ld", adc1_val, adc2_val);
        if (dbg_str)
            d_debug_c(" (q %5d.%.2d, d %5d.%.2d)", P_2F(csa.sen_i_sq), P_2F(csa.sen_i_sd));
    }

    // current --> pwm
    if (csa.state != ST_STOP) {
        float v_alpha, v_beta;
        int32_t target_current;
        if (csa.state >= ST_SPEED) {
            float current = cal_current_bk + (float)(csa.cal_current - cal_current_bk) * (speed_loop_cnt + 1) / 5;
            target_current = lroundf(current);
        } else if (csa.state == ST_CALI) {
            target_current = -csa.cali_current;
        } else {
            target_current = csa.cal_current;
        }
        // sign select direction for cali_current

#if defined(CAL_CURRENT_SMOOTH)
        static bool near_limit = false;
        if (!near_limit) {
            if (fabsf(csa.pid_i_sq.target - target_current) <= 0.01f)
                pid_f_set_target(&csa.pid_i_sq, target_current);
            else
                csa.pid_i_sq.target += sign(target_current - csa.pid_i_sq.target) * 0.01f;
        } else {
            csa.pid_i_sq.target -= sign(csa.pid_i_sq.target) * 0.005f;
        }
#else
        pid_f_set_target(&csa.pid_i_sq, target_current);
#endif
        csa.cal_v_sq = pid_f_compute(&csa.pid_i_sq, csa.sen_i_sq, csa.sen_i_sq) / voltage_ratio;
        csa.cal_v_sd = pid_f_compute(&csa.pid_i_sd, csa.sen_i_sd, csa.sen_i_sd) / voltage_ratio; // target default 0

        if (csa.anticogging_en) {
            int8_t idx_val = *(int8_t *)(ANTICOGGING_TBL + tbl_idx * 2 + 1);
            int8_t idx_next_val = *(int8_t *)(CALI_ENCODER_TBL + tbl_idx_next * 2 + 1);
            float val = idx_val + (idx_next_val - idx_val) * tbl_percent;
            csa.cal_v_sq += (csa.anticogging_max_val[1] * val / 100.0f) / voltage_ratio;
        }
        float err_v_sq = csa.cal_v_sq - csa.cal_v_sq_avg;
        csa.cal_v_sq_avg += err_v_sq * 0.001f;

        if (csa.state == ST_CALI)
            csa.cal_v_sd = 0;

        // rotate 2d vector, origin: (sd, sq), after: (alpha, beta)
        v_alpha = csa.cal_v_sd * cos_tmp_angle_elec - csa.cal_v_sq * sin_tmp_angle_elec;
        v_beta =  csa.cal_v_sd * sin_tmp_angle_elec + csa.cal_v_sq * cos_tmp_angle_elec;
        // limit vector magnitude
        float norm = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
        float limit = 2047 * 1.15f; // deliver 15% more power by svpwm
        if (norm > limit) {
            v_alpha *= limit / norm;
            v_beta *= limit / norm;
            vector_over_limit = norm; // for debug
        }
#if defined(CAL_CURRENT_SMOOTH)
        near_limit = norm > (limit * 0.97f); // 0.96
#endif

        out_pwm_u = lroundf(v_alpha);
        out_pwm_v = lroundf(-v_alpha / 2 + v_beta * 0.866025404f); // (√3÷2)
        out_pwm_w = -out_pwm_u - out_pwm_v;
        // avoid over range again
        int16_t out_min = min(out_pwm_u, min(out_pwm_v, out_pwm_w));
        int16_t out_max = max(out_pwm_u, max(out_pwm_v, out_pwm_w));
        int16_t out_mid = (out_max + out_min) / 2;
        out_pwm_u = clip(out_pwm_u - out_mid, -2047, 2047);
        out_pwm_v = clip(out_pwm_v - out_mid, -2047, 2047);
        out_pwm_w = clip(out_pwm_w - out_mid, -2047, 2047);

        /*
        append_dprintf(DBG_CURRENT, "u:%.2f %.2f %d ", pid_cur_u.target, pid_cur_u.i_term, out_pwm_u);
        append_dprintf(DBG_CURRENT, "v:%.2f %.2f %d ", pid_cur_v.target, pid_cur_v.i_term, out_pwm_v);
        append_dprintf(DBG_CURRENT, "w:%d", out_pwm_w);
        */
        if (dbg_str)
            d_debug_c(", o %5d.%.2d %5d %5d %5d\n", P_2F(csa.cal_v_sq), out_pwm_u, out_pwm_v, out_pwm_w);
        if (csa.motor_wire_swap)
            swap(out_pwm_u, out_pwm_v);
    } else {
        csa.cal_v_sq = 0;
        pid_f_set_target(&csa.pid_i_sq, 0);
        pid_f_set_target(&csa.pid_i_sd, 0);
        pid_f_reset(&csa.pid_i_sq, 0, 0);
        pid_f_reset(&csa.pid_i_sd, 0, 0);

        //peak_cur_cnt = 0;
        if (dbg_str)
            d_debug_c("\n");
    }

    // select best csa.adc_sel
    if (csa.state != ST_STOP) {
        // DRV_PWM_HALF * 0.6 = 1228.8 (the cross point is 0.5)
        if ((csa.adc_sel == 0 && max(out_pwm_u, out_pwm_v) > 1229) ||
            (csa.adc_sel == 1 && max(out_pwm_w, out_pwm_u) > 1229) ||
            (csa.adc_sel == 2 && max(out_pwm_w, out_pwm_v) > 1229)) {

            if (out_pwm_w > max(out_pwm_u, out_pwm_v)) {
                csa.adc_sel = 0;
            } else if (out_pwm_v > max(out_pwm_w, out_pwm_u)) {
                csa.adc_sel = 1;
            } else {
                csa.adc_sel = 2;
            }
        }
    }
    if (csa.adc_sel == 0) {
        hadc1.Instance->JSQR = (hadc1.Instance->JSQR & 0x1ff) | (1 << 9); // a
        hadc2.Instance->JSQR = (hadc2.Instance->JSQR & 0x1ff) | (2 << 9); // b
    } else if (csa.adc_sel == 1) {
        hadc1.Instance->JSQR = (hadc1.Instance->JSQR & 0x1ff) | (3 << 9); // c
        hadc2.Instance->JSQR = (hadc2.Instance->JSQR & 0x1ff) | (1 << 9); // a
    } else {
        hadc1.Instance->JSQR = (hadc1.Instance->JSQR & 0x1ff) | (3 << 9); // c
        hadc2.Instance->JSQR = (hadc2.Instance->JSQR & 0x1ff) | (2 << 9); // b
    }

    //gpio_set_val(&led_r, !gpio_get_val(&led_r)); // debug for hw config
    csa.dbg_u = out_pwm_u;
    csa.dbg_v = out_pwm_v;

    // write 4095 to all pwm channel for brake (set A, B, C to zero)
    // pwm range: [1, 4095]
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, DRV_PWM_HALF - out_pwm_u); // TIM1_CH3: A
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, DRV_PWM_HALF - out_pwm_v); // TIM1_CH2: B
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, DRV_PWM_HALF - out_pwm_w); // TIM1_CH1: C

    raw_dbg(0);
    csa.loop_cnt++;

    if (!LL_ADC_REG_IsConversionOngoing(hadc1.Instance)) {
        uint32_t adc1_val = HAL_ADC_GetValue(&hadc1); // clear flag by read
        uint32_t adc2_val = HAL_ADC_GetValue(&hadc2);
        LL_ADC_REG_StartConversion(hadc1.Instance);
        if (!adc_has_new) {
            adc_temperature = adc1_val;
            adc_dc = adc2_val;
            adc_has_new = true;
        }
    }

    uint16_t enc_check = encoder_read();
    if (enc_check != csa.ori_encoder)
        d_warn("encoder dat late\n");

    encoder_isr_prepare();
    gpio_set_val(&dbg_out1, 0);
}
