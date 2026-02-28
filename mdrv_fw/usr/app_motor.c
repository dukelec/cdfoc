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

static adc_samp_t adc_samp = {0};
static encoder_filter_t enc_filter = {0};

static encoder_linearizer_t enc_lin = {
        .lut = (int8_t *)ENC_LIN_TBL
};

static anticog_t anticog = {
        .lut = (int8_t *)ENC_LIN_TBL
};

static trap_planner_t trap_planner = {
        .dt = 25.0f / CURRENT_LOOP_FREQ
};

static int vector_over_limit = 0;
static uint8_t pos_loop_cnt = 0;
static uint8_t speed_loop_cnt = 0;
static float cal_speed_bk = 0;
static int32_t cal_current_bk = 0;


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

        d_debug("drv 05: %04x\n", drv_read_reg(0x05)); // default 0x0159, 100ns dead time
        d_debug("drv 03: %04x\n", drv_read_reg(0x03)); // default 0x03ff
        d_debug("drv 04: %04x\n", drv_read_reg(0x04)); // default 0x07ff
        drv_write_reg(0x03, 0x0388); // 260mA, 520mA
        drv_write_reg(0x04, 0x0788); // 260mA, 520mA
        d_debug("drv 03: %04x\n", drv_read_reg(0x03));
        d_debug("drv 04: %04x\n", drv_read_reg(0x04));

        d_debug("drv 02: %04x\n", drv_read_reg(0x02)); // default 0x0000
        drv_write_reg(0x02, 0x1 << 2); // COAST mode
        d_debug("drv 02: %04x\n", drv_read_reg(0x02));

        delay_systick(5);
        d_debug("drv 06: %04x\n", drv_read_reg(0x06)); // default 0x0283
        drv_write_reg(0x06, 0x0283 | (7 << 2)); // cali amplifier
        d_debug("drv 06: %04x\n", drv_read_reg(0x06));
        delay_systick(5);
        drv_write_reg(0x06, 0x0283);
        d_debug("drv 06: %04x\n", drv_read_reg(0x06));
        delay_systick(5);

        adc_samp_cali(&adc_samp);

        drv_write_reg(0x02, (1 << 7) | (1 << 5)); // otw err, 3x pwm mode
        d_debug("drv 02: %04x\n", drv_read_reg(0x02));
    }
    return 0;
}

uint8_t motor_w_hook_after(uint16_t sub_offset, uint8_t len, uint8_t *dat)
{
    uint32_t flags;

    if (csa.state == ST_POS_TP) {
        local_irq_save(flags);
        if (csa.cal_pos != csa.tp_pos)
            trap_planner.state = 1; // restart t_curve
        local_irq_restore(flags);
    }
    return 0;
}


void app_motor_init(void)
{
    pid_f_reset(&csa.pid_i_sq, 0);
    pid_f_reset(&csa.pid_i_sd, 0);
    pid_f_reset(&csa.pid_speed, 0);
    pid_i_reset(&csa.pid_pos, 0);
    smo_init(&csa.smo, true);
    pll_init(&csa.pll, true);
    csa.bus_voltage = csa.nominal_voltage;
    csa2encoder_linearizer_mt(&enc_lin);
    csa2anticog_mt(&anticog);
}

void app_motor_maintain(void)
{
    static uint32_t t_last = 0;
    if (vector_over_limit && (t_last == 0 || get_systick() - t_last > 500)) {
        d_debug("!~!~ %d ~!~!\n", vector_over_limit);
        vector_over_limit = 0;
        t_last = get_systick();
    }
    smo_init(&csa.smo, false);
    pll_init(&csa.pll, false);

    csa2encoder_filter_mt(&enc_filter);
    csa2trap_planner_mt(&trap_planner);

    if (adc_samp.has_new_regular) {
        int16_t adc_dc = adc_samp.regular_i[1];
        int16_t adc_temperature = adc_samp.regular_i[0];

        float v_dc = (adc_dc / 4095.0f * 3.3f) / 4.7f * (4.7f + 75);
        csa.bus_voltage += (v_dc - csa.bus_voltage) * 0.05f;

        //    pull-up: 10K
        float r_ntc = (10000.0f * adc_temperature) / (4095 - adc_temperature);
        float temperature = (1.0f / ((1.0f / csa.ntc_b) * logf(r_ntc / csa.ntc_r25) + (1.0f / (25 + 273.15f))) - 273.15f);
        csa.temperature += (temperature - csa.temperature) * 0.02f;
        adc_samp.has_new_regular = false;

        if (csa.temperature > csa.temperature_err) {
            csa.err_flag_.motor_otsd = 1;
            state_w_hook_before(0, 0, (uint8_t []){ST_STOP});
            csa.state = ST_STOP;
        } else if (csa.temperature > csa.temperature_warn) {
            csa.err_flag_.motor_otw = 1;
        }
        if (csa.bus_voltage < csa.voltage_min)
            csa.err_flag_.motor_uvlo = 1;
        if (csa.bus_voltage > csa.voltage_max)
            csa.err_flag_.motor_ovlo = 1;
    }
}


static inline void position_loop_update(void)
{
    if (++pos_loop_cnt < 5)
        return;
    pos_loop_cnt = 0;

    if (csa.state != ST_POS_TP) {
        trap_planner_reset(&trap_planner, csa.sen_pos, csa.sen_speed_avg);
        trap_planner2csa_rst(&trap_planner);
    } else {
        csa2trap_planner(&trap_planner);
        trap_planner_update(&trap_planner, csa.sen_pos);
        trap_planner2csa(&trap_planner);
    }

    if (csa.state < ST_POSITION) {
        //pid_i_reset(&csa.pid_pos, csa.sen_speed_avg);
        pid_i_set_target(&csa.pid_pos, csa.sen_pos);
        csa.cal_pos = csa.sen_pos;
        cal_speed_bk = csa.sen_speed_avg;
        if (csa.state == ST_STOP) {
            csa.cal_speed = 0;
            cal_speed_bk = 0;
        }
    } else {
        pid_i_set_target(&csa.pid_pos, csa.cal_pos);
        cal_speed_bk = csa.cal_speed;
        csa.cal_speed = pid_i_update_p_only(&csa.pid_pos, csa.sen_pos);
        if (csa.state == ST_POS_TP)
            csa.cal_speed += csa.tp_vel_out;
    }

    raw_dbg(2);
    raw_dbg(3);
}


static inline void speed_loop_update(void)
{
    if (++speed_loop_cnt < 5)
        return;
    speed_loop_cnt = 0;
    position_loop_update();
    float sl_speed = (csa.pll.omega / csa.motor_poles) / (2 * M_PIf) * 0x10000;

    if (csa.state < ST_SPEED) {
        pid_f_reset(&csa.pid_speed, csa.cal_current);
        if (csa.sl_state) {
            pid_f_set_target(&csa.pid_speed, sl_speed);
            csa.cal_speed = sl_speed;
        } else {
            pid_f_set_target(&csa.pid_speed, csa.sen_speed_avg);
            csa.cal_speed = csa.sen_speed_avg;
        }
        if (csa.state == ST_STOP) {
            csa.cal_current = 0;
            csa.cal_speed = 0;
            cal_speed_bk = 0;
        }
        cal_current_bk = csa.cal_current;
    } else {
        if (csa.state == ST_SPEED) {
            float v_step = (float)csa.tp_accel / (CURRENT_LOOP_FREQ / 5.0f);
            float speed = csa.pid_speed.target <= csa.cal_speed ?
                    min(csa.pid_speed.target + v_step, csa.cal_speed) : max(csa.pid_speed.target - v_step, csa.cal_speed);
            pid_f_set_target(&csa.pid_speed, speed);
        } else {
            float speed = cal_speed_bk + (csa.cal_speed - cal_speed_bk) / 5 * (pos_loop_cnt + 1);
            pid_f_set_target(&csa.pid_speed, speed);
        }
        cal_current_bk = csa.cal_current;
        if (csa.sl_state) {
            csa.cal_current = lroundf(pid_f_update(&csa.pid_speed, sl_speed, sl_speed));
        } else {
            csa.cal_current = lroundf(pid_f_update(&csa.pid_speed, csa.sen_speed_avg, csa.sen_speed));
        }
    }

    raw_dbg(1);
}


void current_loop_update(void)
{
    int16_t anticog_iq = 0, anticog_vq = 0;
    float voltage_mag = 0;

    gpio_set_val(&dbg_out1, 1);
    gpio_set_val(&s_cs, 1);

    float voltage_ratio = csa.bus_voltage / csa.nominal_voltage;
    float sin_tmp_angle_elec, cos_tmp_angle_elec; // reduce the amount of calculations

    adc_samp_inject(&adc_samp, csa.state == ST_STOP, csa.motor_wire_swap);
    adc_samp2csa(&adc_samp);

    float i_alpha = csa.sen_i[0];
    float i_beta = (csa.sen_i[0] + csa.sen_i[1] * 2) / 1.7320508f; // √3

    if (csa.state != ST_STOP) {
        csa.smo.i_alpha_real = i_alpha * 3.3f / 2047 / 20.0f / 0.02f; // 20 V/V amplifier gain
        csa.smo.i_beta_real = i_beta * 3.3f / 2047 / 20.0f / 0.02f;
        smo_update(&csa.smo);
        pll_update(&csa.pll, csa.smo.e_alpha, csa.smo.e_beta);
    }

    csa.ori_encoder = encoder_read();
    uint16_t cali_encoder = csa.ori_encoder;
    if (csa.encoder_linearizer_en)
        cali_encoder = encoder_linearizer(&enc_lin, csa.ori_encoder);
    encoder_filter(&enc_filter, cali_encoder);
    encoder_filter2csa(&enc_filter);

    speed_loop_update();

    float encoder_sub_range = (float)0x10000 / csa.motor_poles;
    float encoder_sub = csa.sen_encoder - (int)(csa.sen_encoder / encoder_sub_range) * encoder_sub_range; // fmodf
    csa.sen_angle_elec = encoder_sub * (M_PIf * 2 / encoder_sub_range);

    if (csa.state == ST_CALI) {
        if (fabsf(csa.cali_angle_speed - csa.cali_angle_speed_tgt) >= 0.01f) {
            if (csa.cali_angle_speed < csa.cali_angle_speed_tgt)
                csa.cali_angle_speed += 0.01f;
            else
                csa.cali_angle_speed -= 0.01f;
        } else {
            csa.cali_angle_speed = csa.cali_angle_speed_tgt;
        }
        csa.cali_angle_elec += csa.cali_angle_speed * (1.0f / CURRENT_LOOP_FREQ);
        if (csa.cali_angle_elec >= M_PIf * 2)
            csa.cali_angle_elec -= M_PIf * 2;
        else if (csa.cali_angle_elec < 0)
            csa.cali_angle_elec += M_PIf * 2;
        sin_tmp_angle_elec = sinf(csa.cali_angle_elec);
        cos_tmp_angle_elec = cosf(csa.cali_angle_elec);
    } else if (csa.sl_state) {
        sin_tmp_angle_elec = sinf(csa.pll.theta); // todo: add compensation
        cos_tmp_angle_elec = cosf(csa.pll.theta);
    } else {
        csa.cali_angle_speed = 0;
        sin_tmp_angle_elec = sinf(csa.sen_angle_elec);
        cos_tmp_angle_elec = cosf(csa.sen_angle_elec);
    }

    csa.sen_i_sq = -i_alpha * sin_tmp_angle_elec + i_beta * cos_tmp_angle_elec;
    csa.sen_i_sd = i_alpha * cos_tmp_angle_elec + i_beta * sin_tmp_angle_elec;

    if (csa.anticog_en)
        anticog_ff(&anticog, csa.sen_encoder, &anticog_iq, &anticog_vq);
    float err_i_sq = csa.sen_i_sq - csa.sen_i_sq_avg;
    csa.sen_i_sq_avg += err_i_sq * 0.001f;

    // current -> pwm
    if (csa.state != ST_STOP) {
        float v_alpha, v_beta;
        int32_t target_current;
        if (csa.state >= ST_SPEED) {
            float current = cal_current_bk + (csa.cal_current - cal_current_bk) * (speed_loop_cnt + 1) / 5.0f;
            target_current = lroundf(current);
        } else if (csa.state == ST_CALI) {
            target_current = csa.cali_current;
        } else {
            target_current = csa.cal_current;
        }

        pid_f_set_target(&csa.pid_i_sq, target_current + anticog_iq);
        csa.cal_v_sq = (pid_f_update(&csa.pid_i_sq, csa.sen_i_sq, csa.sen_i_sq) + anticog_vq) / voltage_ratio;
        csa.cal_v_sd = pid_f_update(&csa.pid_i_sd, csa.sen_i_sd, csa.sen_i_sd) / voltage_ratio;
        if (csa.state == ST_CALI) {
            pid_f_reset(&csa.pid_i_sd, 0);
            csa.cal_v_sd = 0;
        }

        float err_v_sq = csa.cal_v_sq - csa.cal_v_sq_avg;
        csa.cal_v_sq_avg += err_v_sq * 0.001f;

        v_alpha = csa.cal_v_sd * cos_tmp_angle_elec - csa.cal_v_sq * sin_tmp_angle_elec;
        v_beta =  csa.cal_v_sd * sin_tmp_angle_elec + csa.cal_v_sq * cos_tmp_angle_elec;
        voltage_mag = svpwm(v_alpha, v_beta, csa.pwm_uvw, csa.pwm_dbg0, csa.sen_i);
        float scale = 1;
        if (voltage_mag > SVPWM_MAX_MAG) {
            vector_over_limit = voltage_mag;
            scale = SVPWM_MAX_MAG / voltage_mag;
        }
        csa.smo.v_alpha_real = (v_alpha * scale / 2047) * (csa.bus_voltage / 2);
        csa.smo.v_beta_real = (v_beta * scale / 2047) * (csa.bus_voltage / 2);
    } else {
        csa.cal_v_sq = 0;
        pid_f_set_target(&csa.pid_i_sq, 0);
        pid_f_set_target(&csa.pid_i_sd, 0);
        pid_f_reset(&csa.pid_i_sq, 0);
        pid_f_reset(&csa.pid_i_sd, 0);
        memset(csa.pwm_dbg0, 0, 2 * 3 * 3); // include pwm_dbg1 pwm_uvw
        smo_init(&csa.smo, true);
        pll_init(&csa.pll, true);
    }

    adc_samp_set_ch(&adc_samp, csa.pwm_uvw, csa.state != ST_STOP && voltage_mag > SVPWM_MAX_MAG / 2);
    motor_pwm_output(csa.pwm_uvw, csa.motor_wire_swap);

    adc_samp_regular(&adc_samp);
    raw_dbg(0);
    csa.loop_cnt++;

    uint16_t enc_check = encoder_read();
    if (enc_check != csa.ori_encoder)
        d_warn("encoder dat late\n");
    encoder_isr_prepare();
    gpio_set_val(&dbg_out1, 0);
}
