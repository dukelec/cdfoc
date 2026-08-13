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
        .lut = (int8_t *)ANTICOG_TBL
};

static trap_planner_t trap_planner = {
        .dt = 25.0f / CURRENT_LOOP_FREQ
};

static pid_i_t pid_pos = {
        .dt = 25.0f / CURRENT_LOOP_FREQ
};

static pid_f_t pid_speed = {
        .dt = 5.0f / CURRENT_LOOP_FREQ
};

static pid_f_t pid_iq = {
        .dt = 1.0f / CURRENT_LOOP_FREQ
};

static pid_f_t pid_id = {
        .dt = 1.0f / CURRENT_LOOP_FREQ
};

static smo_t smo = {
        .delta_t = 1.0f / CURRENT_LOOP_FREQ
};

static pll_t pll = {
        .delta_t = 1.0f / CURRENT_LOOP_FREQ
};

static int vector_over_limit = 0;
static uint8_t pos_loop_cnt = 0;
static uint8_t speed_loop_cnt = 0;
static float tgt_speed_bk = 0;
static int16_t tgt_iq_bk = 0;
static float inv_voltage_ratio = 1.0f;

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
        if (csa.tgt_pos != csa.tp_pos)
            trap_planner.state = 1; // restart t_curve
        local_irq_restore(flags);
    }
    return 0;
}


void app_motor_init(void)
{
    csa2pid_mt(&pid_pos, &pid_speed, &pid_iq, &pid_id, 1.0f);
    csa2smo_mt(&smo);
    csa2pll_mt(&pll);
    pid_f_reset(&pid_iq, 0);
    pid_f_reset(&pid_id, 0);
    pid_f_reset(&pid_speed, 0);
    pid_i_reset(&pid_pos, 0);
    smo_init(&smo, true);
    pll_init(&pll, true);
    smo2csa(&smo);
    pll2csa(&pll);
    csa.bus_voltage = csa.nominal_voltage;
    csa.bus_voltage_f = csa.nominal_voltage / 10.0f;
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

    if (adc_samp.has_new_regular) {
        int16_t adc_dc = adc_samp.regular_i[1];
        int16_t adc_temp = adc_samp.regular_i[0];

        float v_dc = (adc_dc / 4095.0f * 3.3f) / 4.7f * (4.7f + 75);
        csa.bus_voltage_f += (v_dc - csa.bus_voltage_f) * 0.05f;
        csa.bus_voltage = lroundf(csa.bus_voltage_f * 10);

        //    pull-up: 10K
        float r_ntc = (10000.0f * adc_temp) / (4095 - adc_temp);
        float temp = (1.0f / ((1.0f / csa.ntc_b) * logf(r_ntc / csa.ntc_r25) + (1.0f / (25 + 273.15f))) - 273.15f);
        csa.motor_temp_f += (temp - csa.motor_temp_f) * 0.02f;
        csa.motor_temp = lroundf(csa.motor_temp_f * 10);
        adc_samp.has_new_regular = false;

        if (csa.motor_temp > csa.temp_err) {
            csa.error_flag_.motor_ot = 1;
            state_w_hook_before(0, 0, (uint8_t []){ST_STOP});
            csa.state = ST_STOP;
        } else if (csa.motor_temp > csa.temp_warn) {
            csa.warn_flag_.motor_ot = 1;
        }
        if (csa.bus_voltage < csa.voltage_min)
            csa.error_flag_.bus_uv = 1;
        if (csa.bus_voltage > csa.voltage_max)
            csa.error_flag_.bus_ov = 1;
    }

    csa2encoder_filter_mt(&enc_filter);
    csa2trap_planner_mt(&trap_planner);

    inv_voltage_ratio = (float)csa.nominal_voltage / csa.bus_voltage;
    csa2pid_mt(&pid_pos, &pid_speed, &pid_iq, &pid_id, inv_voltage_ratio);
    csa2smo_mt(&smo);
    csa2pll_mt(&pll);
}


static inline void position_loop_update(void)
{
    if (++pos_loop_cnt < 5)
        return;
    pos_loop_cnt = 0;

    if (csa.state != ST_POS_TP) {
        trap_planner_reset(&trap_planner, csa.meas_pos, csa.meas_speed_avg);
        trap_planner2csa_rst(&trap_planner);
    } else {
        csa2trap_planner(&trap_planner);
        trap_planner_update(&trap_planner, csa.meas_pos);
        trap_planner2csa(&trap_planner);
    }

    if (csa.state < ST_POSITION) {
        //pid_i_reset(&pid_pos, csa.meas_speed_avg);
        pid_i_set_target(&pid_pos, csa.meas_pos);
        csa.tgt_pos = csa.meas_pos;
        tgt_speed_bk = csa.meas_speed_avg;
        if (csa.state == ST_STOP) {
            csa.tgt_speed = 0;
            tgt_speed_bk = 0;
        }
    } else {
        pid_i_set_target(&pid_pos, csa.tgt_pos);
        tgt_speed_bk = csa.tgt_speed;
        csa.tgt_speed = lroundf(pid_i_update_p_only(&pid_pos, csa.meas_pos));
        if (csa.state == ST_POS_TP)
            csa.tgt_speed += csa.tp_vel_out;
    }

    if (csa.dbg_raw_en == 4)
        raw_dbg(2);
}


static inline void speed_loop_update(void)
{
    if (++speed_loop_cnt < 5)
        return;
    speed_loop_cnt = 0;
    position_loop_update();
    float sl_speed = (pll.omega / csa.motor_poles) / (2 * M_PIf) * 0x10000;
    float meas_speed = csa.sl_state ? sl_speed : csa.meas_speed;
    float meas_speed_avg = csa.sl_state ? sl_speed : csa.meas_speed_avg;

    if (csa.state < ST_SPEED) {
        pid_f_reset(&pid_speed, csa.tgt_iq);
        pid_f_set_target(&pid_speed, meas_speed_avg);
        csa.tgt_speed = lroundf(meas_speed_avg);
        if (csa.state == ST_STOP) {
            csa.tgt_iq = 0;
            csa.tgt_id = 0;
            csa.tgt_speed = 0;
            tgt_speed_bk = 0;
        }
        tgt_iq_bk = csa.tgt_iq;
    } else {
        if (csa.state == ST_SPEED) {
            float v_step = (float)csa.tp_accel / (CURRENT_LOOP_FREQ / 5.0f);
            float speed = pid_speed.target <= csa.tgt_speed ?
                    min(pid_speed.target + v_step, csa.tgt_speed) : max(pid_speed.target - v_step, csa.tgt_speed);
            pid_f_set_target(&pid_speed, speed);
        } else {
            float speed = tgt_speed_bk + (csa.tgt_speed - tgt_speed_bk) / 5 * (pos_loop_cnt + 1);
            pid_f_set_target(&pid_speed, speed);
        }
        tgt_iq_bk = csa.tgt_iq;
        csa.tgt_iq = lroundf(pid_f_update(&pid_speed, meas_speed_avg, meas_speed));
    }

    if (csa.dbg_raw_en == 2)
        raw_dbg(1);
}


void current_loop_update(void)
{
    int16_t anticog_iq = 0, anticog_vq = 0;
    float voltage_mag = 0;

    gpio_set_val(&dbg_out1, 1);
    gpio_set_val(&s_cs, 1);

    float sin_tmp_angle_elec, cos_tmp_angle_elec; // reduce the amount of calculations

    adc_samp_inject(&adc_samp, csa.state == ST_STOP, csa.motor_wire_swap);
    adc_samp2csa(&adc_samp);

    float i_alpha = csa.meas_i[0];
    float i_beta = (csa.meas_i[0] + csa.meas_i[1] * 2) / 1.7320508f; // √3

    if (csa.state != ST_STOP) {
        csa.smo_i_alpha_real = i_alpha * 3.3f / 16376 / 20.0f / 0.02f;
        csa.smo_i_beta_real = i_beta * 3.3f / 16376 / 20.0f / 0.02f;
        csa2smo(&smo);
        smo_update(&smo);
        smo2csa(&smo);
        pll_update(&pll, smo.e_alpha, smo.e_beta);
        pll2csa(&pll);
    }

    csa.ori_encoder = encoder_read();
    uint16_t cali_encoder = csa.ori_encoder;
    if (csa.enc_linear_en)
        cali_encoder = encoder_linearizer(&enc_lin, csa.ori_encoder);
    encoder_filter(&enc_filter, cali_encoder);
    encoder_filter2csa(&enc_filter);

    speed_loop_update();

    // electrical angle = (meas_encoder · MOTOR_POLES) mod one turn (0x10000), wraps in int16
    csa.meas_elec_angle = (int16_t)(uint16_t)(csa.meas_encoder * csa.motor_poles);

    if (csa.sl_state == 1 || csa.sl_state == 2) {
        sl_angle_update();
    } else if (csa.sl_state == 3) {
        csa.tgt_elec_angle = lroundf(pll.theta / (M_PIf * 2) * 0x10000);
    } else if (csa.state >= ST_CURRENT) {
        csa.tgt_elec_angle = csa.meas_elec_angle;
    }
    sin_tmp_angle_elec = sinf(csa.tgt_elec_angle / 65536.0f * (M_PIf * 2));
    cos_tmp_angle_elec = cosf(csa.tgt_elec_angle / 65536.0f * (M_PIf * 2));

    float meas_iq = -i_alpha * sin_tmp_angle_elec + i_beta * cos_tmp_angle_elec;
    float meas_id = i_alpha * cos_tmp_angle_elec + i_beta * sin_tmp_angle_elec;
    csa.meas_iq = clip(lroundf(meas_iq), -32768, 32767);
    csa.meas_id = clip(lroundf(meas_id), -32768, 32767);

    if (csa.anticog_en)
        anticog_ff(&anticog, csa.meas_encoder, &anticog_iq, &anticog_vq);
    float err_iq = meas_iq - csa.meas_iq_avg_f;
    csa.meas_iq_avg_f += err_iq * 0.001f;
    csa.meas_iq_avg = clip(lroundf(csa.meas_iq_avg_f), -32768, 32767);


    if (csa.state >= ST_CURRENT) {
        float current = tgt_iq_bk + (csa.tgt_iq - tgt_iq_bk) * (speed_loop_cnt + 1) / 5.0f;
        int32_t target_current = lroundf(current);
        pid_f_set_target(&pid_iq, target_current + anticog_iq);
        pid_f_set_target(&pid_id, csa.tgt_id);
        float tgt_vq = pid_f_update(&pid_iq, meas_iq, meas_iq) + anticog_vq * inv_voltage_ratio;
        float tgt_vd = pid_f_update(&pid_id, meas_id, meas_id);
        csa.tgt_vq = clip(lroundf(tgt_vq), -32768, 32767);
        csa.tgt_vd = clip(lroundf(tgt_vd), -32768, 32767);
    } else {
        pid_f_set_target(&pid_iq, meas_iq);
        pid_f_set_target(&pid_id, meas_id);
        pid_f_reset(&pid_iq, csa.tgt_vq);
        pid_f_reset(&pid_id, csa.tgt_vd);
    }

    if (csa.state == ST_STOP) {
        csa.tgt_vq = 0;
        csa.tgt_vd = 0;
        pid_f_set_target(&pid_iq, 0);
        pid_f_set_target(&pid_id, 0);
        pid_f_reset(&pid_iq, 0);
        pid_f_reset(&pid_id, 0);
    }

    float err_vq = csa.tgt_vq - csa.tgt_vq_avg_f;
    csa.tgt_vq_avg_f += err_vq * 0.001f;
    csa.tgt_vq_avg = clip(lroundf(csa.tgt_vq_avg_f), -32768, 32767);

    float v_alpha = csa.tgt_vd * cos_tmp_angle_elec - csa.tgt_vq * sin_tmp_angle_elec;
    float v_beta =  csa.tgt_vd * sin_tmp_angle_elec + csa.tgt_vq * cos_tmp_angle_elec;
    voltage_mag = svpwm(v_alpha, v_beta, csa.pwm_uvw, csa.pwm_dbg0, csa.state == ST_VOLTAGE ? NULL : csa.meas_i);
    if (voltage_mag > SVPWM_MAX_MAG)
        vector_over_limit = voltage_mag;

    csa.smo_v_alpha_real = v_alpha / SVPWM_FULL * (csa.bus_voltage_f / 2);
    csa.smo_v_beta_real = v_beta / SVPWM_FULL * (csa.bus_voltage_f / 2);

    if (csa.state == ST_STOP) {
        smo_init(&smo, true);
        pll_init(&pll, true);
        smo2csa(&smo);
        pll2csa(&pll);
        csa.smo_v_alpha_real = 0;
        csa.smo_v_beta_real = 0;
    }


    adc_samp_set_ch(&adc_samp, csa.pwm_uvw, csa.state != ST_STOP && voltage_mag > SVPWM_MAX_MAG / 2);
    motor_pwm_output(csa.pwm_uvw, csa.motor_wire_swap);

    adc_samp_regular(&adc_samp);
    if (csa.dbg_raw_en <= 1)
        raw_dbg(0);
    csa.loop_cnt++;

    uint16_t enc_check = encoder_read();
    if (enc_check != csa.ori_encoder)
        d_warn("encoder dat late\n");
    encoder_isr_prepare();
    gpio_set_val(&dbg_out1, 0);
}
