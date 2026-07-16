/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "app_main.h"
#include "math.h"

reg2r_t csa_w_allow[] = {
        { .offset = offsetof(csa_t, magic_code), .size = 4 },
        { .offset = offsetof(csa_t, do_reboot),
                .size = offsetof(csa_t, tp_max_err) + sizeof(csa.tp_max_err) - offsetof(csa_t, do_reboot) },
        { .offset = offsetof(csa_t, pid_pos_kp),
                .size = offsetof(csa_t, _reserved_tgt_elec_angle) - offsetof(csa_t, pid_pos_kp) }
};

csa_hook_t csa_w_hook[] = {
        {
            .range = { .offset = offsetof(csa_t, state), .size = 1 },
            .before = state_w_hook_before
        }, {
            .range = { .offset = offsetof(csa_t, tp_pos), .size = 0x14 },
            .after = motor_w_hook_after
        }
};

csa_hook_t csa_r_hook[] = {};

int csa_w_allow_num = sizeof(csa_w_allow) / sizeof(reg2r_t);
int csa_w_hook_num = sizeof(csa_w_hook) / sizeof(csa_hook_t);
int csa_r_hook_num = sizeof(csa_r_hook) / sizeof(csa_hook_t);


const csa_t csa_dft = {
        .magic_code = 0xcdcd,
        .conf_ver = APP_CONF_VER,

        .mac = 0xfe,
        .baud_rate_l = 115200,
        .baud_rate_h = 115200,
        .bus_filter_m = { 0xff, 0xff },
        .bus_mode = 1,
        .bus_idle_wait_len = 0x0a,
        .bus_tx_permit_len = 0x14,
        .bus_max_idle_len = 0xc8,
        .bus_tx_pre_len = 0x01,
        .dbg_en = false,

        .pid_pos_kp = 50,
        .pid_pos_out_min = -65536*100,
        .pid_pos_out_max = 65536*100,
        .pid_speed_kp = 0.16,
        .pid_speed_ki = 16,
        .pid_speed_out_min = -15000,
        .pid_speed_out_max = 15000,
        .pid_iq_kp = 0.1,
        .pid_iq_ki = 50,
        .pid_iq_out_min = -16384,
        .pid_iq_out_max = 16384,
        .pid_id_kp = 0.07,
        .pid_id_ki = 50,
        .pid_id_out_min = -16384,
        .pid_id_out_max = 16384,

        .motor_poles = 7,
        .bias_encoder = 0x1234,

        .qxchg_set = {
                { .offset = offsetof(csa_t, tp_pos), .size = 4 * 3 }
        },
        .qxchg_ret = {
                { .offset = offsetof(csa_t, tgt_pos), .size = 8 }
        },

        .dbg_str_msk = 0x0, //0 or 0xff,

        .dbg_raw_en = 0,
        .dbg_raw = {
                { .offset = offsetof(csa_t, tgt_vq), .size = 2 },
                { .offset = offsetof(csa_t, meas_iq), .size = 4 },
                { .offset = offsetof(csa_t, meas_encoder), .size = 2 }
        },

        .tp_speed = 65536*20,
        .tp_accel = 65536*5,

        .cali_angle_elec = (float)M_PI/2,
        .cali_voltage = SVPWM_FULL / 5,

        .nominal_voltage = 240,
        .tp_max_err = 0x1000,
        .ntc_b = 3970,
        .ntc_r25 = 100000, // 100k
        .temp_warn = 900,
        .temp_err = 1000,
        .voltage_min = 70,
        .voltage_max = 380
};

csa_t csa;


void load_conf(void)
{
    uint16_t magic_code = *(uint16_t *)APP_CONF_ADDR;
    uint16_t conf_ver = *(uint16_t *)(APP_CONF_ADDR + 2);
    csa = csa_dft;

    if (magic_code == 0xcdcd && conf_ver == APP_CONF_VER) {
        memcpy(&csa, (void *)APP_CONF_ADDR, offsetof(csa_t, _end_save));
        csa.conf_from = 1;
    } else if (magic_code == 0xcdcd && (conf_ver >> 12) == (APP_CONF_VER >> 12)) {
        memcpy(&csa, (void *)APP_CONF_ADDR, offsetof(csa_t, _end_common));
        csa.conf_from = 2;
        csa.conf_ver = APP_CONF_VER;
    }
    if (csa.conf_from) {
        memset(&csa.do_reboot, 0, 3);
        csa.dbg_raw_en = 0;
    }
}

int save_conf(void)
{
    uint8_t ret = flash_erase(APP_CONF_ADDR, 2048);
    if (ret != HAL_OK)
        d_info("conf: failed to erase flash\n");
    ret = flash_write(APP_CONF_ADDR, offsetof(csa_t, _end_save), (uint8_t *)&csa);

    if (ret == HAL_OK) {
        d_info("conf: save to flash successed, size: %d\n", offsetof(csa_t, _end_save));
        return 0;
    } else {
        d_error("conf: save to flash error\n");
        return 1;
    }
}


int flash_erase(uint32_t addr, uint32_t len)
{
    int ret = -1;
    uint32_t err_sector = 0xffffffff;
    FLASH_EraseInitTypeDef f;

    uint32_t ofs = addr & ~0x08000000;
    if (ofs <= 0x6000 && 0x6000 < ofs + len) {
        d_error("nvm erase: avoid erasing self\n");
        return ret;
    }

    f.TypeErase = FLASH_TYPEERASE_PAGES;
    f.Banks = FLASH_BANK_1;
    f.Page = ofs / 2048;
    f.NbPages = (ofs + len) / 2048 - f.Page;
    if ((ofs + len) % 2048)
        f.NbPages++;

    ret = HAL_FLASH_Unlock();
    if (ret == HAL_OK)
        ret = HAL_FLASHEx_Erase(&f, &err_sector);
    ret |= HAL_FLASH_Lock();
    d_debug("nvm erase: %08lx +%08lx (%ld %ld), %08lx, ret: %d\n", addr, len, f.Page, f.NbPages, err_sector, ret);
    return ret;
}

int flash_write(uint32_t addr, uint32_t len, const uint8_t *buf)
{
    int ret = -1;

    uint64_t *dst_dat = (uint64_t *) addr;
    int cnt = (len + 7) / 8;
    uint64_t *src_dat = (uint64_t *)buf;

    ret = HAL_FLASH_Unlock();
    for (int i = 0; ret == HAL_OK && i < cnt; i++) {
        uint64_t dat = get_unaligned32((uint8_t *)(src_dat + i));
        dat |= (uint64_t)get_unaligned32((uint8_t *)(src_dat + i) + 4) << 32;
        ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)(dst_dat + i), dat);
    }
    ret |= HAL_FLASH_Lock();

    d_verbose("nvm write: %p %ld(%d), ret: %d\n", dst_dat, len, cnt, ret);
    return ret;
}


#define t_name(expr)  \
        (_Generic((expr), \
                int8_t: "b", uint8_t: "B", \
                int16_t: "h", uint16_t: "H", \
                int32_t: "i", uint32_t: "I", \
                int: "i", \
                bool: "b", \
                float: "f", \
                char *: "[c]", \
                int8_t *: "[b]", \
                uint8_t *: "[B]", \
                int16_t *: "[h]", \
                uint32_t *: "[I]", \
                float *: "[f]", \
                regr_t: "H,B2", \
                regr_t *: "{H,B2}", \
                default: "-"))


#define CSA_SHOW(_p, _x, _desc) \
        d_debug("  [ 0x%04x, %d, \"%s\", " #_p ", \"" #_x "\", \"%s\" ],\n", \
                offsetof(csa_t, _x), sizeof(csa._x), t_name(csa._x), _desc);

#define CSA_SHOW_SUB(_p, _x, _y_t, _y, _desc) \
        d_debug("  [ 0x%04x, %d, \"%s\", " #_p ", \"" #_x "_" #_y "\", \"%s\" ],\n", \
                offsetof(csa_t, _x) + offsetof(_y_t, _y), sizeof(csa._x._y), t_name(csa._x._y), _desc);

void csa_list_show(void)
{
    d_info("csa_list_show:\n\n");
    while (frame_free_head.len < FRAME_MAX - 5);

    CSA_SHOW(1, magic_code, "Magic code: 0xcdcd");
    CSA_SHOW(1, conf_ver, "Config version");
    CSA_SHOW(1, conf_from, "0: default config, 1: all from flash, 2: partly from flash");
    CSA_SHOW(0, do_reboot, "1: reboot to bl, 2: reboot to app");
    CSA_SHOW(0, keep_bl, "Keep running in bootloader");
    CSA_SHOW(0, save_conf, "Write 1 to save current config to flash");
    d_info("\n");

    CSA_SHOW(1, mac, "RS-485 port id, range: 0~254");
    CSA_SHOW(0, baud_rate_l, "RS-485 low baud rate");
    CSA_SHOW(0, baud_rate_h, "RS-485 high baud rate");
    CSA_SHOW(1, bus_filter_m, "Multicast address");
    CSA_SHOW(0, bus_mode, "0: Traditional, 1: Arbitration, 2: Break Sync");
    CSA_SHOW(0, bus_idle_wait_len, "Idle wait time");
    CSA_SHOW(0, bus_tx_permit_len, "Allow send wait time");
    CSA_SHOW(0, bus_max_idle_len, "Max idle wait time for BS mode");
    CSA_SHOW(0, bus_tx_pre_len, "Active TX_EN before TX");
    d_debug("\n");

    CSA_SHOW(0, dbg_en, "1: Report debug message to host, 0: do not report");
    CSA_SHOW(0, dbg_raw_en, "1: current, 2: speed, 4: position loop");
    CSA_SHOW(1, dbg_raw, "Config raw debug data sources");
    CSA_SHOW(1, qxchg_mcast, "Quick-exchange multicast offset and size");
    CSA_SHOW(1, qxchg_set, "Quick-exchange write data components");
    CSA_SHOW(1, qxchg_ret, "Quick-exchange return data components");
    d_info("\n");

    CSA_SHOW(0, nominal_voltage, "Nominal voltage, 0.1 V");
    CSA_SHOW(0, enc_linear_en, "");
    CSA_SHOW(0, enc_linear_max, "");
    CSA_SHOW(0, anticog_en, "");
    CSA_SHOW(0, anticog_max_iq, "");
    CSA_SHOW(0, anticog_ratio_vq, "");
    CSA_SHOW(0, motor_wire_swap, "Software swaps motor wiring");
    CSA_SHOW(0, motor_poles, "Motor poles");
    CSA_SHOW(1, bias_encoder, "Offset for encoder value");
    CSA_SHOW(0, bias_pos, "Offset for position value");
    CSA_SHOW(0, cali_voltage, "Encoder calibration voltage, counts");
    CSA_SHOW(0, ntc_b, "");
    CSA_SHOW(0, ntc_r25, "");
    CSA_SHOW(0, voltage_min, "Undervoltage threshold, 0.1 V");
    CSA_SHOW(0, voltage_max, "Overvoltage threshold, 0.1 V");
    CSA_SHOW(0, temp_err, "Overtemperature threshold, 0.1 C");
    CSA_SHOW(0, temp_warn, "Temperature warning threshold, 0.1 C");

    CSA_SHOW(1, tp_pos, "Set target position");
    CSA_SHOW(1, tp_speed, "Set target speed");
    CSA_SHOW(1, tp_accel, "Set target accel");
    CSA_SHOW(0, tp_max_err, "Limit position error");
    CSA_SHOW(0, tp_state, "Trap planner state");
    CSA_SHOW(0, tp_vel_out, "Current planned velocity");
    d_info("\n");

    while (frame_free_head.len < FRAME_MAX - 5);
    CSA_SHOW(0, pid_pos_kp, "");
    CSA_SHOW(0, pid_pos_out_min, "");
    CSA_SHOW(0, pid_pos_out_max, "");
    CSA_SHOW(0, pid_speed_kp, "");
    CSA_SHOW(0, pid_speed_ki, "");
    CSA_SHOW(0, pid_speed_out_min, "");
    CSA_SHOW(0, pid_speed_out_max, "");
    CSA_SHOW(0, pid_iq_kp, "");
    CSA_SHOW(0, pid_iq_ki, "");
    CSA_SHOW(0, pid_iq_out_min, "");
    CSA_SHOW(0, pid_iq_out_max, "");
    CSA_SHOW(0, pid_id_kp, "");
    CSA_SHOW(0, pid_id_ki, "");
    CSA_SHOW(0, pid_id_out_min, "");
    CSA_SHOW(0, pid_id_out_max, "");

    CSA_SHOW(0, enc_cali, "Write 1 to calibrate encoder");
    CSA_SHOW(0, state, "0: stop, 1: voltage, 2: current, 3: speed, 4: position, 5: trap planner");
    CSA_SHOW(1, error_flag, "");
    CSA_SHOW(1, warn_flag, "");
    CSA_SHOW(1, tgt_pos, "Position target");
    CSA_SHOW(1, tgt_speed, "Speed target");
    CSA_SHOW(0, tgt_iq, "Iq target");
    CSA_SHOW(0, tgt_vq, "Vq target");
    CSA_SHOW(0, tgt_vd, "Vd target");
    CSA_SHOW(1, meas_encoder, "Encoder value");
    CSA_SHOW(0, meas_elec_angle, "Measured electrical angle");
    CSA_SHOW(1, meas_pos, "Measured position");
    CSA_SHOW(1, meas_speed, "Measured speed");
    CSA_SHOW(0, meas_iq, "Measured Iq");
    CSA_SHOW(0, meas_id, "Measured Id");
    CSA_SHOW(0, bus_voltage, "Bus voltage, 0.1 V");
    CSA_SHOW(0, motor_temp, "Motor temperature, 0.1 C");
    CSA_SHOW(1, ori_encoder, "Origin encoder value");
    CSA_SHOW(0, meas_speed_avg, "");
    CSA_SHOW(0, meas_iq_avg, "");
    CSA_SHOW(0, tgt_vq_avg, "");

    CSA_SHOW(0, loop_cnt, "Increase at current loop, for raw debug");
    CSA_SHOW(0, adc_sel, "");
    CSA_SHOW(0, meas_i, "");
    CSA_SHOW(0, pwm_dbg0, "");
    CSA_SHOW(0, pwm_dbg1, "");
    CSA_SHOW(0, pwm_uvw, "");
    CSA_SHOW(1, nob_encoder, "Encoder value before adding bias");
    CSA_SHOW(1, nob_pos, "Position before adding bias");
    CSA_SHOW(0, meas_rpm_avg, "");
    CSA_SHOW(0, meas_iq_avg_f, "");
    CSA_SHOW(0, tgt_vq_avg_f, "");
    CSA_SHOW(0, tp_acc_brake, "Required braking acceleration");
    CSA_SHOW(0, bus_voltage_f, "");
    CSA_SHOW(0, motor_temp_f, "");
    CSA_SHOW(0, cali_angle_speed_tgt, "Calibration mode speed");
    CSA_SHOW(0, cali_angle_speed, "");
    CSA_SHOW(0, cali_angle_elec, "Calibration mode angle");
    CSA_SHOW(0, drv_error_flag, "Detailed gate-driver error flags");
    CSA_SHOW(1, dbg_str_msk, "Config which debug strings are sent");
    d_info("\n");

    while (frame_free_head.len < FRAME_MAX - 5);
    d_info("\x1b[92mColor Test...\x1b[0m\n");
}
