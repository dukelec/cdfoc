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
        { .offset = offsetof(csa_t, magic_code), .size = offsetof(csa_t, pid_pos) - offsetof(csa_t, magic_code) },
        { .offset = offsetof(csa_t, pid_pos), .size = offsetof(pid_i_t, target) },
        { .offset = offsetof(csa_t, pid_speed), .size = offsetof(pid_f_t, target) },
        { .offset = offsetof(csa_t, pid_i_sq), .size = offsetof(pid_f_t, target) },
        { .offset = offsetof(csa_t, pid_i_sd), .size = offsetof(pid_f_t, target) },
        { .offset = offsetof(csa_t, peak_cur_threshold),
                .size = offsetof(csa_t, cal_v_sq) - offsetof(csa_t, peak_cur_threshold) }
};

csa_hook_t csa_w_hook[] = {
        {
            .range = { .offset = offsetof(csa_t, state), .size = 1 },
            .before = state_w_hook_before
        }, {
            .range = { .offset = offsetof(csa_t, tp_pos), .size = offsetof(csa_t, tp_state) - offsetof(csa_t, tp_pos) },
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

        .bus_cfg = CDCTL_CFG_DFT(0xfe),
        .dbg_en = false,

        .pid_pos = { // motor must have enough power to follow the target position
                .kp = 50,
                .out_min = -65536*100,
                .out_max = 65536*100,
                .dt = 25.0f / CURRENT_LOOP_FREQ
        },
        .pid_speed = {
                .kp = 0.02, .ki = 2,
                .out_min = -3000,
                .out_max = 3000, // limit output current
                .dt = 5.0f / CURRENT_LOOP_FREQ
        },
        .pid_i_sq =  {
                .kp = 0.1, .ki = 50,
                .out_min = -2165,
                .out_max = 2165,
                .dt = 1.0f / CURRENT_LOOP_FREQ
        },
        .pid_i_sd =  {
                .kp = 0.07, .ki = 50,
                .out_min = -1600,
                .out_max = 1600,
                .dt = 1.0f / CURRENT_LOOP_FREQ
        },

        .motor_poles = 7,
        .bias_encoder = 0x1234,

        .qxchg_set = {
                { .offset = offsetof(csa_t, tp_pos), .size = 4 * 3 }
        },
        .qxchg_ret = {
                { .offset = offsetof(csa_t, cal_pos), .size = 8 }
        },

        .dbg_str_msk = 0x0, //0 or 0xff,

        .dbg_raw_msk = 0,
        .dbg_raw = {
                { // cur
                        { .offset = offsetof(csa_t, sen_i_sq), .size = 4 * 2 }, // sen_i_sq, sen_i_sd
                        { .offset = offsetof(csa_t, pid_i_sq) + offsetof(pid_f_t, target), .size = 4 * 2 }, // target, i_term
                        { .offset = offsetof(csa_t, pid_i_sd) + offsetof(pid_f_t, i_term), .size = 4 }, // i_term
                        { .offset = offsetof(csa_t, cal_v_sq), .size = 4 * 2 }, // sq_cal, sd_cal
                        { .offset = offsetof(csa_t, sen_encoder), .size = 2 }
                }, { // speed
                        { .offset = offsetof(csa_t, sen_speed), .size = 4 },
                        { .offset = offsetof(csa_t, pid_speed) + offsetof(pid_f_t, target), .size = 4 * 2 }, // target, i_term
                        { .offset = offsetof(csa_t, cal_current), .size = 4 },
                        { .offset = offsetof(csa_t, sen_encoder), .size = 2 },
                        { .offset = offsetof(csa_t, sen_speed_avg), .size = 4 }
                }, { // pos
                        { .offset = offsetof(csa_t, sen_pos), .size = 4 },
                        { .offset = offsetof(csa_t, pid_pos) + offsetof(pid_i_t, target), .size = 4 * 2 }, // target, i_term
                        { .offset = offsetof(csa_t, cal_speed), .size = 4 },
                        { .offset = offsetof(csa_t, tp_vel_out), .size = 4 },
                        { .offset = offsetof(csa_t, sen_speed_avg), .size = 4 }
                }, { // t_curve
                        { .offset = offsetof(csa_t, tp_state), .size = 1 },
                        { .offset = offsetof(csa_t, tp_pos), .size = 4 },
                        { .offset = offsetof(csa_t, cal_pos), .size = 4 },
                        { .offset = offsetof(csa_t, tp_vel_out), .size = 4 },
                        { .offset = offsetof(csa_t, tp_acc_brake), .size = 4 }
                }
        },

        .tp_speed = 65536*20,
        .tp_accel = 65536*5,

        .cali_angle_elec = (float)M_PI/2,
        .cali_current = 200,

        .nominal_voltage = 24.0f,
        .tp_max_err = 0x1000,
        .ntc_b = 3970,
        .ntc_r25 = 100000, // 100k
        .temperature_warn = 90,
        .temperature_err = 100,
        .voltage_min = 7,
        .voltage_max = 38
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
    } else if (magic_code == 0xcdcd && (conf_ver >> 8) == (APP_CONF_VER >> 8)) {
        memcpy(&csa, (void *)APP_CONF_ADDR, offsetof(csa_t, _end_common));
        csa.conf_from = 2;
        csa.conf_ver = APP_CONF_VER;
    }
    if (csa.conf_from)
        memset(&csa.do_reboot, 0, 3);
    csa.pid_pos.dt = csa_dft.pid_pos.dt;
    csa.pid_speed.dt = csa_dft.pid_speed.dt;
    csa.pid_i_sq.dt = csa_dft.pid_i_sq.dt;
    csa.pid_i_sd.dt = csa_dft.pid_i_sd.dt;
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
    CSA_SHOW(0, save_conf, "Write 1 to save current config to flash");
    d_info("\n");

    CSA_SHOW_SUB(1, bus_cfg, cdctl_cfg_t, mac, "RS-485 port id, range: 0~254");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, baud_l, "RS-485 baud rate for first byte");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, baud_h, "RS-485 baud rate for follow bytes");
    CSA_SHOW_SUB(1, bus_cfg, cdctl_cfg_t, filter_m, "Multicast address");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, mode, "0: Traditional, 1: Arbitration, 2: Break Sync");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, tx_permit_len, "Allow send wait time");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, max_idle_len, "Max idle wait time for BS mode");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, tx_pre_len, "Active TX_EN before TX");
    d_debug("\n");

    CSA_SHOW(0, dbg_en, "1: Report debug message to host, 0: do not report");
    d_info("\n");

    CSA_SHOW_SUB(0, pid_pos, pid_i_t, kp, "");
    CSA_SHOW_SUB(0, pid_pos, pid_i_t, out_min, "");
    CSA_SHOW_SUB(0, pid_pos, pid_i_t, out_max, "");
    d_info("\n");

    CSA_SHOW_SUB(0, pid_speed, pid_f_t, kp, "");
    CSA_SHOW_SUB(0, pid_speed, pid_f_t, ki, "");
    CSA_SHOW_SUB(0, pid_speed, pid_f_t, out_min, "");
    CSA_SHOW_SUB(0, pid_speed, pid_f_t, out_max, "");
    d_info("\n");

    CSA_SHOW_SUB(0, pid_i_sq, pid_f_t, kp, "");
    CSA_SHOW_SUB(0, pid_i_sq, pid_f_t, ki, "");
    CSA_SHOW_SUB(0, pid_i_sq, pid_f_t, out_min, "");
    CSA_SHOW_SUB(0, pid_i_sq, pid_f_t, out_max, "");
    d_info("\n");

    CSA_SHOW_SUB(0, pid_i_sd, pid_f_t, kp, "");
    CSA_SHOW_SUB(0, pid_i_sd, pid_f_t, ki, "");
    CSA_SHOW_SUB(0, pid_i_sd, pid_f_t, out_min, "");
    CSA_SHOW_SUB(0, pid_i_sd, pid_f_t, out_max, "");
    d_info("\n");

    CSA_SHOW(0, motor_poles, "Motor poles");
    CSA_SHOW(0, motor_wire_swap, "Software swaps motor wiring");
    CSA_SHOW(1, bias_encoder, "Offset for encoder value");
    CSA_SHOW(0, bias_pos, "Offset for pos value");
    d_info("\n");
    
    CSA_SHOW(1, qxchg_mcast, "Offset and size for quick-exchange multicast");
    CSA_SHOW(1, qxchg_set, "Config the write data components for quick-exchange channel");
    CSA_SHOW(1, qxchg_ret, "Config the return data components for quick-exchange channel");
    d_info("\n");

    CSA_SHOW(1, dbg_str_msk, "Config which debug data to be send");
    d_info("\n");

    CSA_SHOW(1, dbg_raw_msk, "Config which raw debug data to be send");
    CSA_SHOW(1, dbg_raw[0], "Config raw debug for current loop");
    CSA_SHOW(1, dbg_raw[1], "Config raw debug for speed loop");
    CSA_SHOW(1, dbg_raw[2], "Config raw debug for position loop");
    CSA_SHOW(1, dbg_raw[3], "Config raw debug for position plan");
    d_info("\n");

    CSA_SHOW(1, tp_pos, "Set target position");
    CSA_SHOW(1, tp_speed, "Set target speed");
    CSA_SHOW(1, tp_accel, "Set target accel");
    d_info("\n");

    CSA_SHOW(0, cali_angle_elec, "Calibration mode angle");
    CSA_SHOW(0, cali_current, "Calibration mode current");
    CSA_SHOW(0, cali_angle_speed_tgt, "Calibration mode speed");
    CSA_SHOW(0, cali_run, "0: stopped, write 1 start calibration");
    d_info("\n");

    CSA_SHOW(0, encoder_linearizer_en, "");
    CSA_SHOW(0, encoder_linearizer_max, "");
    CSA_SHOW(0, anticog_en, "");
    CSA_SHOW(0, anticog_max_iq, "");
    CSA_SHOW(0, anticog_max_vq, "");
    CSA_SHOW(0, nominal_voltage, "");
    CSA_SHOW(0, tp_max_err, "Limit position error");
    CSA_SHOW(0, ntc_b, "");
    CSA_SHOW(0, ntc_r25, "");
    CSA_SHOW(0, temperature_warn, "");
    CSA_SHOW(0, temperature_err, "");
    CSA_SHOW(0, voltage_min, "");
    CSA_SHOW(0, voltage_max, "");
    d_info("\n");
    while (frame_free_head.len < FRAME_MAX - 5);

    CSA_SHOW(0, state, "0: stop, 1: calibrate, 2: cur loop, 3: speed loop, 4: pos loop, 5: t_curve");
    CSA_SHOW(1, err_flag, "");
    d_info("\n");

    CSA_SHOW(1, cal_pos, "pos loop target");
    CSA_SHOW(1, cal_speed, "speed loop target");
    CSA_SHOW(0, cal_current, "cur loop target");
    CSA_SHOW(0, cal_v_sq, "v_sq info");
    CSA_SHOW(0, cal_v_sd, "v_sd info");
    d_info("\n");

    CSA_SHOW(1, ori_encoder, "Origin encoder value");
    d_info("\n");

    CSA_SHOW(1, nob_encoder, "Encoder value before add bias");
    CSA_SHOW(1, nob_pos, "sen_pos before add offset");
    CSA_SHOW(1, sen_encoder, "Encoder value filtered");
    CSA_SHOW(1, sen_speed, "delta_encoder filtered");
    CSA_SHOW(1, sen_pos, "multiturn + sen_encoder data");
    CSA_SHOW(0, sen_speed_avg, "");
    CSA_SHOW(0, sen_rpm_avg, "");
    CSA_SHOW(0, sen_i_sq, "i_sq from adc");
    CSA_SHOW(0, sen_i_sd, "i_sd from adc");
    CSA_SHOW(0, sen_angle_elec, "Get electric angle from sen_encoder");
    d_info("\n");

    CSA_SHOW(0, loop_cnt, "Increase at current loop, for raw dbg");
    d_info("\n");

    d_debug("   //--------------- Follows are not writable: -------------------\n");
    CSA_SHOW(0, tp_state, "trap_planner: -1: disable, 0: idle, 1: planning");
    CSA_SHOW(0, tp_vel_out, "Current planned velocity");
    CSA_SHOW(0, tp_acc_brake, "Required braking acceleration");
    d_info("\n");

    CSA_SHOW(0, adc_sel, "");
    CSA_SHOW(0, sen_i, "");
    CSA_SHOW(0, pwm_dbg0, "");
    CSA_SHOW(0, pwm_dbg1, "");
    CSA_SHOW(0, pwm_uvw, "");
    d_info("\n");

    CSA_SHOW(0, sen_i_sq_avg, "");
    CSA_SHOW(0, cal_v_sq_avg, "");
    CSA_SHOW(0, bus_voltage, "");
    CSA_SHOW(0, temperature, "");
    CSA_SHOW(0, cali_angle_speed, "");
    d_info("\n");

    while (frame_free_head.len < FRAME_MAX - 5);
    d_info("\x1b[92mColor Test...\x1b[0m\n");
}
