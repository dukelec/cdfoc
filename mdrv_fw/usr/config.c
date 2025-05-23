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
            .range = { .offset = offsetof(csa_t, tc_pos), .size = offsetof(csa_t, tc_state) - offsetof(csa_t, tc_pos) },
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
                .kp = 100, .ki = 4000, .kd = 0.02,
                .out_min = -65536*100,
                .out_max = 65536*100, // limit output speed
                .period = 25.0 / CURRENT_LOOP_FREQ,
                .filter_len = 3
        },
        .pid_speed = {
                .kp = 0.006, .ki = 0.8, .kd = 0.000008,
                .out_min = -3000,
                .out_max = 3000, // limit output current
                .period = 5.0 / CURRENT_LOOP_FREQ,
                .filter_len = 3
        },
        .pid_i_sq =  {
                .kp = 0.12, .ki = 450,
                .out_min = -2000,
                .out_max = 2000,
                .period = 1.0 / CURRENT_LOOP_FREQ
        },
        .pid_i_sd =  {
                .kp = 0.07, .ki = 350,
                .out_min = -1600,
                .out_max = 1600,
                .period = 1.0 / CURRENT_LOOP_FREQ
        },

        .motor_poles = 7,
        .bias_encoder = 0x06ad,

        .qxchg_set = {
                { .offset = offsetof(csa_t, tc_pos), .size = 4 * 3 }
        },
        .qxchg_ret = {
                { .offset = offsetof(csa_t, cal_pos), .size = 8 }
        },

        .dbg_str_msk = 0x0, //0 or 0xff,
        .dbg_str_skip = 0x1fff, // 0x01ff,

        .dbg_raw_msk = 0,
        .dbg_raw_th = 200,
        .dbg_raw = {
                { // cur : target (i_sq), i_term, last_input
                        { .offset = offsetof(csa_t, pid_i_sq) + offsetof(pid_f_t, target), .size = 4 * 3 },
                        // i_term, last_input
                        { .offset = offsetof(csa_t, pid_i_sd) + offsetof(pid_f_t, i_term), .size = 4 * 2 },
                        { .offset = offsetof(csa_t, sen_encoder), .size = 2 },
                        { .offset = offsetof(csa_t, cal_v_sq), .size = 4 * 2 }
                        //{ .offset = offsetof(csa_t, nob_encoder), .size = 2 }
                }, { // speed
                        { .offset = offsetof(csa_t, pid_speed) + offsetof(pid_f_t, target), .size = 4 * 3 },
                        { .offset = offsetof(csa_t, cal_current), .size = 4 },
                        { .offset = offsetof(csa_t, sen_encoder), .size = 2 },
                        { .offset = offsetof(csa_t, delta_encoder), .size = 4 }
                }, { // pos
                        { .offset = offsetof(csa_t, pid_pos) + offsetof(pid_i_t, target), .size = 4 * 3 },
                        { .offset = offsetof(csa_t, cal_speed), .size = 4 },
                }, { // t_curve
                        { .offset = offsetof(csa_t, tc_state), .size = 1 },
                        { .offset = offsetof(csa_t, tc_pos), .size = 4 },
                        { .offset = offsetof(csa_t, cal_pos), .size = 4 },
                        //{ .offset = offsetof(csa_t, sen_pos), .size = 4 },
                        { .offset = offsetof(csa_t, tc_vc), .size = 4 },
                        { .offset = offsetof(csa_t, tc_ac), .size = 4 }//,
                        //{ .offset = offsetof(csa_t, sen_speed), .size = 4 }
                }
        },

        .tc_speed = 65536*20,
        .tc_accel = 65536*5,

        .cali_angle_elec = (float)M_PI/2,
        .cali_current = 200,
        //.cali_angle_step = 0.003179136f // @ ic-MU3 25KHz spi
        //.cali_angle_step = 0

        .nominal_voltage = 12.0f,
        .tc_max_err = 0x1000
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
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, mode, "0: Arbitration, 1: Break Sync");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, tx_permit_len, "Allow send wait time");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, max_idle_len, "Max idle wait time for BS mode");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, tx_pre_len, " Active TX_EN before TX");
    d_debug("\n");

    CSA_SHOW(0, dbg_en, "1: Report debug message to host, 0: do not report");
    d_info("\n");

    CSA_SHOW_SUB(0, pid_pos, pid_i_t, kp, "");
    CSA_SHOW_SUB(0, pid_pos, pid_i_t, ki, "");
    CSA_SHOW_SUB(0, pid_pos, pid_i_t, kd, "");
    CSA_SHOW_SUB(0, pid_pos, pid_i_t, out_min, "");
    CSA_SHOW_SUB(0, pid_pos, pid_i_t, out_max, "");
    d_info("\n");

    CSA_SHOW_SUB(0, pid_speed, pid_f_t, kp, "");
    CSA_SHOW_SUB(0, pid_speed, pid_f_t, ki, "");
    CSA_SHOW_SUB(0, pid_speed, pid_f_t, kd, "");
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
    CSA_SHOW(1, dbg_str_skip, "Reduce debug data");
    d_info("\n");

    CSA_SHOW(1, dbg_raw_msk, "Config which raw debug data to be send");
    CSA_SHOW(0, dbg_raw_th, "Config raw debug data package size");
    CSA_SHOW(1, dbg_raw[0], "Config raw debug for current loop");
    CSA_SHOW(1, dbg_raw[1], "Config raw debug for speed loop");
    CSA_SHOW(1, dbg_raw[2], "Config raw debug for position loop");
    CSA_SHOW(1, dbg_raw[3], "Config raw debug for position plan");
    d_info("\n");

    CSA_SHOW(1, tc_pos, "Set target position");
    CSA_SHOW(1, tc_speed, "Set target speed");
    CSA_SHOW(1, tc_accel, "Set target accel");
    d_info("\n");

    CSA_SHOW(0, cali_angle_elec, "Calibration mode angle");
    CSA_SHOW(0, cali_current, "Calibration mode current");
    CSA_SHOW(0, cali_angle_step, "Calibration mode speed");
    CSA_SHOW(0, cali_run, "0: stopped, write 1 start calibration");
    d_info("\n");

    CSA_SHOW(0, cali_encoder_en, "");
    CSA_SHOW(0, anticogging_en, "");
    CSA_SHOW(0, anticogging_max_val, "");
    CSA_SHOW(0, nominal_voltage, "");
    CSA_SHOW(0, tc_max_err, "Limit position error");
    d_info("\n");
    while (frame_free_head.len < FRAME_MAX - 5);

    CSA_SHOW(0, state, "0: stop, 1: calibrate, 2: cur loop, 3: speed loop, 4: pos loop, 5: t_curve");
    //CSA_SHOW(0, err_flag, "not used");
    d_info("\n");

    CSA_SHOW(1, cal_pos, "pos loop target");
    CSA_SHOW(1, cal_speed, "speed loop target");
    CSA_SHOW(0, cal_current, "cur loop target");
    CSA_SHOW(0, cal_v_sq, "v_sq info");
    CSA_SHOW(0, cal_v_sd, "v_sd info");
    d_info("\n");

    CSA_SHOW(1, ori_encoder, "Origin encoder value");
    CSA_SHOW(1, ori_pos, "sen_pos before add offset");
    d_info("\n");

    CSA_SHOW(1, delta_encoder, "Encoder value delta");
    CSA_SHOW(1, nob_encoder, "Encoder value before add bias");
    CSA_SHOW(1, sen_encoder, "Encoder value filtered");
    CSA_SHOW(1, sen_pos, "multiturn + sen_encoder data");
    CSA_SHOW(1, sen_speed, "delta_encoder filtered");
    CSA_SHOW(0, sen_i_sq, "i_sq from adc");
    CSA_SHOW(0, sen_i_sd, "i_sd from adc");
    CSA_SHOW(0, sen_angle_elec, "Get electric angle from sen_encoder");
    d_info("\n");

    CSA_SHOW(0, loop_cnt, "Increase at current loop, for raw dbg");
    d_info("\n");

    d_debug("   //--------------- Follows are not writable: -------------------\n");
    CSA_SHOW(0, tc_state, "t_curve: 0: stop, 1: run, 2: tailer");
    CSA_SHOW(0, tc_vc, "Motor current speed");
    CSA_SHOW(0, tc_ac, "Motor current accel");
    d_info("\n");

    CSA_SHOW(0, adc_sel, "");
    CSA_SHOW(0, dbg_ia, "");
    CSA_SHOW(0, dbg_ib, "");
    CSA_SHOW(0, dbg_u, "");
    CSA_SHOW(0, dbg_v, "");
    d_info("\n");

    CSA_SHOW(0, sen_i_sq_avg, "");
    CSA_SHOW(0, cal_v_sq_avg, "");
    CSA_SHOW(0, sen_speed_avg, "");
    CSA_SHOW(0, sen_rpm_avg, "");
    CSA_SHOW(0, bus_voltage, "");
    CSA_SHOW(0, temperature, "");
    d_info("\n");

    while (frame_free_head.len < FRAME_MAX - 5);
    d_info("\x1b[92mColor Test...\x1b[0m\n");
}
