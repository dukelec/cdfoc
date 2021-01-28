/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#include "app_main.h"
#include "math.h"

regr_t csa_w_allow[] = {
        { .offset = offsetof(csa_t, magic_code), .size = offsetof(csa_t, pid_pos) - offsetof(csa_t, magic_code) },
        { .offset = offsetof(csa_t, pid_pos), .size = offsetof(pid_i_t, target) },
        { .offset = offsetof(csa_t, pid_speed), .size = offsetof(pid_f_t, target) },
        { .offset = offsetof(csa_t, pid_i_sq), .size = offsetof(pid_f_t, target) },
        { .offset = offsetof(csa_t, pid_i_sd), .size = offsetof(pid_f_t, target) },
        { .offset = offsetof(csa_t, peak_cur_threshold),
                .size = offsetof(csa_t, cal_i_sq) - offsetof(csa_t, peak_cur_threshold) }
};

csa_hook_t csa_w_hook[] = {
        {
            .range = { .offset = offsetof(csa_t, tc_pos), .size = offsetof(csa_t, tc_state) - offsetof(csa_t, tc_pos) },
            .after = motor_w_hook
        }
};

csa_hook_t csa_r_hook[] = {};

int csa_w_allow_num = sizeof(csa_w_allow) / sizeof(regr_t);
int csa_w_hook_num = sizeof(csa_w_hook) / sizeof(csa_hook_t);
int csa_r_hook_num = sizeof(csa_r_hook) / sizeof(csa_hook_t);


const csa_t csa_dft = {
        .magic_code = 0xcdcd,
        .conf_ver = APP_CONF_VER,

        .bus_net = 0,
        .bus_mac = 254,
        .bus_baud_low = 1000000,
        .bus_baud_high = 1000000,
        .dbg_en = true,
        .dbg_dst = { .addr = {0x80, 0x00, 0x00}, .port = 9 },

        .pid_pos = {
                .kp = 15, .ki = 200, .kd = 0.02,
                .out_min = -65536*100,
                .out_max = 65536*100, // limit output speed
                .period = 25.0 / CURRENT_LOOP_FREQ
        },
        .pid_speed = {
                .kp = 0.002, .ki = 1.2,
                .out_min = -3000,
                .out_max = 3000, // limit output current
                .period = 5.0 / CURRENT_LOOP_FREQ
        },
        .pid_i_sq =  {
                .kp = 1, .ki = 600,
                .out_min = DRV_PWM_HALF * -0.9,
                .out_max = DRV_PWM_HALF * 0.9,
                .period = 1.0 / CURRENT_LOOP_FREQ
        },
        .pid_i_sd =  {
                .kp = 0.5, .ki = 400,
                .out_min = DRV_PWM_HALF * -0.4, // TODO: limit q + d
                .out_max = DRV_PWM_HALF * 0.4,
                .period = 1.0 / CURRENT_LOOP_FREQ
        },

        .motor_poles = 7,
        .bias_encoder = 0xa890,

        .qxchg_set = {
                { .offset = offsetof(csa_t, tc_pos), .size = 4 * 3 }
        },
        .qxchg_ret = {
                { .offset = offsetof(csa_t, cal_pos), .size = 8 }
        },

        .dbg_str_msk = 0x0, //0 or 0xff,
        .dbg_str_skip = 0x1fff, // 0x01ff,

        .dbg_raw_dst = { .addr = {0x80, 0x00, 0x00}, .port = 0xa },
        .dbg_raw_msk = 0,
        .dbg_raw_th = 200,
        .dbg_raw_skip = { 0, 0, 0, 0 },
        .dbg_raw = {
                { // cur : target (cal_current), i_term, last_input
                        { .offset = offsetof(csa_t, pid_i_sq) + offsetof(pid_f_t, target), .size = 4 * 3 },
                        { .offset = offsetof(csa_t, sen_encoder), .size = 2 },
                        { .offset = offsetof(csa_t, noc_encoder), .size = 2 }
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

        .tc_speed = 65536*20,//*1
        .tc_accel = 65536*5,//*20,

        .tc_rpt_end = false,
        .tc_rpt_dst = { .addr = {0x80, 0x00, 0x00}, .port = 0x10 },

        .cali_angle_elec = (float)M_PI/2,
        .cali_current = 200,
        //.cali_angle_step = 0.003179136f // @ ic-MU3 25KHz spi
        //.cali_angle_step = 0
};

csa_t csa;


void load_conf(void)
{
    csa_t app_tmp;
    memcpy(&app_tmp, (void *)APP_CONF_ADDR, offsetof(csa_t, state));
    memset(&app_tmp.conf_from, 0, 4);

    if (app_tmp.magic_code == 0xcdcd && app_tmp.conf_ver == APP_CONF_VER) {
        memcpy(&csa, &app_tmp, offsetof(csa_t, state));
        csa.conf_from = 1;
    } else {
        csa = csa_dft;
    }
}

int save_conf(void)
{
    uint8_t ret;
    uint32_t err_page = 0;
    FLASH_EraseInitTypeDef f;
    f.TypeErase = FLASH_TYPEERASE_PAGES;
    f.Banks = FLASH_BANK_1;
    f.Page = 63; // last page
    f.NbPages = 1;

    ret = HAL_FLASH_Unlock();
    if (ret == HAL_OK)
        ret = HAL_FLASHEx_Erase(&f, &err_page);

    if (ret != HAL_OK)
        d_info("conf: failed to erase flash\n");

    uint64_t *dst_dat = (uint64_t *)APP_CONF_ADDR;
    uint64_t *src_dat = (uint64_t *)&csa;
    int cnt = (offsetof(csa_t, state) + 7) / 8;

    for (int i = 0; ret == HAL_OK && i < cnt; i++)
        ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)(dst_dat + i), *(src_dat + i));
    ret |= HAL_FLASH_Lock();

    if (ret == HAL_OK) {
        d_info("conf: save to flash successed\n");
        return 0;
    } else {
        d_error("conf: save to flash error\n");
        return 1;
    }
}


#define t_name(expr)  \
        (_Generic((expr), \
                int8_t: "int8_t", uint8_t: "uint8_t", \
                int16_t: "int16_t", uint16_t: "uint16_t", \
                int32_t: "int32_t", uint32_t: "uint32_t", \
                int: "int32_t", \
                bool: "bool", \
                float: "float", \
                uint8_t *: "uint8_t *", \
                regr_t *: "regr_t *", \
                void *: "void *", \
                default: "--"))


#define CSA_SHOW(_x, _desc) \
        d_debug("   R_" #_x " = 0x%04x # len: %d, %s | %s\n", \
                offsetof(csa_t, _x), sizeof(csa._x), t_name(csa._x), _desc);

#define CSA_SHOW_SUB(_x, _y_t, _y, _desc) \
        d_debug("   R_" #_x "_" #_y " = 0x%04x # len: %d, %s | %s\n", \
                offsetof(csa_t, _x) + offsetof(_y_t, _y), sizeof(csa._x._y), t_name(csa._x._y), _desc);

void csa_list_show(void)
{
    d_info("csa_list_show:\n\n"); debug_flush(true);

    CSA_SHOW(conf_ver, "Magic Code: 0xcdcd");
    CSA_SHOW(conf_from, "0: default config, 1: load from flash");
    CSA_SHOW(do_reboot, "Write 1 to reboot");
    CSA_SHOW(save_conf, "Write 1 to save current config to flash");
    d_info("\n"); debug_flush(true);

    CSA_SHOW(bus_mac, "RS-485 port id, range: 0~254");
    CSA_SHOW(bus_baud_low, "RS-485 baud rate for first byte");
    CSA_SHOW(bus_baud_high, "RS-485 baud rate for follow bytes");
    CSA_SHOW(dbg_en, "1: Report debug message to host, 0: do not report");
    CSA_SHOW_SUB(dbg_dst, cdn_sockaddr_t, addr, "Send debug message to this address");
    CSA_SHOW_SUB(dbg_dst, cdn_sockaddr_t, port, "Send debug message to this port");
    d_info("\n"); debug_flush(true);

    CSA_SHOW_SUB(pid_pos, pid_i_t, kp, "");
    CSA_SHOW_SUB(pid_pos, pid_i_t, ki, "");
    CSA_SHOW_SUB(pid_pos, pid_i_t, kd, "");
    CSA_SHOW_SUB(pid_pos, pid_i_t, out_min, "");
    CSA_SHOW_SUB(pid_pos, pid_i_t, out_max, "");
    CSA_SHOW_SUB(pid_pos, pid_i_t, period, "");
    d_info("\n"); debug_flush(true);

    CSA_SHOW_SUB(pid_speed, pid_f_t, kp, "");
    CSA_SHOW_SUB(pid_speed, pid_f_t, ki, "");
    CSA_SHOW_SUB(pid_speed, pid_f_t, kd, "");
    CSA_SHOW_SUB(pid_speed, pid_f_t, out_min, "");
    CSA_SHOW_SUB(pid_speed, pid_f_t, out_max, "");
    CSA_SHOW_SUB(pid_speed, pid_f_t, period, "");
    d_info("\n"); debug_flush(true);

    CSA_SHOW_SUB(pid_i_sq, pid_f_t, kp, "");
    CSA_SHOW_SUB(pid_i_sq, pid_f_t, ki, "");
    CSA_SHOW_SUB(pid_i_sq, pid_f_t, kd, "");
    CSA_SHOW_SUB(pid_i_sq, pid_f_t, out_min, "");
    CSA_SHOW_SUB(pid_i_sq, pid_f_t, out_max, "");
    CSA_SHOW_SUB(pid_i_sq, pid_f_t, period, "");
    d_info("\n"); debug_flush(true);

    CSA_SHOW_SUB(pid_i_sd, pid_f_t, kp, "");
    CSA_SHOW_SUB(pid_i_sd, pid_f_t, ki, "");
    CSA_SHOW_SUB(pid_i_sd, pid_f_t, kd, "");
    CSA_SHOW_SUB(pid_i_sd, pid_f_t, out_min, "");
    CSA_SHOW_SUB(pid_i_sd, pid_f_t, out_max, "");
    CSA_SHOW_SUB(pid_i_sd, pid_f_t, period, "");
    d_info("\n"); debug_flush(true);

    CSA_SHOW(bias_encoder, "Offset for encoder value");
    CSA_SHOW(bias_pos, "Offset for pos value");
    CSA_SHOW(qxchg_set, "Config the write data components for quick-exchange channel");
    CSA_SHOW(qxchg_ret, "Config the return data components for quick-exchange channel");
    CSA_SHOW(qxchg_ro, "Config the return data components for the read only quick-exchange channel");
    d_info("\n"); debug_flush(true);

    CSA_SHOW(dbg_str_msk, "Config which debug data to be send");
    CSA_SHOW(dbg_str_skip, "Reduce debug data");
    d_info("\n"); debug_flush(true);

    CSA_SHOW_SUB(dbg_raw_dst, cdn_sockaddr_t, addr, "Send raw debug data to this address");
    CSA_SHOW_SUB(dbg_raw_dst, cdn_sockaddr_t, port, "Send raw debug data to this port");
    CSA_SHOW(dbg_raw_msk, "Config which raw debug data to be send");
    CSA_SHOW(dbg_raw_th, "Config raw debug data package size");
    CSA_SHOW(dbg_raw_skip, "Reduce raw debug data");
    CSA_SHOW(dbg_raw[0], "Config raw debug for current loop");
    CSA_SHOW(dbg_raw[1], "Config raw debug for speed loop");
    CSA_SHOW(dbg_raw[2], "Config raw debug for position loop");
    CSA_SHOW(dbg_raw[3], "Config raw debug for position plan");
    d_info("\n"); debug_flush(true);

    CSA_SHOW(tc_pos, "Set target position");
    CSA_SHOW(tc_speed, "Set target speed");
    CSA_SHOW(tc_accel, "Set target accel");
    CSA_SHOW(tc_rpt_end, "Report finish event");
    CSA_SHOW_SUB(tc_rpt_dst, cdn_sockaddr_t, addr, "Send report to this address");
    CSA_SHOW_SUB(tc_rpt_dst, cdn_sockaddr_t, port, "Send report to this port");
    d_info("\n"); debug_flush(true);

    CSA_SHOW(cali_angle_elec, "Calibration mode angle");
    CSA_SHOW(cali_current, "Calibration mode current");
    CSA_SHOW(cali_angle_step, "Calibration mode speed");
    d_info("\n"); debug_flush(true);

    CSA_SHOW(state, "0: stop, 1: cur loop, 2: speed loop, 3: pos loop, 4: t_curve");
    //CSA_SHOW(err_flag, "not used");
    d_info("\n"); debug_flush(true);

    CSA_SHOW(cal_pos, "pos loop target");
    CSA_SHOW(cal_speed, "speed loop target");
    CSA_SHOW(cal_current, "cur loop target");
    CSA_SHOW(cal_i_sq, "i_sq info");
    CSA_SHOW(cal_i_sd, "i_sd info");
    d_info("\n"); debug_flush(true);

    CSA_SHOW(ori_encoder, "noc_encoder before add offset");
    CSA_SHOW(ori_pos, "sen_pos before add offset");
    d_info("\n"); debug_flush(true);

    CSA_SHOW(delta_encoder, "encoder value delta");
    CSA_SHOW(noc_encoder, "encoder value");
    CSA_SHOW(sen_encoder, "encoder value filtered");
    CSA_SHOW(sen_pos, "multiturn + sen_encoder data");
    CSA_SHOW(sen_speed, "delta_encoder filtered");
    CSA_SHOW(sen_i_sq, "i_sq from adc");
    CSA_SHOW(sen_i_sd, "i_sd from adc");
    CSA_SHOW(sen_angle_elec, "Get electric angle from sen_encoder");
    d_info("\n"); debug_flush(true);

    CSA_SHOW(loop_cnt, "Increase at current loop, for raw dbg");
    d_info("\n"); debug_flush(true);

    d_debug("   #--------------- Follows are not writable: -------------------\n");
    CSA_SHOW(tc_state, "t_curve: 0: stop, 1: run, 2: tailer");
    CSA_SHOW(tc_vc, "Motor current speed");
    CSA_SHOW(tc_ac, "Motor current accel");
    d_info("\n"); debug_flush(true);
}
