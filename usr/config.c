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

regr_t regr_wa[] = {
        { .offset = offsetof(csa_t, do_reboot), .size = offsetof(csa_t, pid_pos) - offsetof(csa_t, do_reboot) },
        { .offset = offsetof(csa_t, pid_pos), .size = offsetof(pid_i_t, target) },
        { .offset = offsetof(csa_t, pid_speed), .size = offsetof(pid_f_t, target) },
        { .offset = offsetof(csa_t, pid_speed), .size = offsetof(pid_f_t, target) },
        { .offset = offsetof(csa_t, peak_cur_threshold),
                .size = offsetof(csa_t, cal_i_sq) - offsetof(csa_t, peak_cur_threshold) }
};

int regr_wa_num = sizeof(regr_wa) / sizeof(regr_t);


csa_t csa = {
        .magic_code = 0xcdcd,
        .conf_ver = APP_CONF_VER,

        .bus_net = 0,
        .bus_mac = 254,
        .bus_baud_low = 1000000,
        .bus_baud_high = 2000000,
        .dbg_en = true,
        .dbg_dst = { .addr = {0x80, 0x00, 0x00}, .port = 9 },

        .pid_cur =  {
                .kp = 1, .ki = 400,
                .out_min = DRV_PWM_HALF * -0.9,
                .out_max = DRV_PWM_HALF * 0.9,
                .period = 1.0 / CURRENT_LOOP_FREQ
        },
        .pid_speed = {
                .kp = 0.02, .ki = 5,
                .out_min = -3000,
                .out_max = 3000, // limit output current
                .period = 5.0 / CURRENT_LOOP_FREQ
        },
        .pid_pos = {
                .kp = 45, .ki = 50, .kd = 0.05,
                .out_min = -5000000,
                .out_max = 5000000, // limit output speed
                .period = 25.0 / CURRENT_LOOP_FREQ
        },

        .bias_encoder = 0x1425,

        .qxchg_set = {
                { .offset = offsetof(csa_t, tc_pos), .size = 4 }
        },
        .qxchg_ret = {
                { .offset = offsetof(csa_t, cal_pos), .size = 8 }
        },

        .dbg_str_msk = 0xff,
        .dbg_str_skip = 0x1fff, // 0x01ff,

        .dbg_raw_dst = { .addr = {0x80, 0x00, 0x00}, .port = 8 },
        .dbg_raw_msk = 0,
        .dbg_raw_th = 200,
        .dbg_raw_skip = { 0, 0, 0, 0 },
        .dbg_raw = {
                { // cur : target, i_term, last_input, cal_i_sq,
                        { .offset = offsetof(csa_t, pid_cur) + offsetof(pid_f_t, target), .size = 4 * 3 },
                        { .offset = offsetof(csa_t, cal_i_sq), .size = 4 },
                        { .offset = offsetof(csa_t, sen_encoder), .size = 2 }
                }, { // speed
                        { .offset = offsetof(csa_t, pid_speed) + offsetof(pid_f_t, target), .size = 4 * 3 },
                        { .offset = offsetof(csa_t, delta_encoder), .size = 2 }
                }, { // pos
                        { .offset = offsetof(csa_t, pid_pos) + offsetof(pid_i_t, target), .size = 4 * 3 },
                        { .offset = offsetof(csa_t, cal_speed), .size = 4 },
                }, { // t_curve
                        { .offset = offsetof(csa_t, tc_state), .size = 1 },
                        { .offset = offsetof(csa_t, tc_pos), .size = 4 },
                        { .offset = offsetof(csa_t, cal_pos), .size = 4 },
                        //{ .offset = offsetof(csa_t, sen_pos), .size = 4 },
                        { .offset = offsetof(csa_t, tc_vc), .size = 4 },
                        { .offset = offsetof(csa_t, tc_ve), .size = 4 }//,
                        //{ .offset = offsetof(csa_t, sen_speed), .size = 4 }
                }
        },

        .tc_speed = 200000,
        .tc_accel = 10000,
        .tc_pos_d = 10000,
        .tc_speed_m = 50000,
        .tc_accel_m = 5000,

        .cali_angle_elec = (float)M_PI/2,
        .cali_current = 200,
        //.cali_angle_step = 0.003f // @ ic-MU3 10KHz spi
        //.cali_angle_step = 0
};


void load_conf(void)
{
    csa_t app_tmp;
    memcpy(&app_tmp, (void *)APP_CONF_ADDR, offsetof(csa_t, state));
    memset(&app_tmp.conf_from, 0, offsetof(csa_t, bus_net));

    if (app_tmp.magic_code == 0xcdcd && app_tmp.conf_ver == APP_CONF_VER) {
        memcpy(&csa, &app_tmp, offsetof(csa_t, state));
        csa.conf_from = 1;
    }
}

int save_conf(void)
{
    uint8_t ret;
    uint32_t err_page = 0;
    FLASH_EraseInitTypeDef f;
    f.TypeErase = FLASH_TYPEERASE_SECTORS;
    f.Sector = 3; // sector 3
    f.NbSectors = 1;
    f.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    ret = HAL_FLASH_Unlock();
    if (ret == HAL_OK)
        ret = HAL_FLASHEx_Erase(&f, &err_page);

    if (ret != HAL_OK)
        d_info("conf: failed to erase flash\n");

    uint32_t *dst_dat = (uint32_t *)APP_CONF_ADDR;
    uint32_t *src_dat = (uint32_t *)&csa;
    int cnt = (offsetof(csa_t, conf_from) + 3) / 4;

    for (int i = 0; ret == HAL_OK && i < cnt; i++)
        ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(dst_dat + i), *(src_dat + i));
    ret |= HAL_FLASH_Lock();

    if (ret == HAL_OK) {
        d_info("conf: save to flash successed\n");
        return 0;
    } else {
        d_error("conf: save to flash error\n");
        return 1;
    }
}


#define CSA_SHOW(_x) \
        d_info("   R_" #_x " = 0x%04x # len: %d\n", offsetof(csa_t, _x), sizeof(csa._x));

#define CSA_SHOW_SUB(_x, _y_t, _y) \
        d_info("   R_" #_x "_" #_y " = 0x%04x # len: %d\n", offsetof(csa_t, _x) + offsetof(_y_t, _y), sizeof(csa._x._y));

void csa_list_show(void)
{
    d_info("csa_list_show:\n\n");

    CSA_SHOW(conf_ver);
    CSA_SHOW(conf_from);
    CSA_SHOW(do_reboot);
    d_info("\n");

    CSA_SHOW(bus_net);
    CSA_SHOW(bus_mac);
    CSA_SHOW(bus_baud_low);
    CSA_SHOW(bus_baud_high);
    CSA_SHOW(dbg_en);
    CSA_SHOW_SUB(dbg_dst, cdn_sockaddr_t, addr);
    CSA_SHOW_SUB(dbg_dst, cdn_sockaddr_t, port);
    d_info("\n");

    CSA_SHOW_SUB(pid_pos, pid_i_t, kp);
    CSA_SHOW_SUB(pid_pos, pid_i_t, ki);
    CSA_SHOW_SUB(pid_pos, pid_i_t, kd);
    CSA_SHOW_SUB(pid_pos, pid_i_t, out_min);
    CSA_SHOW_SUB(pid_pos, pid_i_t, out_max);
    CSA_SHOW_SUB(pid_pos, pid_i_t, period);
    d_info("\n");

    CSA_SHOW_SUB(pid_speed, pid_f_t, kp);
    CSA_SHOW_SUB(pid_speed, pid_f_t, ki);
    CSA_SHOW_SUB(pid_speed, pid_f_t, kd);
    CSA_SHOW_SUB(pid_speed, pid_f_t, out_min);
    CSA_SHOW_SUB(pid_speed, pid_f_t, out_max);
    CSA_SHOW_SUB(pid_speed, pid_f_t, period);
    d_info("\n");

    CSA_SHOW_SUB(pid_cur, pid_f_t, kp);
    CSA_SHOW_SUB(pid_cur, pid_f_t, ki);
    CSA_SHOW_SUB(pid_cur, pid_f_t, kd);
    CSA_SHOW_SUB(pid_cur, pid_f_t, out_min);
    CSA_SHOW_SUB(pid_cur, pid_f_t, out_max);
    CSA_SHOW_SUB(pid_cur, pid_f_t, period);
    d_info("\n");

    CSA_SHOW(bias_encoder);
    CSA_SHOW(bias_pos);
    CSA_SHOW(qxchg_set);
    CSA_SHOW(qxchg_ret);
    CSA_SHOW(qxchg_ro);
    d_info("\n");

    CSA_SHOW(dbg_str_msk);
    CSA_SHOW(dbg_str_skip);
    d_info("\n");

    CSA_SHOW_SUB(dbg_raw_dst, cdn_sockaddr_t, addr);
    CSA_SHOW_SUB(dbg_raw_dst, cdn_sockaddr_t, port);
    CSA_SHOW(dbg_raw_msk);
    CSA_SHOW(dbg_raw_th);
    CSA_SHOW(dbg_raw_skip);
    CSA_SHOW(dbg_raw);
    d_info("\n");

    CSA_SHOW(tc_pos);
    CSA_SHOW(tc_speed);
    CSA_SHOW(tc_accel);
    d_info("\n");

    CSA_SHOW(tc_pos_d);
    CSA_SHOW(tc_pos_m);
    CSA_SHOW(tc_speed_m);
    CSA_SHOW(tc_accel_m);
    d_info("\n");

    CSA_SHOW(cali_angle_elec);
    CSA_SHOW(cali_current);
    CSA_SHOW(cali_angle_step);
    d_info("\n");

    CSA_SHOW(state);
    CSA_SHOW(err_flag);
    d_info("\n");

    CSA_SHOW(cal_pos);
    CSA_SHOW(cal_speed);
    CSA_SHOW(cal_current);
    CSA_SHOW(cal_i_sq);
    d_info("\n");

    CSA_SHOW(ori_encoder);
    CSA_SHOW(ori_pos);
    d_info("\n");

    CSA_SHOW(delta_encoder);
    CSA_SHOW(noc_encoder);
    CSA_SHOW(sen_encoder);
    CSA_SHOW(sen_pos);
    CSA_SHOW(sen_speed);
    CSA_SHOW(sen_current);
    CSA_SHOW(sen_angle_elec);
    d_info("\n");

    CSA_SHOW(loop_cnt);
    d_info("\n");

    CSA_SHOW(tc_state);
    CSA_SHOW(tc_vc);
    CSA_SHOW(tc_ve);
    d_info("\n");
}
