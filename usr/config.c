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
        { .offset = offsetof(csa_t, bus_net), .size = offsetof(csa_t, pid_pos) - offsetof(csa_t, bus_net) },
        { .offset = offsetof(csa_t, pid_pos), .size = offsetof(pid_i_t, target) },
        { .offset = offsetof(csa_t, pid_speed), .size = offsetof(pid_f_t, target) },
        { .offset = offsetof(csa_t, pid_speed), .size = offsetof(pid_f_t, target) },
        { .offset = offsetof(csa_t, peak_cur_threshold),
                .size = offsetof(csa_t, ori_encoder) - offsetof(csa_t, peak_cur_threshold) }
};

int regr_wa_num = sizeof(regr_wa) / sizeof(regr_t);


csa_t csa = {
        .magic_code = 0xcdcd,
        .conf_ver = APP_CONF_VER,

        .bus_net = 0,
        .bus_mac = 254,
        .bus_baud_low = 115200,
        .bus_baud_high = 115200,

        .dbg_en = true,
        .dbg_dst = {
                .addr = {0x80, 0x00, 0x00},
                .port = 9
        },

        .pid_cur =  {
                .kp = 0, .ki = 200,
                .out_min = DRV_PWM_HALF * -0.9,
                .out_max = DRV_PWM_HALF * 0.9,
                .period = 1.0 / CURRENT_LOOP_FREQ
        },
        .pid_speed = {
                .kp = 0.1, .ki = 5.0,
                .out_min = -2000,
                .out_max = 2000, // limit output current
                .period = 5.0 / CURRENT_LOOP_FREQ
        },
        .pid_pos = {
                .kp = 50, .ki = 100, .kd = 0,
                .out_min = -2000000,
                .out_max = 2000000, // limit output speed
                .period = 25.0 / CURRENT_LOOP_FREQ
        },

        .bias_encoder = 0x4590,

        .dio_set = {
                { .offset = offsetof(csa_t, tc_pos), .size = 4 }
        },
        .dio_ret = {
                { .offset = offsetof(csa_t, cal_pos), .size = 8 }
        },

        .dbg_str_msk = 0x1fff, // 0x01ff,

        .tc_speed = 500000,
        .tc_accel = 2000000,

        .cali_angle_elec = (float)M_PI/2,
        .cali_current = 200,
        .cali_angle_step = 0.002f
};


void load_conf(void)
{
    csa_t app_tmp;
    memcpy(&app_tmp, (void *)APP_CONF_ADDR, sizeof(csa_t));
    if (app_tmp.magic_code == 0xcdcd && app_tmp.conf_ver == APP_CONF_VER) {
        memcpy(&csa, &app_tmp, offsetof(csa_t, conf_from));
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
        d_info("  " #_x " : 0x%04x, len: %d\n", offsetof(csa_t, _x), sizeof(csa._x));

#define CSA_SHOW_SUB(_x, _y_t, _y) \
        d_info("  " #_x "." #_y " : 0x%04x, len: %d\n", offsetof(csa_t, _x) + offsetof(_y_t, _y), sizeof(csa._x._y));

void csa_list_show(void)
{
    d_info("csa_list_show:\n\n");

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
    CSA_SHOW(dio_set);
    CSA_SHOW(dio_ret);
    CSA_SHOW(dbg_str_msk);
    d_info("\n");

    CSA_SHOW(tc_pos);
    CSA_SHOW(tc_speed);
    CSA_SHOW(tc_accel);
    d_info("\n");

    CSA_SHOW(cali_angle_elec);
    CSA_SHOW(cali_current);
    CSA_SHOW(cali_angle_step);
    d_info("\n");

    CSA_SHOW(conf_from);
    CSA_SHOW(state);
    CSA_SHOW(err_flag);
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

    CSA_SHOW(cal_pos);
    CSA_SHOW(cal_speed);
    CSA_SHOW(cal_current);
    d_info("\n");

    CSA_SHOW(loop_cnt);
    d_info("\n");

    CSA_SHOW(tc_run);
    CSA_SHOW(tc_s_cur);
    CSA_SHOW(tc_v_cur);
    CSA_SHOW(tc_cnt);
    CSA_SHOW(tc_steps);
    d_info("\n");
}
