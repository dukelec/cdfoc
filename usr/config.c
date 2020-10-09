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

        .bus_net = 0,
        .bus_mac = 2,
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
                .kp = 100, .ki = 5000.0,
                .out_min = -2000,
                .out_max = 2000, // limit output current
                .period = 5.0 / CURRENT_LOOP_FREQ
        },
        .pid_pos = {
                .kp = 0.001, .ki = 0.01, .kd = 0,
                .out_min = -100,
                .out_max = 100, // limit output speed
                .period = 25.0 / CURRENT_LOOP_FREQ
        },


        .bias_encoder = 0x4590,

        .dbg_str_msk = 0x1fff, // 0x01ff,

        .loop_cnt = 0,

        .state = ST_STOP,
        .cali_angle_elec = (float)M_PI/2
};


void load_conf_early(void)
{
#if 0
    csa_t app_tmp;
    memcpy(&app_tmp, (void *)APP_CONF_ADDR, sizeof(csa_t));
    if (app_tmp.magic_code == 0xcdcd)
        memcpy(&csa, &app_tmp, sizeof(csa_t));
#endif
}

void load_conf(void)
{
#if 0
    csa_t app_tmp;
    memcpy(&app_tmp, (void *)APP_CONF_ADDR, sizeof(csa_t));
    if (app_tmp.magic_code == 0xcdcd) {
        d_info("conf: load from flash\n");
        memcpy(&csa, &app_tmp, sizeof(csa_t));
    } else {
        d_info("conf: use default\n");
    }
#endif
}

void save_conf(void)
{
#if 0
    uint8_t ret;
    uint32_t err_page = 0;
    FLASH_EraseInitTypeDef f;

    f.TypeErase = FLASH_TYPEERASE_PAGES;
    f.PageAddress = APP_CONF_ADDR;
    f.NbPages = 1;

    ret = HAL_FLASH_Unlock();
    if (ret == HAL_OK)
        ret = HAL_FLASHEx_Erase(&f, &err_page);

    if (ret != HAL_OK)
        d_info("conf: failed to erase flash\n");

    uint32_t *dst_dat = (uint32_t *)APP_CONF_ADDR;
    uint32_t *src_dat = (uint32_t *)&csa;
    uint8_t cnt = (sizeof(csa_t) + 3) / 4;
    uint8_t i;

    for (i = 0; ret == HAL_OK && i < cnt; i++)
        ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                (uint32_t)(dst_dat + i), *(src_dat + i));

    ret |= HAL_FLASH_Lock();

    if (ret == HAL_OK)
        d_info("conf: save to flash successed\n");
    else
        d_error("conf: save to flash error\n");
#endif
}
