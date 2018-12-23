/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#include "app_main.h"

app_conf_t app_conf = {
        .magic_code = 0xcdcd,
        .bl_wait = 30, // wait 3 sec

        .rs485_net = 0,
        .rs485_mac = 2,
        .rs485_baudrate_low = 115200,
        .rs485_baudrate_high = 115200,

        .dbg_en = true,
        .dbg_dst = {
                .addr.cd_addr8 = {0x80, 0x00, 0x00},
                .port = 9
        }
};


void load_conf_early(void)
{
#if 0
    app_conf_t app_tmp;
    memcpy(&app_tmp, (void *)APP_CONF_ADDR, sizeof(app_conf_t));
    if (app_tmp.magic_code == 0xcdcd)
        memcpy(&app_conf, &app_tmp, sizeof(app_conf_t));
#endif
}

void load_conf(void)
{
#if 0
    app_conf_t app_tmp;
    memcpy(&app_tmp, (void *)APP_CONF_ADDR, sizeof(app_conf_t));
    if (app_tmp.magic_code == 0xcdcd) {
        d_info("conf: load from flash\n");
        memcpy(&app_conf, &app_tmp, sizeof(app_conf_t));
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
    uint32_t *src_dat = (uint32_t *)&app_conf;
    uint8_t cnt = (sizeof(app_conf_t) + 3) / 4;
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
