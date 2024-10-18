/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __APP_MAIN_H__
#define __APP_MAIN_H__

#include "cdnet_core.h"
#include "cd_debug.h"
#include "cdbus_uart.h"
#include "cdctl.h"

// printf float value without enable "-u _printf_float"
// e.g.: printf("%d.%.2d\n", P_2F(2.14));
#define P_2F(x) (int)(x), abs(((x)-(int)(x))*100)  // "%d.%.2d"
#define P_3F(x) (int)(x), abs(((x)-(int)(x))*1000) // "%d.%.3d"


#define APP_CONF_ADDR       0x0801f800 // page 63, the last page
#define APP_CONF_VER        0x0100

#define FRAME_MAX           10
#define PACKET_MAX          30


typedef struct {
    uint16_t        offset;
    uint16_t        size;
} regr_t; // reg range


typedef struct {
    uint16_t        magic_code; // 0xcdcd
    uint16_t        conf_ver;
    uint8_t         conf_from;  // 0: default, 1: load from flash
    bool            do_reboot;
    bool            keep_in_bl;
    bool            save_conf;

    uint8_t         bus_net;
    cdctl_cfg_t     bus_cfg;
    bool            dbg_en;
    cdn_sockaddr_t  dbg_dst;

    uint8_t         _keep[1024]; // covers the areas in the app csa that need to be saved

    // end of flash
    uint8_t         _end_save;

} csa_t; // config status area

extern csa_t csa;
extern const csa_t csa_dft;

int flash_erase(uint32_t addr, uint32_t len);
int flash_write(uint32_t addr, uint32_t len, const uint8_t *buf);

void app_main(void);
void load_conf(void);
int save_conf(void);

void common_service_init(void);
void common_service_routine(void);

extern gpio_t led_r;
extern gpio_t led_g;
extern cdn_ns_t dft_ns;

#endif
