/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#ifndef __APP_MAIN_H__
#define __APP_MAIN_H__

#include "cdnet_dispatch.h"
#include "cd_debug.h"
#include "cdbus_uart.h"
#include "cdctl_it.h"
#include "pid_f.h"
#include "pid_i.h"

// printf float value without enable "-u _printf_float"
// e.g.: printf("%d.%.2d\n", P_2F(2.14));
#define P_2F(x) (int)(x), abs(((x)-(int)(x))*100)  // "%d.%.2d"
#define P_3F(x) (int)(x), abs(((x)-(int)(x))*1000) // "%d.%.3d"


#define APP_CONF_ADDR       0x0800c000 // sector 3
#define APP_CONF_VER        0x0001

#define CURRENT_LOOP_FREQ   (168000000 / 4096 / 2)
#define DRV_PWM_HALF        2048

#define FRAME_MAX           10
#define PACKET_MAX          300


typedef enum {
    ST_STOP = 0,
    ST_CALI,
    ST_CONST_CURRENT,
    ST_CONST_SPEED,
    ST_POSITION,
    ST_POS_TC
} state_t;

typedef enum {
    LED_POWERON = 0,
    LED_WARN,
    LED_ERROR
} led_state_t;

typedef struct {
    uint16_t        offset;
    uint16_t        size;
} regr_t; // reg range


typedef struct {
    uint16_t        magic_code; // 0xcdcd
    uint16_t        conf_ver;
    bool            conf_from;  // 0: default, 1: load from flash
    bool            do_reboot;
    bool            _reserved;
    bool            save_conf;

    //uint8_t       bus_mode; // a, bs, trad
    uint8_t         bus_net;
    uint8_t         bus_mac;
    uint32_t        bus_baud_low;
    uint32_t        bus_baud_high;
    //uint16_t      bus_tx_premit_len;
    //uint16_t      bus_max_idle_len;
    bool            dbg_en;
    cdn_sockaddr_t  dbg_dst;

    pid_i_t         pid_pos;
    pid_f_t         pid_speed;
    pid_f_t         pid_cur;

    float           peak_cur_threshold;
    int32_t         peak_cur_duration;

    uint16_t        bias_encoder;
    int32_t         bias_pos;

    regr_t          qxchg_set[10];
    regr_t          qxchg_ret[10];
    regr_t          qxchg_ro[10];

    uint8_t         dbg_str_msk;
    uint16_t        dbg_str_skip;     // for period string debug

    cdn_sockaddr_t  dbg_raw_dst;
    uint8_t         dbg_raw_msk;
    uint8_t         dbg_raw_th;      // len threshold (+ 1 samples < pkt size)
    uint8_t         dbg_raw_skip[4]; // take samples every few times
    regr_t          dbg_raw[4][10];  // for: cur, speed, pos, tcurve

    int32_t         tc_pos;
    uint32_t        tc_speed;
    uint32_t        tc_accel;

    uint32_t        tc_pos_d;        // delta for middle
    int32_t         tc_pos_m;
    uint32_t        tc_speed_m;
    uint32_t        tc_accel_m;

    float           cali_angle_elec;
    float           cali_current;
    float           cali_angle_step; // increase cali_angle_elec

    // end of eeprom

    state_t         state;
    uint16_t        err_flag;

    int32_t         cal_pos;
    float           cal_speed;
    int32_t         cal_current;
    float           cal_i_sq;

    uint16_t        ori_encoder;
    int32_t         ori_pos;

    float           delta_encoder;
    uint16_t        noc_encoder; // no compensation
    uint16_t        sen_encoder;
    int32_t         sen_pos;
    float           sen_speed;
    float           sen_current;
    float           sen_angle_elec;

    uint32_t        loop_cnt;
    int32_t         peak_cur_cnt;

    // for t_curve
    uint8_t         tc_state; // 0: stop, 1: run, 2: tailer
    float           tc_vc;    // cur speed
    float           tc_ve;    // end speed

} csa_t; // config status area

extern csa_t csa;
extern regr_t regr_wa[]; // writable list
extern int regr_wa_num;

void app_main(void);
void load_conf(void);
int save_conf(void);
void csa_list_show(void);

void common_service_init(void);
void common_service_routine(void);

void set_led_state(led_state_t state);

void app_motor_init(void);
void app_motor_routine(void);
void limit_det_isr(void);
void selection_sort(int32_t arr[], int len, int32_t order[]);

uint16_t encoder_read(void);

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim1;

extern gpio_t led_r;
extern gpio_t led_g;
extern cdn_ns_t dft_ns;
extern list_head_t frame_free_head;

#endif
