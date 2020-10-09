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


#define APP_CONF_ADDR       0x0801F800 // last page

#define CURRENT_LOOP_FREQ   (168000000 / 4096 / 2)
#define DRV_PWM_HALF        2048


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
    uint8_t         conf_ver;

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

    regr_t          d_set[10];
    regr_t          d_ret[10];
    //uint8_t       dbg_raw_max[3]; // data group per pkt
    //regr_t        dbg_raw[3][10]; // for periods corresponding to 3 loops
    uint16_t        dbg_str_msk;    // for period string debug

    int32_t         tc_pos;
    float           tc_speed;
    float           tc_accel;

    int32_t         cal_pos;
    float           cal_speed;
    float           cal_current;

    float           cali_angle_elec;
    float           cali_current;
    float           cali_angle_step; // increase cali_angle_elec

    // end of eeprom

    state_t         state;
    uint16_t        err_flag;

    uint16_t        ori_encoder;
    int32_t         ori_pos;

    uint16_t        sen_encoder;
    int32_t         sen_pos;
    float           sen_speed;
    float           sen_angle_elec;
    float           sen_current;

    uint32_t        loop_cnt;
    int32_t         peak_cur_cnt;

    // for t_curve
    bool            tc_run;
    float           tc_s_cur;
    float           tc_v_cur;
    int             tc_cnt;
    int             tc_steps;
    int32_t         tc_s_s;
    float           tc_v_s;
    float           tc_s_seg[3], tc_t_seg[3], tc_a_seg[3];

} csa_t; // config status area

extern csa_t csa;
extern regr_t regr_wa[]; // writable list
extern int regr_wa_num;

void app_main(void);
void load_conf_early(void);
void load_conf(void);
void save_conf(void);
void common_service_init(void);
void common_service_routine(void);

void set_led_state(led_state_t state);

void app_motor(void);
void app_motor_init(void);
void limit_det_isr(void);

uint16_t encoder_read(void);

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim1;

extern gpio_t led_r;
extern gpio_t led_g;
extern cdn_ns_t dft_ns;

#endif
