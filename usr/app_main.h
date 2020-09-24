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
    ST_CALIBRATION,
    ST_CONST_CURRENT,
    ST_CONST_SPEED,
    ST_POSITION
} state_t;

typedef enum {
    LED_POWERON = 0,
    LED_WARN,
    LED_ERROR
} led_state_t;


typedef struct {
    uint16_t        magic_code; // 0xcdcd
    bool            bl_wait; // run app after timeout (unit 0.1s), 0xff: never

    uint8_t         rs485_net;
    uint8_t         rs485_mac;
    uint32_t        rs485_baudrate_low;
    uint32_t        rs485_baudrate_high;

    bool            dbg_en;
    cd_sockaddr_t   dbg_dst;

    pid_f_t         pid_cur;
    pid_f_t         pid_speed;
    pid_i_t         pid_pos;

    float           peak_cur_threshold;
    int32_t         peak_cur_duration;

    uint16_t        loop_msk;

    // end of eeprom

    uint16_t        loop_cnt;
    uint16_t        encoder_val;
    float           angle_elec_in;
    float           angle_elec_out;
    float           current_in;
    float           current_out;

    state_t         state;

    int32_t         peak_cur_cnt;

} csa_t; // config status area

extern csa_t csa;

void app_main(void);
void load_conf_early(void);
void load_conf(void);
void save_conf(void);
void common_service_init(void);
void common_service_routine(void);
void debug_init(bool *en, cd_sockaddr_t *dst);

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

#endif
