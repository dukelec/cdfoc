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
#include "cdctl_it.h"
#include "pid_f.h"
#include "pid_i.h"

// printf float value without enable "-u _printf_float"
// e.g.: printf("%d.%.2d\n", P_2F(2.14));
#define P_2F(x) (int)(x), abs(((x)-(int)(x))*100)  // "%d.%.2d"
#define P_3F(x) (int)(x), abs(((x)-(int)(x))*1000) // "%d.%.3d"


#define BL_ARGS             0x20000000 // first word
#define CALI_ENCODER_TBL    0x0801b800 // 8k, 2bytes x 4096
#define ANTICOGGING_TBL     0x0801d800 // 8k, 2bytes x 4096
#define APP_CONF_ADDR       0x0801f800 // page 63, the last page
#define APP_CONF_VER        0x0106

#define CURRENT_LOOP_FREQ   (170000000 / 4096 / 2)
#define DRV_PWM_HALF        2048

#define FRAME_MAX           10
#define PACKET_MAX          60


typedef enum {
    ST_STOP = 0,
    ST_CALI,
    ST_CURRENT,
    ST_SPEED,
    ST_POSITION,
    ST_POS_TC
} state_t;

typedef struct {
    uint16_t        offset;
    uint8_t         size;
} regr_t; // reg range

typedef struct {
    uint16_t        offset;
    uint16_t        size;
} reg2r_t; // reg range


typedef struct {
    uint16_t        magic_code;     // 0xcdcd
    uint16_t        conf_ver;
    uint8_t         conf_from;      // 0: default, 1: all from flash, 2: partly from flash
    uint8_t         do_reboot;
    bool            _reserved_bl;   // keep_in_bl for bl
    bool            save_conf;

    uint8_t         bus_net;
    cdctl_cfg_t     bus_cfg;
    bool            dbg_en;
    cdn_sockaddr_t  dbg_dst;
    #define         _end_common pid_pos

    pid_i_t         pid_pos;
    pid_f_t         pid_speed;
    pid_f_t         pid_i_sq;
    pid_f_t         pid_i_sd;

    float           peak_cur_threshold;
    int32_t         peak_cur_duration;

    uint8_t         motor_poles;
    uint16_t        bias_encoder;
    int32_t         bias_pos;

    regr_t          qxchg_mcast;     // for multicast
    regr_t          qxchg_set[5];
    regr_t          qxchg_ret[5];
    regr_t          qxchg_ro[5];

    uint8_t         dbg_str_msk;
    uint16_t        dbg_str_skip;    // for period string debug

    cdn_sockaddr_t  dbg_raw_dst;
    uint8_t         dbg_raw_msk;
    uint8_t         dbg_raw_th;      // len threshold (+ 1 samples < pkt size)
    regr_t          dbg_raw[4][6];  // for: cur, speed, pos, tcurve

    int32_t         tc_pos;
    uint32_t        tc_speed;
    uint32_t        tc_accel;

    bool            tc_rpt_end;
    cdn_sockaddr_t  tc_rpt_dst;

    float           cali_angle_elec;
    float           cali_current;
    float           cali_angle_step; // increase cali_angle_elec
    bool            cali_run;

    bool            cali_encoder_en;
    bool            anticogging_en;
    float           anticogging_max_val[2];

    float           nominal_voltage;
    uint8_t         _reserved[24];

    // end of flash
    #define         _end_save state

    state_t         state;
    uint16_t        err_flag;

    int32_t         cal_pos;
    float           cal_speed;
    int32_t         cal_current;
    float           cal_v_sq;
    float           cal_v_sd;

    uint16_t        ori_encoder;
    int32_t         ori_pos;

    float           delta_encoder;
    uint16_t        nob_encoder; // no bias
    uint16_t        sen_encoder;
    int32_t         sen_pos;
    float           sen_speed;
    float           sen_i_sq;
    float           sen_i_sd;
    float           sen_angle_elec;

    uint32_t        loop_cnt;
    int32_t         peak_cur_cnt;

    // for t_curve
    uint8_t         tc_state; // 0: stop, 1: run, 2: tailer
    float           tc_vc;    // cur speed
    float           tc_ac;    // cur accel

    uint8_t         adc_sel;  // cur adc channel group

    int16_t         dbg_ia;
    int16_t         dbg_ib;
    int16_t         dbg_u;
    int16_t         dbg_v;

    float           sen_i_sq_avg;
    float           cal_v_sq_avg;
    uint8_t         _reserved2[8];
    float           sen_speed_avg;
    float           sen_rpm_avg;
    float           bus_voltage;
    float           temperature;

} csa_t; // config status area


typedef uint8_t (*hook_func_t)(uint16_t sub_offset, uint8_t len, uint8_t *dat);

typedef struct {
    reg2r_t         range;
    hook_func_t     before;
    hook_func_t     after;
} csa_hook_t;


extern csa_t csa;
extern const csa_t csa_dft;

extern reg2r_t csa_w_allow[]; // writable list
extern int csa_w_allow_num;

extern csa_hook_t csa_w_hook[];
extern int csa_w_hook_num;
extern csa_hook_t csa_r_hook[];
extern int csa_r_hook_num;

uint32_t eflash_read_id(spi_t *dev);
uint8_t eflash_read_status(spi_t *dev);
void eflash_read(spi_t *dev, uint32_t addr, int len, uint8_t *buf);
void eflash_write(spi_t *dev, uint32_t addr, int len, const uint8_t *buf);
void eflash_erase_chip(spi_t *dev);
void eflash_cmd(spi_t *dev, uint8_t cmd);

int flash_erase(uint32_t addr, uint32_t len);
int flash_write(uint32_t addr, uint32_t len, const uint8_t *buf);

void app_main(void);
void load_conf(void);
int save_conf(void);
void csa_list_show(void);

void common_service_init(void);
void common_service_routine(void);

uint8_t state_w_hook_before(uint16_t sub_offset, uint8_t len, uint8_t *dat);
uint8_t motor_w_hook_after(uint16_t sub_offset, uint8_t len, uint8_t *dat);
uint16_t drv_read_reg(uint8_t reg);
void drv_write_reg(uint8_t reg, uint16_t val);
void app_motor_init(void);
void app_motor_routine(void);
void adc_isr(void);

uint16_t encoder_read(void);
void encoder_isr_prepare(void);

extern SPI_HandleTypeDef hspi1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim1;

extern gpio_t drv_en;
extern gpio_t led_r;
extern gpio_t led_g;
extern gpio_t s_cs;
extern gpio_t dbg_out1;
//extern gpio_t dbg_out2;
extern gpio_t sen_int;
extern cdn_ns_t dft_ns;
extern list_head_t frame_free_head;

extern uint32_t end; // end of bss


inline void encoder_isr(void)
{
    __HAL_DMA_ENABLE(hspi1.hdmatx);
}

#endif
