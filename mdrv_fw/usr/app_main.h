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

#include <math.h>
#include "cdnet_core.h"
#include "cd_debug.h"
#include "cdbus_uart.h"
#include "cdctl_it.h"
#include "pid_f.h"
#include "pid_i.h"

// printf float value without enable "-u _printf_float"
// e.g.: printf("%d.%.2d\n", P_2F(2.14));
#define P_2F(x) (int)(x), abs((int)(((x)-(int)(x))*100))  // "%d.%.2d"
#define P_3F(x) (int)(x), abs((int)(((x)-(int)(x))*1000)) // "%d.%.3d"
#define M_PIf   ((float)M_PI)


#define BL_ARGS             0x20000000 // first word
#define ENC_LIN_TBL         0x0801d800 // 4k
#define ANTICOG_TBL         0x0801e800 // 4k
#define APP_CONF_ADDR       0x0801f800 // page 63, the last page
#define APP_CONF_VER        0x0300

#define CURRENT_LOOP_FREQ   (170000000.f / (4096 * 2))
#define DRV_PWM_HALF        2048

#define FRAME_MAX           60
#define PACKET_MAX          60
#define CDN_MAX_PAYLOAD     251


typedef enum {
    ST_STOP = 0,
    ST_VOLTAGE,
    ST_CURRENT,
    ST_SPEED,
    ST_POSITION,
    ST_POS_TP
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
    uint8_t         do_reboot;      // 1: reboot to bl, 2: reboot to app
    bool            keep_bl;
    bool            save_conf;
    uint8_t         _reserved0[7];

    uint8_t         mac;
    uint32_t        baud_rate_l;
    uint32_t        baud_rate_h;
    uint8_t         bus_filter_m[2];
    uint8_t         _reserved01[2];
    uint8_t         bus_mode;
    uint8_t         bus_idle_wait_len;
    uint16_t        bus_tx_permit_len;
    uint16_t        bus_max_idle_len;
    uint8_t         bus_tx_pre_len;
    uint8_t         _reserved1[13];

    bool            dbg_en;
    #define         _end_common dbg_raw_en
    uint8_t         dbg_raw_en;
    regr_t          dbg_raw[6];
    uint8_t         _reserved2[22];

    regr_t          qxchg_mcast;     // for multicast
    regr_t          qxchg_set[4];
    regr_t          qxchg_ret[5];
    uint8_t         _reserved3[24];

    uint16_t        nominal_voltage; // unit 0.1 V
    uint8_t         _reserved40[14];
    uint8_t         _reserved41[32]; // hall configuration

    uint8_t         enc_linear_en;
    int16_t         enc_linear_max;
    uint8_t         _reserved51[12];

    uint8_t         anticog_en;
    int16_t         anticog_max_iq;
    int16_t         anticog_ratio_vq;
    uint8_t         _reserved53[8];

    uint8_t         motor_wire_swap;
    uint8_t         motor_poles;

    uint8_t         _reserved6a[2]; // min_gap
    uint16_t        bias_encoder;
    int32_t         bias_pos;
    uint8_t         _reserved6b[8]; // pos limits
    int32_t         _bias_pos2;
    uint16_t        cali_voltage;
    uint16_t        ntc_b;
    uint32_t        ntc_r25; // NTC resistor @ 25 C
    uint8_t         _reserved6c[20];

    uint8_t         _reserved7a[6]; // over-current protection
    uint16_t        voltage_min;    // unit 0.1 V
    uint16_t        voltage_max;    // unit 0.1 V
    uint8_t         _reserved7b[6]; // voltage timeout and stall protection
    uint16_t        temp_err;       // unit 0.1 C
    uint16_t        temp_warn;      // unit 0.1 C
    uint8_t         _reserved7c[28];

    int32_t         tp_pos;
    uint32_t        tp_speed;
    uint32_t        tp_accel;
    uint8_t         _reserved8a[4]; // tp_accel_e
    uint32_t        tp_max_err;
    uint8_t         _reserved8b[27];
    int8_t          tp_state;
    int32_t         tp_vel_out;
    uint8_t         _reserved9[44];

    float           pid_pos_kp;
    uint8_t         _reserved10[8];
    int32_t         pid_pos_out_min;
    int32_t         pid_pos_out_max;
    uint8_t         _reserved11[28];

    float           pid_speed_kp;
    float           pid_speed_ki;
    uint8_t         _reserved12[4];
    int16_t         pid_speed_out_min;
    int16_t         pid_speed_out_max;
    uint8_t         _reserved13[32];

    float           pid_iq_kp;
    float           pid_iq_ki;
    uint8_t         _reserved14[4];
    int16_t         pid_iq_out_min;
    int16_t         pid_iq_out_max;
    uint8_t         _reserved140[32];

    float           pid_id_kp;
    float           pid_id_ki;
    uint8_t         _reserved141[4];
    int16_t         pid_id_out_min;
    int16_t         pid_id_out_max;


    // end of flash
    #define         _end_save _reserved15
    uint8_t         _reserved15[430];

    uint8_t         enc_cali;
    uint8_t         _reserved150;

    uint8_t         state;
    union {
        uint8_t error_flag;
        struct {
            uint8_t fast_oc       : 1;
            uint8_t slow_oc       : 1;
            uint8_t bus_uv        : 1;
            uint8_t bus_ov        : 1;
            uint8_t motor_stall   : 1;
            uint8_t               : 1;
            uint8_t drv_fault     : 1;
            uint8_t motor_ot      : 1;
        } error_flag_;
    };
    uint8_t         _reserved16;
    union {
        uint8_t warn_flag;
        struct {
            uint8_t               : 7;
            uint8_t motor_ot      : 1;
        } warn_flag_;
    };
    uint8_t         _reserved17[60];

    int32_t         tgt_pos;
    int32_t         tgt_speed;
    int16_t         tgt_iq;
    int16_t         _reserved_tgt_id;
    int16_t         tgt_vq;
    int16_t         tgt_vd;
    int16_t         _reserved_tgt_elec_angle;
    uint8_t         _reserved18[46];

    uint8_t         _reserved180[16]; // hall values

    uint16_t        meas_encoder;
    int16_t         meas_elec_angle;
    int32_t         meas_pos;
    int32_t         meas_speed;
    int16_t         meas_iq;
    int16_t         meas_id;
    int16_t         _reserved_bus_current;
    int16_t         bus_voltage; // unit 0.1 V
    int16_t         motor_temp;  // unit 0.1 C
    int16_t         _reserved_board_temp[2];
    uint8_t         _reserved_measurement2[6];
    int32_t         _meas_pos2;
    uint16_t        ori_encoder;
    uint16_t        _ori_encoder2;
    uint8_t         _reserved181[16];

    int32_t         meas_speed_avg;
    int16_t         meas_iq_avg;
    int16_t         tgt_vq_avg;
    uint8_t         _reserved182[16];

    // internal/project-specific data below is outside the protocol table.
    uint32_t        loop_cnt;
    uint8_t         adc_sel;     // cur adc channel group
    int16_t         meas_i[3];
    int16_t         pwm_dbg0[3]; // before deadtime compensate
    int16_t         pwm_dbg1[3]; // after deadtime compensate
    int16_t         pwm_uvw[3];
    uint8_t         _reserved183[16];

    uint16_t        nob_encoder; // no bias
    int32_t         nob_pos;
    float           meas_rpm_avg;
    float           meas_iq_avg_f;
    float           tgt_vq_avg_f;

    float           tp_acc_brake;

    float           bus_voltage_f;
    float           motor_temp_f;
    float           cali_angle_speed_tgt; // target speed [rad/s]
    float           cali_angle_speed;
    float           cali_angle_elec;

    uint16_t        drv_error_flag;
    uint8_t         dbg_str_msk; // bit0: dump_hw_status

} csa_t; // config status area

_Static_assert(offsetof(csa_t, mac) == 0x000f, "CSA mac offset");
_Static_assert(offsetof(csa_t, baud_rate_l) == 0x0010, "CSA baud_rate offset");
_Static_assert(offsetof(csa_t, dbg_en) == 0x0030, "CSA dbg_en offset");
_Static_assert(offsetof(csa_t, dbg_raw) == 0x0032, "CSA dbg_raw offset");
_Static_assert(offsetof(csa_t, qxchg_mcast) == 0x0060, "CSA qxchg offset");
_Static_assert(offsetof(csa_t, qxchg_set) == 0x0064, "CSA qxchg_set offset");
_Static_assert(offsetof(csa_t, qxchg_ret) == 0x0074, "CSA qxchg_ret offset");
_Static_assert(offsetof(csa_t, nominal_voltage) == 0x00a0, "CSA nominal_voltage offset");
_Static_assert(offsetof(csa_t, enc_linear_en) == 0x00d0, "CSA enc_linear offset");
_Static_assert(offsetof(csa_t, enc_linear_max) == 0x00d2, "CSA enc_linear_max offset");
_Static_assert(offsetof(csa_t, anticog_en) == 0x00e0, "CSA anticog offset");
_Static_assert(offsetof(csa_t, anticog_max_iq) == 0x00e2, "CSA anticog_max_iq offset");
_Static_assert(offsetof(csa_t, motor_wire_swap) == 0x00ee, "CSA motor_wire_swap offset");
_Static_assert(offsetof(csa_t, bias_encoder) == 0x00f2, "CSA bias_encoder offset");
_Static_assert(offsetof(csa_t, bias_pos) == 0x00f4, "CSA bias_pos offset");
_Static_assert(offsetof(csa_t, _bias_pos2) == 0x0100, "CSA bias_pos2 offset");
_Static_assert(offsetof(csa_t, cali_voltage) == 0x0104, "CSA cali_voltage offset");
_Static_assert(offsetof(csa_t, ntc_b) == 0x0106, "CSA ntc_b offset");
_Static_assert(offsetof(csa_t, ntc_r25) == 0x0108, "CSA ntc_r25 offset");
_Static_assert(offsetof(csa_t, voltage_min) == 0x0126, "CSA voltage_min offset");
_Static_assert(offsetof(csa_t, voltage_max) == 0x0128, "CSA voltage_max offset");
_Static_assert(offsetof(csa_t, temp_err) == 0x0130, "CSA temp_err offset");
_Static_assert(offsetof(csa_t, temp_warn) == 0x0132, "CSA temp_warn offset");
_Static_assert(offsetof(csa_t, tp_pos) == 0x0150, "CSA tp_pos offset");
_Static_assert(offsetof(csa_t, tp_accel) == 0x0158, "CSA tp_accel offset");
_Static_assert(offsetof(csa_t, tp_max_err) == 0x0160, "CSA tp_max_err offset");
_Static_assert(offsetof(csa_t, tp_state) == 0x017f, "CSA tp_state offset");
_Static_assert(offsetof(csa_t, tp_vel_out) == 0x0180, "CSA tp_vel_out offset");
_Static_assert(offsetof(csa_t, pid_pos_kp) == 0x01b0, "CSA pid_pos offset");
_Static_assert(offsetof(csa_t, pid_speed_kp) == 0x01e0, "CSA pid_speed offset");
_Static_assert(offsetof(csa_t, pid_iq_kp) == 0x0210, "CSA pid_iq offset");
_Static_assert(offsetof(csa_t, pid_id_kp) == 0x0240, "CSA pid_id offset");
_Static_assert(offsetof(csa_t, _end_save) == 0x0250, "CSA saved config end");
_Static_assert(offsetof(csa_t, enc_cali) == 0x03fe, "CSA enc_cali offset");
_Static_assert(offsetof(csa_t, state) == 0x0400, "CSA state offset");
_Static_assert(offsetof(csa_t, error_flag) == 0x0401, "CSA error_flag offset");
_Static_assert(offsetof(csa_t, warn_flag) == 0x0403, "CSA warn_flag offset");
_Static_assert(offsetof(csa_t, tgt_pos) == 0x0440, "CSA target offset");
_Static_assert(offsetof(csa_t, tgt_speed) == 0x0444, "CSA tgt_speed offset");
_Static_assert(offsetof(csa_t, tgt_iq) == 0x0448, "CSA tgt_iq offset");
_Static_assert(offsetof(csa_t, tgt_vq) == 0x044c, "CSA tgt_vq offset");
_Static_assert(offsetof(csa_t, meas_encoder) == 0x0490, "CSA measurement offset");
_Static_assert(offsetof(csa_t, meas_elec_angle) == 0x0492, "CSA meas_elec_angle offset");
_Static_assert(offsetof(csa_t, meas_pos) == 0x0494, "CSA meas_pos offset");
_Static_assert(offsetof(csa_t, meas_speed) == 0x0498, "CSA meas_speed offset");
_Static_assert(offsetof(csa_t, meas_iq) == 0x049c, "CSA meas_iq offset");
_Static_assert(offsetof(csa_t, meas_id) == 0x049e, "CSA meas_id offset");
_Static_assert(offsetof(csa_t, bus_voltage) == 0x04a2, "CSA bus_voltage offset");
_Static_assert(offsetof(csa_t, motor_temp) == 0x04a4, "CSA motor_temp offset");
_Static_assert(offsetof(csa_t, _reserved_board_temp) == 0x04a6, "CSA board_temp offset");
_Static_assert(offsetof(csa_t, _meas_pos2) == 0x04b0, "CSA meas_pos2 offset");
_Static_assert(offsetof(csa_t, ori_encoder) == 0x04b4, "CSA ori_encoder offset");
_Static_assert(offsetof(csa_t, _ori_encoder2) == 0x04b6, "CSA ori_encoder2 offset");
_Static_assert(offsetof(csa_t, meas_speed_avg) == 0x04c8, "CSA meas_speed_avg offset");
_Static_assert(offsetof(csa_t, meas_iq_avg) == 0x04cc, "CSA meas_iq_avg offset");
_Static_assert(offsetof(csa_t, tgt_vq_avg) == 0x04ce, "CSA tgt_vq_avg offset");
_Static_assert(offsetof(csa_t, loop_cnt) == 0x04e0, "CSA internal data offset");


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

void comm_service_init(void);
void comm_service_poll(void);

uint8_t state_w_hook_before(uint16_t sub_offset, uint8_t len, uint8_t *dat);
uint8_t motor_w_hook_after(uint16_t sub_offset, uint8_t len, uint8_t *dat);
uint16_t drv_read_reg(uint8_t reg);
void drv_write_reg(uint8_t reg, uint16_t val);
void app_motor_init(void);
void app_motor_maintain(void);
void current_loop_update(void);

uint16_t encoder_read(void);
void encoder_isr_prepare(void);

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern SPI_HandleTypeDef hspi1;
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
extern cdctl_dev_t r_dev;

extern uint32_t end; // end of bss

#include "adc_samp.h"
#include "encoder_linearizer.h"
#include "encoder_filter.h"
#include "anticog.h"
#include "svpwm.h"
#include "trap_planner.h"
#include "csa_sync.h"
#include "misc.h"
#endif
