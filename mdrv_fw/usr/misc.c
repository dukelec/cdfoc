/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "math.h"
#include "app_main.h"


void raw_dbg(int idx)
{
    static cd_frame_t *frm_raw[4] = { NULL };
    static bool frm_less = false;

    if (!(csa.dbg_raw_msk & (1 << idx))) {
        if (frm_raw[idx]) {
            cd_list_put(&frame_free_head, frm_raw[idx]);
            frm_raw[idx] = NULL;
        }
        return;
    }

    if (frm_less && frame_free_head.len >= FRAME_MAX - 5)
        frm_less = false;

    if (!frm_less && !frm_raw[idx]) {
        if (frame_free_head.len < 5) {
            frm_less = true;
            return;

        } else {
            frm_raw[idx] = cd_list_get(&frame_free_head);
            frm_raw[idx]->dat[0] = csa.bus_cfg.mac;
            frm_raw[idx]->dat[1] = 0x0;
            frm_raw[idx]->dat[2] = 6;
            frm_raw[idx]->dat[3] = 0x40 | idx;
            frm_raw[idx]->dat[4] = 0xa;
            put_unaligned32(csa.loop_cnt, frm_raw[idx]->dat + 5);
        }
    }
    if (!frm_raw[idx])
        return;

    for (int i = 0; i < 6; i++) {
        regr_t *regr = &csa.dbg_raw[idx][i];
        if (!regr->size)
            break;
        uint8_t *dst_dat = frm_raw[idx]->dat + frm_raw[idx]->dat[2] + 3;
        memcpy(dst_dat, ((void *) &csa) + regr->offset, regr->size);
        frm_raw[idx]->dat[2] += regr->size;
    }

    if (frm_raw[idx]->dat[2] >= csa.dbg_raw_th) {
        cdctl_put_tx_frame(&r_dev.cd_dev, frm_raw[idx]);
        frm_raw[idx] = NULL;
    }
}


void cali_elec_angle(void)
{
    static int pole_cnt = -1;
    static int sub_cnt;
    static uint32_t t_last;
    static uint16_t a0, a180;
    static int amount_f, amount_r;
    static int dir = 1; // -1 or +1
    static int32_t a_first;

    if (!csa.cali_run)
        return;

    if (pole_cnt == -1) {
        csa.cali_encoder_en = false;
        csa.anticogging_en = false;
        csa.cali_angle_elec = 0;
        csa.cali_angle_step = 0;
        csa.state = ST_CALI;
        t_last = get_systick();
        pole_cnt = sub_cnt = 0;
        amount_f = amount_r = 0;
        dir = 1;
        a_first = -1;
        d_info("cali: init...\n");
        d_info("cali: ----------------\n");
    }

    uint32_t t_cur = get_systick();
    if (t_cur - t_last < 1000000 / CD_SYSTICK_US_DIV)
        return;
    t_last = t_cur;

    d_debug("cali [%d, %d]", pole_cnt, sub_cnt);

    if (sub_cnt == 0) {
        a0 = csa.nob_encoder;
        d_debug_c(" - a0:   %04x\n", a0);
    } else if (sub_cnt == 2) {
        a180 = csa.nob_encoder;
        d_debug_c(" - a180: %04x\n", a180);
    } else {
        d_debug_c("\n");
    }

    if ((dir == 1 && sub_cnt == 3) || (dir == -1 && sub_cnt == 0)) {
        uint16_t a_shift = a0 - (pole_cnt * 0x10000 / csa.motor_poles);
        if (a_first < 0)
            a_first = a_shift;
        d_info("cali: a0: %04x, a_shift: %04x\n", a0, a_shift);
        d_info("cali: ----------------\n");
        int16_t delta_a = a_shift - a_first;
        if (dir == 1)
            amount_f += delta_a;
        else
            amount_r += delta_a;

        if (dir == -1 && pole_cnt == 0) {
            uint16_t avg_cw = a_first + amount_f / csa.motor_poles;
            uint16_t avg_ccw = a_first + amount_r / csa.motor_poles;
            d_info("cali: finished, result:\n");
            d_info("cali:  cw: %02x\n", avg_cw);
            d_info("cali: ccw: %02x\n", avg_ccw);
            csa.bias_encoder = a_first + (amount_f + amount_r) / csa.motor_poles / 2;
            d_info("cali: avg: %02x, updated to bias_encoder\n", csa.bias_encoder);
            uint8_t dat = ST_STOP;
            state_w_hook_before(0, 1, &dat);
            csa.state = ST_STOP;
            csa.cali_run = false;
            pole_cnt = -1;
        }
    }

    if (csa.cali_run) {
        if (dir == 1) {
            sub_cnt++;
            if (sub_cnt > 3) {
                sub_cnt = 0;
                pole_cnt++;
                if (pole_cnt == csa.motor_poles) {
                    pole_cnt = csa.motor_poles - 1;
                    dir = -1;
                    sub_cnt = 3;
                    d_info("cali: ================\n");
                }
            }
        } else {
            sub_cnt--;
            if (sub_cnt < 0) {
                sub_cnt = 3;
                pole_cnt--;
            }
        }
    }

    csa.cali_angle_elec = (float)M_PI/2 * sub_cnt;
}
