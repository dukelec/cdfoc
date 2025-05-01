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

static cdn_sock_t sock_tc_rpt = { .port = 0x10, .ns = &dft_ns, .tx_only = true };
static cdn_sock_t sock_raw_dbg = { .port = 0xa, .ns = &dft_ns, .tx_only = true }; // raw debug
static list_head_t raw_pend = { 0 };


void misc_init(void)
{
    cdn_sock_bind(&sock_raw_dbg);
}

void dbg_routine(void)
{
    if (frame_free_head.len > 1) {
        cdn_pkt_t *pkt = cdn_list_get(&raw_pend);
        if (pkt)
            cdn_sock_sendto(&sock_raw_dbg, pkt);
    }

    static uint8_t tc_state_old = 0;
    if (csa.tc_rpt_end) {
        if (csa.tc_state == 0 && tc_state_old != 0) {
            cdn_pkt_t *pkt = cdn_pkt_alloc(sock_tc_rpt.ns);
            if (pkt) {
                pkt->dst = csa.tc_rpt_dst;
                cdn_pkt_prepare(&sock_tc_rpt, pkt);
                pkt->dat[0] = 0x40;
                pkt->len = 1;
                cdn_sock_sendto(&sock_tc_rpt, pkt);
            }
        }
    }
    tc_state_old = csa.tc_state;
}


void raw_dbg(int idx)
{
    static cdn_pkt_t *pkt_raw[4] = { NULL };
    static bool pkt_less = false;

    if (!(csa.dbg_raw_msk & (1 << idx))) {
        if (pkt_raw[idx]) {
            cdn_pkt_free(&dft_ns, pkt_raw[idx]);
            pkt_raw[idx] = NULL;
        }
        return;
    }

    if (pkt_less) {
        if (raw_pend.len == 0 && dft_ns.free_frm->len >= FRAME_MAX - 5) {
            pkt_less = false;
        }
    }

    if (!pkt_less && !pkt_raw[idx]) {
        if (dft_ns.free_pkt->len < 5 || dft_ns.free_frm->len < 5) {
            pkt_less = true;
            return;

        } else {
            pkt_raw[idx] = cdn_pkt_alloc(sock_raw_dbg.ns);
            pkt_raw[idx]->dst = csa.dbg_raw_dst;
            cdn_pkt_prepare(&sock_raw_dbg, pkt_raw[idx]);
            pkt_raw[idx]->dat[0] = 0x40 | idx;
            put_unaligned32(csa.loop_cnt, pkt_raw[idx]->dat + 1);
            pkt_raw[idx]->len = 5;
        }
    }
    if (!pkt_raw[idx])
        return;

    for (int i = 0; i < 6; i++) {
        regr_t *regr = &csa.dbg_raw[idx][i];
        if (!regr->size)
            break;
        uint8_t *dst_dat = pkt_raw[idx]->dat + pkt_raw[idx]->len;
        memcpy(dst_dat, ((void *) &csa) + regr->offset, regr->size);
        pkt_raw[idx]->len += regr->size;
    }

    if (pkt_raw[idx]->len >= csa.dbg_raw_th) {
        cdn_list_put(&raw_pend, pkt_raw[idx]);
        pkt_raw[idx] = NULL;
    }
}


void cali_elec_angle(void)
{
    static int pole_cnt = -1;
    static int sub_cnt;
    static uint32_t t_last;
    static uint16_t a0, a1;
    static int amount_f, amount_r;
    static int dir = 1; // -1 or +1
    static uint16_t m_first;

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
        m_first = -1;
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
        d_debug_c(" - a0: %04x\n", a0);
    } else if (sub_cnt == 2) {
        a1 = csa.nob_encoder;
        d_debug_c(" - a1: %04x\n", a1);
    } else {
        d_debug_c("\n");
    }

    if ((dir == 1 && sub_cnt == 3) || (dir == -1 && sub_cnt == 0)) {
        int16_t delta_a = a1 - a0;
        uint16_t n = a0 + delta_a / 2;
        uint16_t m = n - (pole_cnt * 0x10000 / csa.motor_poles);
        if (m_first < 0)
            m_first = m;
        d_info("cali: n: %04x, m: %04x\n", n, m);
        d_info("cali: ----------------\n");
        int16_t delta_m = m - m_first;
        if (dir == 1)
            amount_f += delta_m;
        else
            amount_r += delta_m;

        if (dir == -1 && pole_cnt == 0) {
            uint16_t avg_cw = m_first + amount_f / csa.motor_poles;
            uint16_t avg_ccw = m_first + amount_r / csa.motor_poles;
            d_info("cali: finished, result:\n");
            d_info("cali:  cw: %02x\n", avg_cw);
            d_info("cali: ccw: %02x\n", avg_ccw);
            csa.bias_encoder = m_first + (amount_f + amount_r) / csa.motor_poles / 2;
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
