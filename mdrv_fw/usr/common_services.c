/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "app_main.h"
#include <math.h>

static char cpu_id[25];
static char info_str[100];

static cdn_sock_t sock1 = { .port = 1, .ns = &dft_ns };
static cdn_sock_t sock5 = { .port = 5, .ns = &dft_ns };
static cdn_sock_t sock6 = { .port = 6, .ns = &dft_ns };
static cdn_sock_t sock8 = { .port = 8, .ns = &dft_ns };


static void get_uid(char *buf)
{
    const char tlb[] = "0123456789abcdef";
    int i;

    for (i = 0; i < 12; i++) {
        uint8_t val = *((char *)UID_BASE + i);
        buf[i * 2 + 0] = tlb[val >> 4];
        buf[i * 2 + 1] = tlb[val & 0xf];
    }
    buf[24] = '\0';
}

static void init_info_str(void)
{
    // M: model; S: serial string; HW: hardware version; SW: software version
    get_uid(cpu_id);
    sprintf(info_str, "M: mdrv; S: %s; SW: %s", cpu_id, SW_VER);
    d_info("info: %s\n", info_str);
}


// device info
static void p1_service_routine(void)
{
    cdn_pkt_t *pkt = cdn_sock_recvfrom(&sock1);
    if (!pkt)
        return;

    if (pkt->len == 1 && pkt->dat[0] == 0) {
        pkt->dat[0] = 0x80;
        strcpy((char *)pkt->dat + 1, info_str);
        pkt->len = strlen(info_str) + 1;
        pkt->dst = pkt->src;
        cdn_sock_sendto(&sock1, pkt);
        csa.conf_from &= 0x7f;
        return;
    }

    // mac address conflicts can be resolved by following:
    if (pkt->len >= 5 && pkt->dat[0] == 0x10) {
        uint16_t max_time = *(uint16_t *)(pkt->dat + 1);
        uint16_t wait_time = rand() / (RAND_MAX / max_time);
        uint8_t mac_start = pkt->dat[3];
        uint8_t mac_end = pkt->dat[4];
        uint8_t local_mac = csa.bus_cfg.mac;
        pkt->dat[pkt->len] = '\0';
        char *string = (char *)pkt->dat + 5;
        d_debug("p1 search: wait %d (%d), [%02x, %02x] (%02x), str: %s\n",
                wait_time, max_time, mac_start, mac_end, local_mac, string);

        if (clip(local_mac, mac_start, mac_end) == local_mac && strstr(info_str, string) != NULL) {
            uint32_t t_last = get_systick();
            while (get_systick() - t_last < wait_time * 1000 / SYSTICK_US_DIV);
            pkt->dat[0] = 0x80;
            strcpy((char *)pkt->dat + 1, info_str);
            pkt->len = strlen(info_str) + 1;
            pkt->dst = pkt->src;
            cdn_sock_sendto(&sock1, pkt);
            csa.conf_from &= 0x7f;
            return;
        } else {
            csa.conf_from |= 0x80;
        }
    }

    d_debug("p1 ser: ignore\n");
    list_put(dft_ns.free_pkts, &pkt->node);
}


// flash memory manipulation
static void p8_service_routine(void)
{
    // erase:   0x2f, addr_32, len_32  | return [0x80] on success
    // write:   0x20, addr_32 + [data] | return [0x80] on success
    // read:    0x00, addr_32, len_8   | return [0x80, data]
    // cal crc: 0x10, addr_32, len_32  | return [0x80, crc_16]

    cdn_pkt_t *pkt = cdn_sock_recvfrom(&sock8);
    if (!pkt)
        return;

    if (pkt->dat[0] == 0x2f && pkt->len == 9) {
        uint32_t addr = *(uint32_t *)(pkt->dat + 1);
        uint32_t len = *(uint32_t *)(pkt->dat + 5);
        int ret = flash_erase(addr, len);
        pkt->len = 1;
        pkt->dat[0] = ret == HAL_OK ? 0x80 : 0x81;

    } else if (pkt->dat[0] == 0x00 && pkt->len == 6) {
        uint32_t *src_dat = (uint32_t *) *(uint32_t *)(pkt->dat + 1);
        uint8_t len = min(pkt->dat[5], CDN_MAX_DAT - 1);
        memcpy(pkt->dat + 1, src_dat, len);
        d_verbose("nvm read: %08x %d\n", src_dat, len);
        pkt->dat[0] = 0x80;
        pkt->len = len + 1;

    } else if (pkt->dat[0] == 0x20 && pkt->len > 5) {
        uint32_t addr = *(uint32_t *)(pkt->dat + 1);
        uint8_t len = pkt->len - 5;
        int ret = flash_write(addr, len, pkt->dat + 5);
        pkt->len = 1;
        pkt->dat[0] = ret == HAL_OK ? 0x80 : 0x81;
#if 0
    } else if (pkt->len == 9 && pkt->dat[0] == 0x10) {
        uint32_t f_addr = *(uint32_t *)(pkt->dat + 1);
        uint32_t f_len = *(uint32_t *)(pkt->dat + 5);
        uint16_t crc = crc16((const uint8_t *)f_addr, f_len);

        d_debug("nvm crc addr: %x, len: %x, crc: %02x", f_addr, f_len, crc);
        *(uint16_t *)(pkt->dat + 1) = crc;
        pkt->dat[0] = 0x80;
        pkt->len = 3;
#endif
    } else {
        d_warn("nvm: wrong cmd, len: %d\n", pkt->len);
        list_put(dft_ns.free_pkts, &pkt->node);
        return;
    }

    pkt->dst = pkt->src;
    cdn_sock_sendto(&sock8, pkt);
    return;
}


static uint8_t csa_hook_exec(bool after, uint16_t offset, uint8_t len, uint8_t *dat)
{
    uint8_t ret = 0;
    csa_hook_t *hook = dat ? csa_w_hook : csa_r_hook;
    int hook_num = dat ? csa_w_hook_num : csa_r_hook_num;
    for (int i = 0; i < hook_num; i++) {
        hook_func_t hook_func = after ? hook[i].after : hook[i].before;
        if (hook_func) {
            reg2r_t *regr = &hook[i].range;
            uint16_t start = clip(offset, regr->offset, regr->offset + regr->size);
            uint16_t end = clip(offset + len, regr->offset, regr->offset + regr->size);
            if (start != end)
                ret = hook_func(start - regr->offset, end - start, dat ? (dat + (regr->offset - offset)) : NULL);
        }
    }
    return ret;
}


// csa manipulation
static void p5_service_routine(void)
{
    uint32_t flags;
    uint8_t ret_val = 0;
    // read:        0x00, offset_16, len_8   | return [0x80, data]
    // read_dft:    0x01, offset_16, len_8   | return [0x80, data]
    // write:       0x20, offset_16 + [data] | return [0x80] on success

    cdn_pkt_t *pkt = cdn_sock_recvfrom(&sock5);
    if (!pkt)
        return;
    if (csa.conf_from & 0x80) {
        d_warn("csa: avoid cmd, len: %d\n", pkt->len);
        list_put(dft_ns.free_pkts, &pkt->node);
        return;
    }

    if (pkt->dat[0] == 0x00 && pkt->len == 4) {
        uint16_t offset = *(uint16_t *)(pkt->dat + 1);
        uint8_t len = min(pkt->dat[3], CDN_MAX_DAT - 1);

        ret_val = csa_hook_exec(false, offset, len, NULL);
        if (!ret_val) {
            local_irq_save(flags);
            memcpy(pkt->dat + 1, ((void *) &csa) + offset, len);
            local_irq_restore(flags);
            ret_val = csa_hook_exec(true, offset, len, NULL);
        }

        d_debug("csa read: %04x %d\n", offset, len);
        pkt->dat[0] = 0x80 | ret_val;
        pkt->len = len + 1;

    } else if (pkt->dat[0] == 0x20 && pkt->len > 3) {
        uint16_t offset = *(uint16_t *)(pkt->dat + 1);
        uint8_t len = pkt->len - 3;
        uint8_t *src_dat = pkt->dat + 3;

        ret_val = csa_hook_exec(false, offset, len, src_dat);
        if (!ret_val) {
            for (int i = 0; i < csa_w_allow_num; i++) {
                reg2r_t *regr = csa_w_allow + i;
                uint16_t start = clip(offset, regr->offset, regr->offset + regr->size);
                uint16_t end = clip(offset + len, regr->offset, regr->offset + regr->size);
                if (start == end) {
                    //printf("csa i%d: [%x, %x), [%x, %x) -> [%x, %x)\n",
                    //        i, regr->offset, regr->offset + regr->size,
                    //        offset, offset + len,
                    //        start, end);
                    continue;
                }

                //printf("csa @ %p, %p <- %p, len %d, dat[0]: %x\n",
                //        &csa, ((void *) &csa) + start, src_dat + (start - offset), end - start,
                //        *(src_dat + (start - offset)));
                local_irq_save(flags);
                memcpy(((void *) &csa) + start, src_dat + (start - offset), end - start);
                local_irq_restore(flags);
            }
            ret_val = csa_hook_exec(true, offset, len, src_dat);
        }

        d_debug("csa write: %04x %d, ret: %02x\n", offset, len, ret_val);
        pkt->len = 1;
        pkt->dat[0] = 0x80 | ret_val;

    } else if (pkt->dat[0] == 0x01 && pkt->len == 4) {
            uint16_t offset = *(uint16_t *)(pkt->dat + 1);
            uint8_t len = min(pkt->dat[3], CDN_MAX_DAT - 1);
            memcpy(pkt->dat + 1, ((void *) &csa_dft) + offset, len);
            d_debug("csa read_dft: %04x %d\n", offset, len);
            pkt->dat[0] = 0x80;
            pkt->len = len + 1;

    } else {
        d_warn("csa: wrong cmd, len: %d\n", pkt->len);
        list_put(dft_ns.free_pkts, &pkt->node);
        return;
    }

    pkt->dst = pkt->src;
    cdn_sock_sendto(&sock5, pkt);
    return;
}

// qxchg
static void p6_service_routine(void)
{
    uint32_t flags;
    uint8_t ret_val = 0;
    cdn_pkt_t *pkt = cdn_sock_recvfrom(&sock6);
    if (!pkt)
        return;

    if (pkt->dat[0] == 0x2f) { // multicast
        if (1 + csa.qxchg_mcast.offset + csa.qxchg_mcast.size <= pkt->len) {
            memmove(&pkt->dat[1], &pkt->dat[1 + csa.qxchg_mcast.offset], csa.qxchg_mcast.size);
            pkt->dat[0] = 0x20;
            pkt->len = 1 + csa.qxchg_mcast.size;
        }
    }

    if (pkt->dat[0] == 0x20 && pkt->len >= 1) {
        uint8_t *src_dat = pkt->dat + 1;
        uint8_t *dst_dat = pkt->dat + 1;

        for (int i = 0; !ret_val && i < 5; i++) {
            regr_t *regr = csa.qxchg_set + i;
            if (!regr->size)
                break;
            uint16_t lim_size = min(pkt->len - (src_dat - pkt->dat), regr->size);
            if (!lim_size)
                break;

            ret_val = csa_hook_exec(false, regr->offset, lim_size, src_dat);
            if (!ret_val) {
                local_irq_save(flags);
                memcpy(((void *) &csa) + regr->offset, src_dat, lim_size);
                local_irq_restore(flags);
                ret_val = csa_hook_exec(true, regr->offset, lim_size, src_dat);
            }

            src_dat += lim_size;
        }

        for (int i = 0; !ret_val && i < 5; i++) {
            regr_t *regr = csa.qxchg_ret + i;
            if (!regr->size)
                break;
            ret_val = csa_hook_exec(false, regr->offset, regr->size, NULL);
            if (!ret_val) {
                local_irq_save(flags);
                memcpy(dst_dat, ((void *) &csa) + regr->offset, regr->size);
                local_irq_restore(flags);
                ret_val = csa_hook_exec(true, regr->offset, regr->size, NULL);
            }
            dst_dat += regr->size;
        }

        pkt->len = dst_dat - pkt->dat;
        pkt->dat[0] = 0x80 | ret_val;
        d_debug("qxchg: i %d, o %d\n", src_dat - pkt->dat, pkt->len);

    } else if (pkt->dat[0] == 0x00 && pkt->len == 1) {
            uint8_t *dst_dat = pkt->dat + 1;

            for (int i = 0; !ret_val && i < 5; i++) {
                regr_t *regr = csa.qxchg_ro + i;
                if (!regr->size)
                    break;
                ret_val = csa_hook_exec(false, regr->offset, regr->size, NULL);
                if (!ret_val) {
                    local_irq_save(flags);
                    memcpy(dst_dat, ((void *) &csa) + regr->offset, regr->size);
                    local_irq_restore(flags);
                    ret_val = csa_hook_exec(true, regr->offset, regr->size, NULL);
                }
                dst_dat += regr->size;
            }

            pkt->len = dst_dat - pkt->dat;
            pkt->dat[0] = 0x80 | ret_val;

    } else {
        d_warn("qxchg: wrong cmd, len: %d\n", pkt->len);
        list_put(dft_ns.free_pkts, &pkt->node);
        return;
    }

    pkt->dst = pkt->src;
    cdn_sock_sendto(&sock6, pkt);
    return;
}


void common_service_init(void)
{
    cdn_sock_bind(&sock6);
    cdn_sock_bind(&sock5);
    cdn_sock_bind(&sock8);
    cdn_sock_bind(&sock1);
    init_info_str();
    uint32_t *u_id = (uint32_t *)UID_BASE;
    srand(u_id[0] + u_id[1] + u_id[2] + get_systick());
}

void common_service_routine(void)
{
    if (csa.save_conf) {
        csa.save_conf = false;
        save_conf();
    }
    if (csa.do_reboot) {
        gpio_set_value(&drv_en, 0);
        NVIC_SystemReset();
    }
    p1_service_routine();
    p5_service_routine();
    p6_service_routine();
    p8_service_routine();
}

