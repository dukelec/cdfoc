/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#include "app_main.h"

static char cpu_id[25];
static char info_str[100];

static cdn_sock_t sock1 = { .port = 1, .ns = &dft_ns };
static cdn_sock_t sock10 = { .port = 10, .ns = &dft_ns };
static cdn_sock_t sock11 = { .port = 11, .ns = &dft_ns };


static void get_uid(char *buf)
{
    const char tlb[] = "0123456789abcdef";
    int i;

    for (i = 0; i < 12; i++) {
        uint8_t val = *((char *)UID_BASE + i);
        buf[i * 2 + 0] = tlb[val & 0xf];
        buf[i * 2 + 1] = tlb[val >> 4];
    }
    buf[24] = '\0';
}

static void init_info_str(void)
{
    // M: model; S: serial string; HW: hardware version; SW: software version
    get_uid(cpu_id);
#ifdef BOOTLOADER
    sprintf(info_str, "M: mdrv (bl); S: %s; SW: %s", cpu_id, SW_VER);
#else
    sprintf(info_str, "M: mdrv; S: %s; SW: %s", cpu_id, SW_VER);
#endif
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
        return;
    }
    d_debug("p1 ser: ignore\n");
    list_put(&dft_ns.free_pkts, &pkt->node);
}


// device control
static void p10_service_routine(void)
{
    cdn_pkt_t *pkt = cdn_sock_recvfrom(&sock10);
    if (!pkt)
        return;
/*
    if (pkt->len && (pkt->dat[0] == 0x60 || pkt->dat[0] == 0x20)) {
        NVIC_SystemReset(); // TODO: return before reset
    } else if (pkt->len && pkt->dat[0] == 0x61) {
        d_debug("p10 ser: save config to flash\n");
        save_conf();
        pkt->len = 1;
        pkt->dat[0] = 0x80;
        pkt->dst = pkt->src;
        cdnet_socket_sendto(&sock10, pkt);
    } else if (pkt->len && pkt->dat[0] == 0x62) {
        d_debug("p10 ser: stay in bootloader\n");
        csa.bl_wait = 0xff;
        pkt->len = 1;
        pkt->dat[0] = 0x80;
        pkt->dst = pkt->src;
        cdnet_socket_sendto(&sock10, pkt);
    } */
    d_debug("p10 ser: ignore\n");
    list_put(&dft_ns.free_pkts, &pkt->node);
}

// flash memory manipulation
static void p11_service_routine(void)
{
    // erase: 0x6f, addr_32, len_32  | return [0x80] on success
    // read:  0x40, addr_32, len_8   | return [0x80, data]
    // write: 0x61, addr_32 + [data] | return [0x80] on success
#if 0
    cdn_pkt_t *pkt = cdn_sock_recvfrom(&sock11);
    if (!pkt)
        return;

    if (pkt->dat[0] == 0x6f && pkt->len == 9) {
        uint8_t ret;
        uint32_t err_page = 0;
        FLASH_EraseInitTypeDef f;
        uint32_t addr = *(uint32_t *)(pkt->dat + 1);
        uint32_t len = *(uint32_t *)(pkt->dat + 5);

        f.TypeErase = FLASH_TYPEERASE_PAGES;
        f.PageAddress = addr;
        f.NbPages = (len + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;

        ret = HAL_FLASH_Unlock();
        if (ret == HAL_OK)
            ret = HAL_FLASHEx_Erase(&f, &err_page);
        ret |= HAL_FLASH_Lock();

        d_debug("nvm erase: %08x +%08x, %08x, ret: %d\n",
                addr, len, err_page, ret);
        pkt->len = 1;
        pkt->dat[0] = ret == HAL_OK ? 0x80 : 0x81;

    } else if (pkt->dat[0] == 0x40 && pkt->len == 6) {
        uint32_t *src_dat = (uint32_t *) *(uint32_t *)(pkt->dat + 1);
        uint8_t len = pkt->dat[5];
        uint8_t cnt = (len + 3) / 4;

        uint32_t *dst_dat = (uint32_t *)(pkt->dat + 1);
        uint8_t i;

        cnt = min(cnt, CDNET_MAX_DAT / 4);

        for (i = 0; i < cnt; i++)
            *(dst_dat + i) = *(src_dat + i);
        d_debug("nvm read: %08x %d(%d)\n", src_dat, len, cnt);
        pkt->dat[0] = 0x80;
        pkt->len = min(cnt * 4, len) + 1;

    } else if (pkt->dat[0] == 0x61 && pkt->len > 5) {
        uint8_t ret;
        uint32_t *dst_dat = (uint32_t *) *(uint32_t *)(pkt->dat + 1);
        uint8_t len = pkt->len - 5;
        uint8_t cnt = (len + 3) / 4;
        uint32_t *src_dat = (uint32_t *)(pkt->dat + 5);
        uint8_t i;

        ret = HAL_FLASH_Unlock();
        for (i = 0; ret == HAL_OK && i < cnt; i++)
            ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                    (uint32_t)(dst_dat + i), *(src_dat + i));
        ret |= HAL_FLASH_Lock();

        d_debug("nvm write: %08x %d(%d), ret: %d\n",
                dst_dat, pkt->len - 5, cnt, ret);
        pkt->len = 1;
        pkt->dat[0] = ret == HAL_OK ? 0x80 : 0x81;

    } else {
        list_put(&cdnet_free_pkts, &pkt->node);
        d_warn("nvm: wrong cmd, len: %d\n", pkt->len);
        return;
    }

    pkt->dst = pkt->src;
    cdnet_socket_sendto(&sock11, pkt);
    return;
#endif
}


void common_service_init(void)
{
    cdn_sock_bind(&sock1);
    cdn_sock_bind(&sock10);
    cdn_sock_bind(&sock11);
    init_info_str();
}

void common_service_routine(void)
{
    p1_service_routine();
    p10_service_routine();
    p11_service_routine();
}
