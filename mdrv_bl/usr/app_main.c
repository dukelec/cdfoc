/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "app_main.h"

extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart3;

uint32_t *bl_args = (uint32_t *)BL_ARGS;
gpio_t led_g = { .group = LED_G_GPIO_Port, .num = LED_G_Pin };
uart_t debug_uart = { .huart = &huart3 };

static gpio_t r_cs = { .group = CD_CS_GPIO_Port, .num = CD_CS_Pin };
static spi_t r_spi = { .hspi = &hspi2, .ns_pin = &r_cs };

static cd_frame_t frame_alloc[FRAME_MAX];
list_head_t frame_free_head = {0};

static cdn_pkt_t packet_alloc[PACKET_MAX];
list_head_t packet_free_head = {0};

static cdctl_dev_t r_dev = {0};    // CDBUS
cdn_ns_t dft_ns = {0};             // CDNET


static void device_init(void)
{
    int i;
    cdn_init_ns(&dft_ns, &packet_free_head, &frame_free_head);

    for (i = 0; i < FRAME_MAX; i++)
        cd_list_put(&frame_free_head, &frame_alloc[i]);
    for (i = 0; i < PACKET_MAX; i++)
        cdn_list_put(&packet_free_head, &packet_alloc[i]);

    cdctl_cfg_t cfg = csa.bus_cfg;
    if (!csa.keep_in_bl)
        cfg.baud_l = cfg.baud_h = 115200;
    cdctl_dev_init(&r_dev, &frame_free_head, &cfg, &r_spi, NULL);

    // 16MHz / (2 + 2) * (73 + 2) / 2^1 = 150MHz
    cdctl_reg_w(&r_dev, REG_PLL_N, 0x2);
    d_info("pll_n: %02x\n", cdctl_reg_r(&r_dev, REG_PLL_N));
    cdctl_reg_w(&r_dev, REG_PLL_ML, 0x49); // 0x49: 73
    d_info("pll_ml: %02x\n", cdctl_reg_r(&r_dev, REG_PLL_ML));

    d_info("pll_ctrl: %02x\n", cdctl_reg_r(&r_dev, REG_PLL_CTRL));
    cdctl_reg_w(&r_dev, REG_PLL_CTRL, 0x10); // enable pll
    d_info("clk_status: %02x\n", cdctl_reg_r(&r_dev, REG_CLK_STATUS));
    cdctl_reg_w(&r_dev, REG_CLK_CTRL, 0x01); // select pll

    d_info("clk_status after select pll: %02x\n", cdctl_reg_r(&r_dev, REG_CLK_STATUS));
    d_info("version after select pll: %02x\n", cdctl_reg_r(&r_dev, REG_VERSION));

    cdn_add_intf(&dft_ns, &r_dev.cd_dev, csa.bus_net, csa.bus_cfg.mac);
}


#define APP_ADDR 0x08006000 // offset: 24KB

static void jump_to_app(void)
{
    uint32_t stack = *(uint32_t*)APP_ADDR;
    uint32_t func = *(uint32_t*)(APP_ADDR + 4);

    gpio_set_val(&led_g, 0);
    printf("jump app...\n");
    while (!__HAL_UART_GET_FLAG(debug_uart.huart, UART_FLAG_TC));
    HAL_UART_DeInit(debug_uart.huart);
    HAL_SPI_DeInit(r_spi.hspi);
    HAL_RCC_DeInit();
    HAL_DeInit();

    // NOTE: change app's SCB->VTOR in app's system_stm32fxxx.c
    //for(int i = 0; i < 256; i++)
    //    HAL_NVIC_DisableIRQ(i);
    HAL_NVIC_DisableIRQ(SysTick_IRQn);
    __set_MSP(stack); // init stack pointer
    ((void(*)()) func)();
    while (true); // avoid hardfault
}


void app_main(void)
{
    printf("\nstart bl...\n");
    load_conf();
    bool dbg_en_bk = csa.dbg_en;
    csa.dbg_en = false; // silence
    debug_init(&dft_ns, &csa.dbg_dst, &csa.dbg_en);
    csa.keep_in_bl = *bl_args == 0xcdcd0001;
    device_init();
    common_service_init();
    printf("conf: %s, args: %08lx\n", csa.conf_from ? "flash" : "dft", *bl_args);
    gpio_set_val(&led_g, 1);

    uint32_t t_last = get_systick();
    bool update_baud = csa.keep_in_bl;

    while (true) {
        if (get_systick() - t_last > (update_baud ? 100000 : 200000) / SYSTICK_US_DIV) {
            t_last = get_systick();
            gpio_set_val(&led_g, !gpio_get_val(&led_g));
        }

        if (!csa.keep_in_bl && !update_baud && get_systick() > 1000000 / SYSTICK_US_DIV) {
            update_baud = true;
            if (csa.bus_cfg.baud_l != 115200 || csa.bus_cfg.baud_h != 115200) {
                cdctl_set_baud_rate(&r_dev, csa.bus_cfg.baud_l, csa.bus_cfg.baud_h);
                cdctl_flush(&r_dev);
                //printf("baud rate updated, %ld %ld\n", csa.bus_cfg.baud_l, csa.bus_cfg.baud_h);
                d_info("baud updated, %ld %ld\n", csa.bus_cfg.baud_l, csa.bus_cfg.baud_h);
            }
            csa.dbg_en = dbg_en_bk;
        }

        if (!csa.keep_in_bl && get_systick() > 2000000 / SYSTICK_US_DIV)
            jump_to_app();

        cdctl_routine(&r_dev);
        cdn_routine(&dft_ns); // handle cdnet
        common_service_routine();
        debug_flush(false);
    }
}

