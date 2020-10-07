/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#include "app_main.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern UART_HandleTypeDef huart1;

gpio_t led_r = { .group = LED_RED_GPIO_Port, .num = LED_RED_Pin };
gpio_t led_g = { .group = LED_GRN_GPIO_Port, .num = LED_GRN_Pin };

static gpio_t drv_en = { .group = DRV_EN_GPIO_Port, .num = DRV_EN_Pin };
static gpio_t drv_nss = { .group = DRV_NSS_GPIO_Port, .num = DRV_NSS_Pin };
static gpio_t s_nss = { .group = S_NSS_GPIO_Port, .num = S_NSS_Pin };
static spi_t s_spi = { .hspi = &hspi2, .ns_pin = &s_nss };

uart_t debug_uart = { .huart = &huart1 };

static gpio_t r_rst_n = { .group = CDCTL_RST_N_GPIO_Port, .num = CDCTL_RST_N_Pin };
static gpio_t r_int_n = { .group = CDCTL_INT_N_GPIO_Port, .num = CDCTL_INT_N_Pin };
static gpio_t r_ns = { .group = CDCTL_NSS_GPIO_Port, .num = CDCTL_NSS_Pin };
static spi_t r_spi = { .hspi = &hspi1, .ns_pin = &r_ns };

#define FRAME_MAX 10
static cd_frame_t frame_alloc[FRAME_MAX];
static list_head_t frame_free_head = {0};

#define PACKET_MAX 50
static cdn_pkt_t packet_alloc[PACKET_MAX];

static cdctl_dev_t r_dev = {0};    // CDBUS
cdn_ns_t dft_ns = {0};             // CDNET


static void device_init(void)
{
    int i;
    for (i = 0; i < FRAME_MAX; i++)
        list_put(&frame_free_head, &frame_alloc[i].node);
    for (i = 0; i < PACKET_MAX; i++)
        list_put(&dft_ns.free_pkts, &packet_alloc[i].node);

    cdctl_dev_init(&r_dev, &frame_free_head, csa.bus_mac,
            csa.bus_baud_low, csa.bus_baud_high,
            &r_spi, &r_rst_n, &r_int_n);
    dft_ns.intfs[0].dev = &r_dev.cd_dev;
    dft_ns.intfs[0].net = csa.bus_net;
    dft_ns.intfs[0].mac = csa.bus_mac;
}

void set_led_state(led_state_t state)
{
    static bool is_err = false;
    if (is_err)
        return;

    switch (state) {
    case LED_POWERON:
        gpio_set_value(&led_r, 0);
        gpio_set_value(&led_g, 1);
        break;
    case LED_WARN:
        gpio_set_value(&led_r, 1);
        gpio_set_value(&led_g, 1);
        break;
    default:
    case LED_ERROR:
        is_err = true;
        gpio_set_value(&led_r, 1);
        gpio_set_value(&led_g, 0);
        break;
    }
}

#ifdef BOOTLOADER
#define APP_ADDR 0x08010000 // offset: 64KB

static void jump_to_app(void)
{
    uint32_t stack = *(uint32_t*)APP_ADDR;
    uint32_t func = *(uint32_t*)(APP_ADDR + 4);
    printf("jump to app...\n");
    __set_MSP(stack); // init stack pointer
    ((void(*)()) func)();
}
#endif


uint16_t encoder_read(void)
{
    uint8_t buf[4];
    spi_mem_read(&s_spi, 0xa6, buf, 2); // 4
    //uint32_t ret_val = buf[0] << 16 | buf[1] << 8 | buf[2] << 0;// | buf[3];
    uint16_t ret_val = buf[0] << 8 | buf[1]; // | buf[2] << 0;// | buf[3];
    //d_debug("= %08x\n", ret_val);
    return ret_val;
}

static uint16_t drv_read_reg(uint8_t reg)
{
    uint16_t rx_val;
    uint16_t val = 0x8000 | reg << 11;

    gpio_set_value(&drv_nss, 0);
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&val, (uint8_t *)&rx_val, 1, HAL_MAX_DELAY);
    gpio_set_value(&drv_nss, 1);
    return rx_val & 0x7ff;
}

static void drv_write_reg(uint8_t reg, uint16_t val)
{
    val |= reg << 11;

    gpio_set_value(&drv_nss, 0);
    HAL_SPI_Transmit(&hspi3, (uint8_t *)&val, 1, HAL_MAX_DELAY);
    gpio_set_value(&drv_nss, 1);
}


void app_main(void)
{
#ifdef BOOTLOADER
    printf("\nstart app_main (bl_wait: %d)...\n", csa.bl_wait);
#else
    printf("\nstart app_main...\n");
#endif

    debug_init(&dft_ns, &csa.dbg_dst, &csa.dbg_en);
    load_conf();
    device_init();
    common_service_init();
    app_motor_init();

    gpio_set_value(&drv_en, 1);
    delay_systick(50);
    d_debug("drv 02: %04x\n", drv_read_reg(0x02));
    drv_write_reg(0x02, drv_read_reg(0x02) | 0x1 << 5);
    d_debug("drv 02: %04x\n", drv_read_reg(0x02));

    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);
    HAL_ADC_Start(&hadc3);
    HAL_ADCEx_InjectedStart_IT(&hadc1);

    d_info("start pwm...\n");
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, DRV_PWM_HALF);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, DRV_PWM_HALF);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, DRV_PWM_HALF);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 100);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    d_info("pwm on.\n");
    set_led_state(LED_POWERON);
    //delay_systick(500);
    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 4095);
    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 4095);
    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 4095);
    //while(1);

#ifdef BOOTLOADER
    uint32_t boot_time = get_systick();
#endif

    while (true) {
#ifdef BOOTLOADER
        if (csa.bl_wait != 0xff &&
                get_systick() - boot_time > csa.bl_wait * 100000 / SYSTICK_US_DIV)
            jump_to_app();
#endif

        //encoder_read();
        //d_debug("drv: %08x\n", drv_read_reg(0x01) << 16 | drv_read_reg(0x00));


        cdn_routine(&dft_ns); // handle cdnet
        common_service_routine();
        app_motor();
        debug_flush();
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == r_int_n.num) {
        cdctl_int_isr(&r_dev);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    cdctl_spi_isr(&r_dev);
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    cdctl_spi_isr(&r_dev);
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    cdctl_spi_isr(&r_dev);
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    printf("spi error...\n");
}
