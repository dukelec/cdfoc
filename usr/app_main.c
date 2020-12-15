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
extern UART_HandleTypeDef huart3;

gpio_t led_r = { .group = LED_R_GPIO_Port, .num = LED_R_Pin };
gpio_t led_g = { .group = LED_G_GPIO_Port, .num = LED_G_Pin };

static gpio_t drv_cs = { .group = DRV_CS_GPIO_Port, .num = DRV_CS_Pin };
static gpio_t s_cs = { .group = SEN_CS_GPIO_Port, .num = SEN_CS_Pin };
static spi_t s_spi = { .hspi = &hspi3, .ns_pin = &s_cs };

uart_t debug_uart = { .huart = &huart3 };

static gpio_t r_rst = { .group = CD_RST_GPIO_Port, .num = CD_RST_Pin };
static gpio_t r_int = { .group = CD_INT_GPIO_Port, .num = CD_INT_Pin };
static gpio_t r_cs = { .group = CD_CS_GPIO_Port, .num = CD_CS_Pin };
static spi_t r_spi = { .hspi = &hspi2, .ns_pin = &r_cs };

static cd_frame_t frame_alloc[FRAME_MAX];
list_head_t frame_free_head = {0};

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
            &r_spi, &r_rst, &r_int);
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
        gpio_set_value(&led_g, 0);
        break;
    default:
    case LED_ERROR:
        is_err = true;
        gpio_set_value(&led_r, 1);
        gpio_set_value(&led_g, 1);
        break;
    }
}


extern uint32_t end; // end of bss
#define STACK_CHECK_SKIP 0x200
#define STACK_CHECK_SIZE (64 + STACK_CHECK_SKIP)

static void stack_check_init(void)
{
    int i;
    printf("stack_check_init: skip: %p ~ %p, to %p\n",
            &end, &end + STACK_CHECK_SKIP, &end + STACK_CHECK_SIZE);
    for (i = STACK_CHECK_SKIP; i < STACK_CHECK_SIZE; i+=4)
        *(uint32_t *)(&end + i) = 0xababcdcd;
}

static void stack_check(void)
{
    int i;
    for (i = STACK_CHECK_SKIP; i < STACK_CHECK_SIZE; i+=4) {
        if (*(uint32_t *)(&end + i) != 0xababcdcd) {
            printf("stack overflow %p (skip: %p ~ %p): %08lx\n",
                    &end + i, &end, &end + STACK_CHECK_SKIP, *(uint32_t *)(&end + i));
            d_error("stack overflow %p (skip: %p ~ %p): %08lx\n",
                    &end + i, &end, &end + STACK_CHECK_SKIP, *(uint32_t *)(&end + i));
            while (true);
        }
    }
}


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

    gpio_set_value(&drv_cs, 0);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&val, (uint8_t *)&rx_val, 1, HAL_MAX_DELAY);
    gpio_set_value(&drv_cs, 1);
    return rx_val & 0x7ff;
}

static void drv_write_reg(uint8_t reg, uint16_t val)
{
    val |= reg << 11;

    gpio_set_value(&drv_cs, 0);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&val, 1, HAL_MAX_DELAY);
    gpio_set_value(&drv_cs, 1);
}


void app_main(void)
{
    printf("\nstart app_main (mdrv)...\n");

    stack_check_init();
    load_conf();
    debug_init(&dft_ns, &csa.dbg_dst, &csa.dbg_en);
    device_init();
    common_service_init();
    d_info("conf (mdrv): %s\n", csa.conf_from ? "load from flash" : "use default");
    //csa_list_show();

    delay_systick(50);
    d_debug("drv 02: %04x\n", drv_read_reg(0x02));
    drv_write_reg(0x02, drv_read_reg(0x02) | 0x1 << 5);
    d_debug("drv 02: %04x\n", drv_read_reg(0x02));

    app_motor_init();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);
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

    while (true) {
        //encoder_read();
        //d_debug("drv: %08x\n", drv_read_reg(0x01) << 16 | drv_read_reg(0x00));

        stack_check();
        app_motor_routine();
        cdn_routine(&dft_ns); // handle cdnet
        common_service_routine();
        debug_flush();
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == r_int.num) {
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
