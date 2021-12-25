/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#include "math.h"
#include "app_main.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;

gpio_t led_r = { .group = LED_R_GPIO_Port, .num = LED_R_Pin };
gpio_t led_g = { .group = LED_G_GPIO_Port, .num = LED_G_Pin };
gpio_t dbg_out1 = { .group = DBG_OUT1_GPIO_Port, .num = DBG_OUT1_Pin };

static i2c_t temperature_drv = { .hi2c = &hi2c1, .dev_addr = 0x90 };
static i2c_t temperature_motor = { .hi2c = &hi2c1, .dev_addr = 0x92 };

gpio_t drv_en = { .group = DRV_EN_GPIO_Port, .num = DRV_EN_Pin };
static gpio_t drv_fault = { .group = DRV_FAULT_GPIO_Port, .num = DRV_FAULT_Pin };
static gpio_t drv_cs = { .group = DRV_CS_GPIO_Port, .num = DRV_CS_Pin };
//static gpio_t s_cs = { .group = SEN_CS_GPIO_Port, .num = SEN_CS_Pin };
//static spi_t s_spi = { .hspi = &hspi1, .ns_pin = &s_cs };

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

static cdn_sock_t sock_vib_rx = { .port = 0xb, .ns = &dft_ns };


static void device_init(void)
{
    int i;
    cdn_init_ns(&dft_ns);

    for (i = 0; i < FRAME_MAX; i++)
        list_put(&frame_free_head, &frame_alloc[i].node);
    for (i = 0; i < PACKET_MAX; i++)
        list_put(&dft_ns.free_pkts, &packet_alloc[i].node);

    cdctl_dev_init(&r_dev, &frame_free_head, &csa.bus_cfg, &r_spi, &r_rst, &r_int);
    cdn_add_intf(&dft_ns, &r_dev.cd_dev, csa.bus_net, csa.bus_cfg.mac);
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

#if 1

static volatile uint16_t sen_rx_val = 0;

// MA73x: SPI_POLARITY_LOW, SPI_PHASE_1EDGE, 16BIT, hw-cs
uint16_t encoder_read(void)
{
    return sen_rx_val;
#if 0
    uint16_t buf_tx[1] = { 0 };
    uint16_t buf_rx[1];

    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)buf_tx, (uint8_t *)buf_rx, 1, HAL_MAX_DELAY);
    //d_debug("%04x %04x\n", buf[0], buf[1]);
    return buf_rx[0];
#endif
}

uint16_t encoder_reg_r(uint8_t addr)
{
    uint16_t buf_tx[1];
    uint16_t buf_rx[1];
    buf_tx[0] = (0x40 | addr) << 8;
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)buf_tx, (uint8_t *)buf_rx, 1, HAL_MAX_DELAY);
    buf_tx[0] = 0;
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)buf_tx, (uint8_t *)buf_rx, 1, HAL_MAX_DELAY);
    if ((buf_rx[0] & 0xff) != 0)
        d_error("enc reg r, err ret: %04x\n", buf_rx[0]);
    return buf_rx[0] >> 8;
}

void encoder_reg_w(uint8_t addr, uint16_t val)
{
    uint16_t buf_tx[1];
    uint16_t buf_rx[1];
    buf_tx[0] = ((0x80 | addr) << 8) | val;
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)buf_tx, (uint8_t *)buf_rx, 1, HAL_MAX_DELAY);
    delay_systick(50000 / SYSTICK_US_DIV);
    buf_tx[0] = 0;
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)buf_tx, (uint8_t *)buf_rx, 1, HAL_MAX_DELAY);
    if (((buf_rx[0] & 0xff) != 0) || (buf_rx[0] >> 8) != val)
        d_error("enc reg w, err ret: %04x\n", buf_rx[0]);
    return;
}

#elif 1
// TLE5012B: SPI_POLARITY_LOW, SPI_PHASE_2EDGE, 16BIT
uint16_t encoder_read(void)
{
    uint16_t buf[2];
    buf[0] = 0x8021;

    gpio_set_value(&s_cs, 0);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, 1, HAL_MAX_DELAY);

    GPIOB->MODER &= ~(1 << (5 * 2 + 1)); // PB5
    HAL_SPI_Receive(&hspi1, (uint8_t *)buf, 2, HAL_MAX_DELAY);
    gpio_set_value(&s_cs, 1);
    GPIOB->MODER |= 1 << (5 * 2 + 1);

    //d_debug("%04x %04x\n", buf[0], buf[1]);

    return 0xffff - (buf[0] << 1);
}

uint16_t encoder_reg_r(uint8_t addr)
{
    uint16_t buf[2];
    buf[0] = 0x8001 | (addr << 4);

    gpio_set_value(&s_cs, 0);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, 1, HAL_MAX_DELAY);

    GPIOB->MODER &= ~(1 << (5 * 2 + 1)); // PB5
    HAL_SPI_Receive(&hspi1, (uint8_t *)buf, 1, HAL_MAX_DELAY);
    gpio_set_value(&s_cs, 1);
    GPIOB->MODER |= 1 << (5 * 2 + 1);

    return buf[0];
}

void encoder_reg_w(uint8_t addr, uint16_t val)
{
    uint16_t buf[2];
    buf[0] = 0x0001 | (addr << 4);
    buf[1] = val;

    gpio_set_value(&s_cs, 0);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, 2, HAL_MAX_DELAY);
    gpio_set_value(&s_cs, 1);
}

#else
// ic-MU: SPI_POLARITY_LOW, SPI_PHASE_1EDGE, 8BIT
uint16_t encoder_read(void)
{
    uint8_t buf[4];
    spi_mem_read(&s_spi, 0xa6, buf, 2); // 4
    //uint32_t ret_val = buf[0] << 16 | buf[1] << 8 | buf[2] << 0;// | buf[3];
    uint16_t ret_val = buf[0] << 8 | buf[1]; // | buf[2] << 0;// | buf[3];
    //d_debug("= %08x\n", ret_val);
    return ret_val;
}
#endif

uint16_t drv_read_reg(uint8_t reg)
{
    uint16_t rx_val;
    uint16_t val = 0x8000 | reg << 11;

    gpio_set_value(&drv_cs, 0);
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&val, (uint8_t *)&rx_val, 1, HAL_MAX_DELAY);
    gpio_set_value(&drv_cs, 1);
    return rx_val & 0x7ff;
}

void drv_write_reg(uint8_t reg, uint16_t val)
{
    val |= reg << 11;

    gpio_set_value(&drv_cs, 0);
    HAL_SPI_Transmit(&hspi3, (uint8_t *)&val, 1, HAL_MAX_DELAY);
    gpio_set_value(&drv_cs, 1);
}


void cali_elec_angle(void)
{
    static int pole_cnt = -1;
    static int sub_cnt;
    static uint32_t t_last;
    static uint32_t a0, a1;
    static int amount_f, amount_r;
    static int dir = 1; // -1 or +1
    static int m_first;

    if (!csa.cali_run)
        return;

    if (pole_cnt == -1) {
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
    if (t_cur - t_last < 1000000 / SYSTICK_US_DIV)
        return;
    t_last = t_cur;

    d_debug("cali [%d, %d]", pole_cnt, sub_cnt);

    if (sub_cnt == 0) {
        a0 = csa.ori_encoder;
        d_debug_c(" - a0: %04x\n", a0);
    } else if (sub_cnt == 2) {
        a1 = csa.ori_encoder;
        d_debug_c(" - a1: %04x\n", a1);
    } else {
        d_debug_c("\n");
    }

    if ((dir == 1 && sub_cnt == 3) || (dir == -1 && sub_cnt == 0)) {
        if (a1 < a0)
            a1 += 0x10000;
        uint32_t n = (a0 + (a1 - a0) / 2) & 0xffff;
        int m = (uint16_t)(n - (65536 * pole_cnt / csa.motor_poles));
        if (m_first < 0)
            m_first = m;
        if (m_first < 65536 / 3 && m < 65536 / 2)
            m += 0x10000;
        d_info("cali: n: %04x, m: %04x\n", n, m);
        d_info("cali: ----------------\n");
        if (dir == 1)
            amount_f += m;
        else
            amount_r += m;

        if (dir == -1 && pole_cnt == 0) {
            d_info("cali: finished, result:\n");
            d_info("cali:  cw: %02x\n", (amount_f / csa.motor_poles) & 0xffff);
            d_info("cali: ccw: %02x\n", (amount_r / csa.motor_poles) & 0xffff);
            d_info("cali: avg: %02x\n", ((amount_f + amount_r) / csa.motor_poles / 2) & 0xffff);
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



void mySPI_DMAReceiveCplt(struct __DMA_HandleTypeDef *hdma)
{
    //gpio_set_value(&dbg_out2, 1);
    //gpio_set_value(&dbg_out2, 0);
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
    csa_list_show();

    delay_systick(50);
    d_debug("sen reg  RD: %04x\n", encoder_reg_r(0x9));
    d_debug("sen reg  FW: %04x\n", encoder_reg_r(0xe));
    d_debug("sen reg HYS: %04x\n", encoder_reg_r(0x10));
    d_debug("sen reg  MG: %04x\n", encoder_reg_r(0x1b));
    if (encoder_reg_r(0xe) > 0x33) {
        d_debug("sen set FW to 0x33 (51)\n");
        encoder_reg_w(0xe, 0x33);
    }
    /*
    if (encoder_reg_r(0x9) != 0) {
        d_debug("sen set RD to 0\n");
        encoder_reg_w(0x9, 0);
    }*/

#if 0
    d_debug("sen reg1: %x\n", encoder_reg_r(1));
    //encoder_reg_w(1, encoder_reg_r(1) & ~0xe4);
    encoder_reg_w(1, encoder_reg_r(1) & ~0xee);
    d_debug("sen reg1: %x\n", encoder_reg_r(1));
#endif

    uint16_t temp_drv_id = 0;
    uint16_t temp_motor_id = 0;
    int temp_drv_ret = i2c_mem_read(&temperature_drv, 0x0f, (uint8_t *)&temp_drv_id, 2);
    int temp_motor_ret = i2c_mem_read(&temperature_motor, 0x0f, (uint8_t *)&temp_motor_id, 2);
    d_debug("temperature id: drv %d: %x, motor %d: %x\n",
            temp_drv_ret, temp_drv_id, temp_motor_ret, temp_motor_id);

    app_motor_init();
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);
    HAL_ADCEx_InjectedStart_IT(&hadc2);
    HAL_ADCEx_InjectedStart_IT(&hadc1);

    static uint16_t sen_tx_val = 0;
    HAL_DMA_Start(htim1.hdma[TIM_DMA_ID_CC4], (uint32_t)&sen_tx_val, (uint32_t)&hspi1.Instance->DR, 1);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC4);

    hspi1.hdmarx->XferCpltCallback = mySPI_DMAReceiveCplt;
    SET_BIT(hspi1.Instance->CR2, SPI_RXFIFO_THRESHOLD);
    HAL_DMA_Start_IT(hspi1.hdmarx, (uint32_t)&hspi1.Instance->DR, (uint32_t)&sen_rx_val, 1);
    SET_BIT(hspi1.Instance->CR2, SPI_CR2_RXDMAEN);
    __HAL_SPI_ENABLE(&hspi1);

    d_info("start pwm...\n");
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, DRV_PWM_HALF);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, DRV_PWM_HALF);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, DRV_PWM_HALF);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1);  // >= 1, ```|_|``` trigger on neg-edge, sensor
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, 12); // >= 1, ```|_|``` trigger on neg-edge, adc
    // adc 3.5 cycles @ 170M/4 -> 82.35 nS -> /2 -> 41.2 nS, pwm 12 -> 70.6 nS /1

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    //__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);

    d_info("pwm on.\n");
    set_led_state(LED_POWERON);

    uint32_t last_fault_val = 0xffffffff;
    cdn_sock_bind(&sock_vib_rx);
    uint32_t t_vib = 0;

    while (true) {
        //encoder_read();
        //d_debug("drv: %08x\n", drv_read_reg(0x01) << 16 | drv_read_reg(0x00));

        cdn_pkt_t *pkt = cdn_sock_recvfrom(&sock_vib_rx);
        if (pkt) {
            csa.vib_angle = *(int16_t *)(pkt->dat + 1);
            csa.vib_magnitude = *(int16_t *)(pkt->dat + 3);
            list_put(&dft_ns.free_pkts, &pkt->node);
            if (get_systick() - t_vib > 1000) {
                t_vib = get_systick();
                d_debug("vib rx: %d (%d)\n", csa.vib_angle, csa.vib_magnitude);
            }
        }

        if (!gpio_get_value(&drv_fault)) {
            uint32_t cur_fault_val = (drv_read_reg(0x00) << 16) | drv_read_reg(0x01);
            gpio_set_value(&led_r, 1);
            csa.cali_run = false;
            if (cur_fault_val != last_fault_val) {
                d_error("drv status: %08x\n", cur_fault_val);
                last_fault_val = cur_fault_val;
            }
        }

        stack_check();
        app_motor_routine();
        cdn_routine(&dft_ns); // handle cdnet
        common_service_routine();
        cali_elec_angle();
        debug_flush(false);
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
