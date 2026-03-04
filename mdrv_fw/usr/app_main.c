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
//#define SEN_ICMU
//#define SEN_MA73X
#define SEN_TLE5012B

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern I2C_HandleTypeDef hi2c1;

gpio_t led_r = { .group = LED_R_GPIO_Port, .num = LED_R_Pin };
gpio_t led_g = { .group = LED_G_GPIO_Port, .num = LED_G_Pin };
gpio_t dbg_out1 = { .group = DBG_OUT1_GPIO_Port, .num = DBG_OUT1_Pin };

static i2c_t temperature_drv = { .hi2c = &hi2c1, .dev_addr = 0x90 };
static i2c_t temperature_motor = { .hi2c = &hi2c1, .dev_addr = 0x92 };

gpio_t drv_en = { .group = DRV_EN_GPIO_Port, .num = DRV_EN_Pin };
static gpio_t drv_fault = { .group = DRV_FAULT_GPIO_Port, .num = DRV_FAULT_Pin };
static gpio_t drv_cs = { .group = DRV_CS_GPIO_Port, .num = DRV_CS_Pin };
gpio_t s_cs = { .group = SEN_CS_GPIO_Port, .num = SEN_CS_Pin };

static gpio_t r_int = { .group = CD_INT_GPIO_Port, .num = CD_INT_Pin };
static gpio_t r_cs = { .group = CD_CS_GPIO_Port, .num = CD_CS_Pin };
static spi_t r_spi = {
        .spi = SPI2,
        .ns_pin = &r_cs,
        .dma_rx = DMA2,
        .dma_ch_rx = DMA2_Channel1,
        .dma_ch_tx = DMA2_Channel2,
        .dma_mask = (2 << 0)
};

static cd_frame_t frame_alloc[FRAME_MAX];
list_head_t frame_free_head = {0};

static cdn_pkt_t packet_alloc[PACKET_MAX];
list_head_t packet_free_head = {0};

cdctl_dev_t r_dev = {0};    // CDBUS
cdn_ns_t dft_ns = {0};      // CDNET


static void device_init(void)
{
    int i;
    cdn_init_ns(&dft_ns, &packet_free_head, &frame_free_head);

    for (i = 0; i < FRAME_MAX; i++)
        cd_list_put(&frame_free_head, &frame_alloc[i]);
    for (i = 0; i < PACKET_MAX; i++)
        cdn_list_put(&packet_free_head, &packet_alloc[i]);

    spi_wr_init(&r_spi);
    cdctl_dev_init(&r_dev, &frame_free_head, &csa.bus_cfg, &r_spi, &r_int, EXTI9_5_IRQn);

    cdn_add_intf(&dft_ns, &r_dev.cd_dev, 0, csa.bus_cfg.mac);
}


static void dump_hw_status(void)
{
    static int t_l = 0;
    if (get_systick() - t_l > 8000) {
        t_l = get_systick();
        printf("ctl: state %d, t_len %ld, r_len %ld, irq %d\n",
                r_dev.state, r_dev.tx_head.len, r_dev.rx_head.len,
                !gpio_get_val(r_dev.int_n));
        printf("  r_cnt %ld (lost %ld, err %ld, no-free %ld), t_cnt %ld (cd %ld, err %ld)\n",
                r_dev.rx_cnt, r_dev.rx_lost_cnt, r_dev.rx_error_cnt,
                r_dev.rx_no_free_node_cnt,
                r_dev.tx_cnt, r_dev.tx_cd_cnt, r_dev.tx_error_cnt);
    }
}


#if defined(SEN_MA73X)
uint16_t encoder_reg_r(uint8_t addr)
{
    uint16_t buf_tx[1];
    uint16_t buf_rx[1];
    buf_tx[0] = (0x40 | addr) << 8;
    gpio_set_val(&s_cs, 0);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)buf_tx, (uint8_t *)buf_rx, 1, HAL_MAX_DELAY);
    gpio_set_val(&s_cs, 1);
    delay_systick(2000 / SYSTICK_US_DIV); // >= 40us
    buf_tx[0] = 0;
    gpio_set_val(&s_cs, 0);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)buf_tx, (uint8_t *)buf_rx, 1, HAL_MAX_DELAY);
    gpio_set_val(&s_cs, 1);
    if ((buf_rx[0] & 0xff) != 0)
        d_error("enc reg r, err ret: %04x\n", buf_rx[0]);
    return buf_rx[0] >> 8;
}

void encoder_reg_w(uint8_t addr, uint16_t val)
{
    uint16_t buf_tx[1];
    uint16_t buf_rx[1];
    buf_tx[0] = ((0x80 | addr) << 8) | val;
    gpio_set_val(&s_cs, 0);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)buf_tx, (uint8_t *)buf_rx, 1, HAL_MAX_DELAY);
    gpio_set_val(&s_cs, 1);
    delay_systick(50000 / SYSTICK_US_DIV); // >= 20ms
    buf_tx[0] = 0;
    gpio_set_val(&s_cs, 0);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)buf_tx, (uint8_t *)buf_rx, 1, HAL_MAX_DELAY);
    gpio_set_val(&s_cs, 1);
    if (((buf_rx[0] & 0xff) != 0) || (buf_rx[0] >> 8) != val)
        d_error("enc reg w, err ret: %04x\n", buf_rx[0]);
    return;
}

#elif defined(SEN_TLE5012B)
uint16_t encoder_reg_r(uint8_t addr)
{
    uint16_t buf[2];
    buf[0] = 0x8001 | (addr << 4);

    gpio_set_val(&s_cs, 0);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, (uint8_t *)buf, 1, HAL_MAX_DELAY);
    gpio_set_val(&s_cs, 1);

    return buf[0];
}

void encoder_reg_w(uint8_t addr, uint16_t val)
{
    uint16_t buf[2];
    buf[0] = 0x0001 | (addr << 4);
    buf[1] = val;

    gpio_set_val(&s_cs, 0);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, 2, HAL_MAX_DELAY);
    gpio_set_val(&s_cs, 1);
}

#endif


#if defined(SEN_MA73X) // MA73x: SPI_POLARITY_LOW, SPI_PHASE_1EDGE, 16BIT ( = DMA data width)

#define SEN_CNT 1
static volatile uint16_t sen_rx_val[1] = {0};
static uint16_t sen_tx_val[1] = {0};
uint16_t encoder_read(void)
{
    return sen_rx_val[0];
}

#elif defined(SEN_ICMU) // ic-MU: SPI_POLARITY_LOW, SPI_PHASE_1EDGE, 8BIT

#define SEN_CNT 3
static volatile uint8_t sen_rx_val[3] = {0};
static uint8_t sen_tx_val[3] = {0xa6, 0, 0};
uint16_t encoder_read(void)
{
    return (sen_rx_val[1] << 8) | sen_rx_val[2];
}

#elif defined(SEN_TLE5012B) // TLE5012B: SPI_POLARITY_LOW, SPI_PHASE_2EDGE, 16BIT

#define SEN_CNT 2           // connection: MCU_MOSI -- resistor -- MCU_MISO -- TLE5012B_DATA
static volatile uint16_t sen_rx_val[2] = {0};
static uint16_t sen_tx_val[2] = {0x8021, 0};
uint16_t encoder_read(void)
{
    return 0xffff - (sen_rx_val[1] << 1);
}

#endif

void encoder_isr_prepare(void)
{
    __HAL_DMA_DISABLE(hspi1.hdmarx);
    hspi1.hdmarx->Instance->CNDTR = SEN_CNT;
    hspi1.hdmarx->Instance->CPAR = (uint32_t)&hspi1.Instance->DR;
    hspi1.hdmarx->Instance->CMAR = (uint32_t)sen_rx_val;
    __HAL_DMA_ENABLE(hspi1.hdmarx);

    __HAL_DMA_DISABLE(hspi1.hdmatx);
    hspi1.hdmatx->Instance->CNDTR = SEN_CNT;
    hspi1.hdmatx->Instance->CPAR = (uint32_t)&hspi1.Instance->DR;
    hspi1.hdmatx->Instance->CMAR = (uint32_t)sen_tx_val;
}

uint16_t drv_read_reg(uint8_t reg)
{
    uint16_t rx_val;
    uint16_t val = 0x8000 | reg << 11;

    gpio_set_val(&drv_cs, 0);
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&val, (uint8_t *)&rx_val, 1, HAL_MAX_DELAY);
    gpio_set_val(&drv_cs, 1);
    return rx_val & 0x7ff;
}

void drv_write_reg(uint8_t reg, uint16_t val)
{
    val |= reg << 11;

    gpio_set_val(&drv_cs, 0);
    HAL_SPI_Transmit(&hspi3, (uint8_t *)&val, 1, HAL_MAX_DELAY);
    gpio_set_val(&drv_cs, 1);
}


void pendsv_user(uint32_t *sp)
{
    if (csa.magic_code == 0x8910) {
        csa.magic_code = 0xcdcd;
        printf("lr: 0x%08lx, pc: 0x%08lx\n", sp[5], sp[6]);
    }

    cdn_poll(&dft_ns); // handle cdnet
    comm_service_poll();
}

__attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile (
        "mrs r0, msp                        \n"
        "ldr r1, =pendsv_user               \n"
        "bx  r1                             \n"
    );
}


void app_main(void)
{
    uint64_t *stack_check = (uint64_t *)((uint32_t)&end + 256);

    gpio_set_val(&led_r, 1);
    gpio_set_val(&led_g, 1);

    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
    HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 2, 2);
    HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 1);

    printf("\nstart app_main (mdrv)...\n");
    *stack_check = 0xababcdcd12123434;

    load_conf();
    device_init();
    comm_service_init();
    d_info("conf (mdrv): %s\n", csa.conf_from ? "load from flash" : "use default");
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    csa_list_show();

    delay_systick(50);
#if defined(SEN_MA73X)
    d_debug("sen reg  RD: %04x\n", encoder_reg_r(0x9));
    d_debug("sen reg  FW: %04x\n", encoder_reg_r(0xe));
    d_debug("sen reg HYS: %04x\n", encoder_reg_r(0x10));
    d_debug("sen reg  MG: %04x\n", encoder_reg_r(0x1b));
    if (encoder_reg_r(0xe) > 0x33) {
        d_debug("sen set FW to 0x33 (51)\n");
        encoder_reg_w(0xe, 0x33);
    }
    //if (encoder_reg_r(0x9) != 0) {
    //    d_debug("sen set RD (rotation direction) to 0\n");
    //    encoder_reg_w(0x9, 0);
    //}
#elif defined(SEN_TLE5012B)
    //d_debug("sen reg1: %x\n", encoder_reg_r(1));
    //encoder_reg_w(1, encoder_reg_r(1) & ~0xee); // ~0xe4
    //d_debug("sen reg1: %x\n", encoder_reg_r(1));
#endif

#if 0
    uint16_t temp_drv_id = 0;
    uint16_t temp_motor_id = 0;
    int temp_drv_ret = i2c_mem_read(&temperature_drv, 0x0f, (uint8_t *)&temp_drv_id, 2);
    int temp_motor_ret = i2c_mem_read(&temperature_motor, 0x0f, (uint8_t *)&temp_motor_id, 2);
    d_debug("temperature id: drv %d: %x, motor %d: %x\n",
            temp_drv_ret, temp_drv_id, temp_motor_ret, temp_motor_id);
#endif

    app_motor_init();
    // adc 24.5 cycles @ 170M/4 -> 576 nS
    LL_ADC_SetChannelSamplingTime(hadc1.Instance, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_24CYCLES_5);
    LL_ADC_SetChannelSamplingTime(hadc1.Instance, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_24CYCLES_5);
    LL_ADC_SetChannelSamplingTime(hadc1.Instance, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_24CYCLES_5);
    LL_ADC_SetChannelSamplingTime(hadc2.Instance, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_24CYCLES_5);
    LL_ADC_SetChannelSamplingTime(hadc2.Instance, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_24CYCLES_5);
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);
    HAL_ADCEx_InjectedStart_IT(&hadc2);
    HAL_ADCEx_InjectedStart_IT(&hadc1);

    static uint16_t gpo_tx_val = SEN_CS_Pin;
    HAL_DMA_Start(htim1.hdma[TIM_DMA_ID_CC4], (uint32_t)&gpo_tx_val, (uint32_t)&SEN_CS_GPIO_Port->BRR, 1);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC4);


    SET_BIT(hspi1.Instance->CR2, SPI_CR2_RXDMAEN);
    SET_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);
    encoder_isr_prepare();
    __HAL_SPI_ENABLE(&hspi1);

    d_info("start pwm...\n");
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, DRV_PWM_HALF);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, DRV_PWM_HALF);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, DRV_PWM_HALF);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 300); // >= 1, ```|_|``` trigger on neg-edge, sensor
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, 190); // >= 1, ```|_|``` trigger on pos-edge, adc

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);

    d_info("pwm on.\n");
    gpio_set_val(&led_r, 0);

    uint32_t last_fault_val = 0xffffffff;

    while (true) {
        if (csa.state != ST_STOP && !gpio_get_val(&drv_fault)) {
            uint32_t cur_fault_val = (drv_read_reg(0x00) << 16) | drv_read_reg(0x01);
            csa.cali_run = false;
            if (cur_fault_val != last_fault_val) {
                d_error("drv status: %08lx\n", cur_fault_val);
                last_fault_val = cur_fault_val;
            }
            if (cur_fault_val != 0) {
                csa.err_flag_.drv_fault = 1;
                if (cur_fault_val & (1 << (8+16)))
                    csa.err_flag_.drv_gdf = 1;
                if (cur_fault_val & (1 << 7))
                    csa.err_flag_.drv_otw = 1;
                if (cur_fault_val & (1 << (6+16)))
                    csa.err_flag_.drv_otsd = 1;
                if (cur_fault_val & (1 << (7+16)))
                    csa.err_flag_.drv_uvlo = 1;
                if (cur_fault_val & (1 << 6))
                    csa.err_flag_.drv_cpuv = 1;
                if (cur_fault_val & (7 << 8))
                    csa.err_flag_.drv_oc = 1;
                if (cur_fault_val & (1 << (9+16)))
                    csa.err_flag_.drv_vds_ocp = 1;
            }
        }
        if (csa.err_flag) {
            gpio_set_val(&led_r, 1);
            gpio_set_val(&led_g, 0);
        } else {
            gpio_set_val(&led_r, 0);
            gpio_set_val(&led_g, 1);
        }

        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
        app_motor_maintain();
        cali_elec_angle();
        sl_maintain();
        if (csa.dbg_str_msk & (1 << 0))
            dump_hw_status();

        if (*stack_check != 0xababcdcd12123434) {
            printf("stack overflow\n");
            while (true);
        }
    }
}


void cdctl_rx_cb(cdctl_dev_t *dev, cd_frame_t *frame)
{
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

void EXTI9_5_IRQHandler(void)
{
    __HAL_GPIO_EXTI_CLEAR_IT(CD_INT_Pin);
    cdctl_int_isr(&r_dev);
}

void DMA2_Channel1_IRQHandler(void)
{
    r_spi.dma_rx->IFCR = r_spi.dma_mask;
    cdctl_spi_isr(&r_dev);
}

void TIM1_CC_IRQHandler(void)
{
    encoder_isr();
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC4);
}

void ADC1_2_IRQHandler(void)
{
    // reading register JDRx automatically clears ADC_FLAG_JEOC
    current_loop_update();
}
