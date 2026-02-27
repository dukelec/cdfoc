/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __ADC_SAMP_H__
#define __ADC_SAMP_H__

#define ADC_CALI_LEN            50


typedef struct {
    int16_t         sen_i[3];
    int8_t          adc_sel;

    int16_t         regular_i[2];
    volatile bool   has_new_regular;

    volatile int    cali_st;

    // internal
    int             cali_cnt;
    int16_t         adc_ofs[3][2];
    int32_t         acc[2];
} adc_samp_t;


static inline void adc_samp_cali(adc_samp_t *as)
{
    as->adc_sel = 0;
    memset(as->acc, 0, sizeof(as->acc));
    as->cali_cnt = 0;
    as->cali_st = 1;
    while (as->cali_st) {}
    d_debug("adc cali: ab %d %d, ca %d %d, cb %d %d\n", as->adc_ofs[0][0], as->adc_ofs[0][1],
            as->adc_ofs[1][0], as->adc_ofs[1][1], as->adc_ofs[2][0], as->adc_ofs[2][1]);
}


static inline void adc_samp_inject(adc_samp_t *as, bool motor_idle, bool motor_wire_swap)
{
    int16_t adc1_val = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    int16_t adc2_val = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    if (motor_wire_swap)
        swap(adc1_val, adc2_val);

    if (as->adc_sel == 0) {
        as->sen_i[0] = adc1_val - as->adc_ofs[0][0];
        as->sen_i[1] = adc2_val - as->adc_ofs[0][1];
        as->sen_i[2] = -as->sen_i[0] - as->sen_i[1];
    } else if (as->adc_sel == 1) {
        as->sen_i[2] = adc1_val - as->adc_ofs[1][0];
        as->sen_i[0] = adc2_val - as->adc_ofs[1][1];
        as->sen_i[1] = -as->sen_i[0] - as->sen_i[2];
    } else {
        as->sen_i[2] = adc1_val - as->adc_ofs[2][0];
        as->sen_i[1] = adc2_val - as->adc_ofs[2][1];
        as->sen_i[0] = -as->sen_i[2] - as->sen_i[1];
    }

    if (motor_idle && as->cali_st) {
        as->acc[0] += adc1_val;
        as->acc[1] += adc2_val;
        if (++as->cali_cnt >= ADC_CALI_LEN) {
            as->cali_cnt = 0;
            as->adc_ofs[as->adc_sel][0] = DIV_ROUND_CLOSEST(as->acc[0], ADC_CALI_LEN);
            as->adc_ofs[as->adc_sel][1] = DIV_ROUND_CLOSEST(as->acc[1], ADC_CALI_LEN);
            memset(as->acc, 0, sizeof(as->acc));
            if (++as->adc_sel == 3) {
                as->adc_sel = 0;
                as->cali_st = 0;
            }
        }
    }
}


static inline void adc_samp_set_ch(adc_samp_t *as, const int16_t *pwm_uvw, bool switch_en)
{
#ifdef CURRENT_ADC_3CH
    if (switch_en) {
        if (pwm_uvw[2] > max(pwm_uvw[0], pwm_uvw[1])) {
            as->adc_sel = 0;
        } else if (pwm_uvw[1] > max(pwm_uvw[2], pwm_uvw[0])) {
            as->adc_sel = 1;
        } else {
            as->adc_sel = 2;
        }
    }
#endif

    if (as->adc_sel == 0) {
        hadc1.Instance->JSQR = (hadc1.Instance->JSQR & 0x1ff) | (1 << 9); // a
        hadc2.Instance->JSQR = (hadc2.Instance->JSQR & 0x1ff) | (2 << 9); // b
    } else if (as->adc_sel == 1) {
        hadc1.Instance->JSQR = (hadc1.Instance->JSQR & 0x1ff) | (3 << 9); // c
        hadc2.Instance->JSQR = (hadc2.Instance->JSQR & 0x1ff) | (1 << 9); // a
    } else {
        hadc1.Instance->JSQR = (hadc1.Instance->JSQR & 0x1ff) | (3 << 9); // c
        hadc2.Instance->JSQR = (hadc2.Instance->JSQR & 0x1ff) | (2 << 9); // b
    }
}


static inline void adc_samp_regular(adc_samp_t *as)
{
    if (!LL_ADC_REG_IsConversionOngoing(hadc1.Instance)) {
        int16_t adc1_val = HAL_ADC_GetValue(&hadc1); // clear flag by read
        int16_t adc2_val = HAL_ADC_GetValue(&hadc2);
        LL_ADC_REG_StartConversion(hadc1.Instance);
        if (!as->has_new_regular) {
            as->regular_i[0] = adc1_val;
            as->regular_i[1] = adc2_val;
            as->has_new_regular = true;
        }
    }
}

#endif
