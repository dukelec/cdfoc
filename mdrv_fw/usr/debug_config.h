/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __DEBUG_CONFIG_H__
#define __DEBUG_CONFIG_H__

#include "arch_wrapper.h"

static inline
void dbg_transmit(uart_t *uart, const uint8_t *buf, uint16_t len)
{
#if 1 // avoid hal check
    uint16_t i;
    for (i = 0; i < len; i++) {
        while (!__HAL_UART_GET_FLAG(uart->huart, UART_FLAG_TXE));
        uart->huart->Instance->TDR = *(buf + i);
    }
#else
    HAL_UART_Transmit(uart->huart, (uint8_t *)buf, len, HAL_MAX_DELAY);
#endif
}

static inline
void dbg_transmit_it(uart_t *uart, const uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit_DMA(uart->huart, (uint8_t *)buf, len);
}

static inline bool dbg_transmit_is_ready(uart_t *uart)
{
#if 1 // DMA
    if (uart->huart->TxXferCount == 0) {
        uart->huart->gState = HAL_UART_STATE_READY;
        return true;
    } else {
        return false;
    }
#else
    return uart->huart->gState == HAL_UART_STATE_READY;
#endif
}

#endif
