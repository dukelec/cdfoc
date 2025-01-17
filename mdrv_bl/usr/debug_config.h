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

static inline void arch_dbg_tx(const uint8_t *buf, uint16_t len)
{
#define DBG_UART         USART3

    for (uint16_t i = 0; i < len; i++) {
        while (!(DBG_UART->ISR & UART_FLAG_TXE)); // UART_FLAG_TXFE
        DBG_UART->TDR = *buf++;
    }
}

#endif
