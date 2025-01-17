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
#if 1

#define DBG_UART         USART3

    for (uint16_t i = 0; i < len; i++) {
        while (!(DBG_UART->ISR & UART_FLAG_TXE)); // UART_FLAG_TXFE
        DBG_UART->TDR = *buf++;
    }

#else

#define DBG_DMA         DMA1
#define DBG_DMA_CH      DMA1_Channel4
#define DBG_DMA_MASK    (2 << 12)       // DMA_ISR.TCIF4

    // init once:
    //DBG_UART->CR3 |= USART_CR3_DMAT;
    //DBG_DMA_CH->CPAR = (uint32_t)&DBG_UART->TDR;

    DBG_DMA_CH->CNDTR = len;
    DBG_DMA_CH->CMAR = (uint32_t)buf;
    DBG_DMA_CH->CCR |= DMA_CCR_EN;

    while (!(DBG_DMA->ISR & DBG_DMA_MASK));
    DBG_DMA->IFCR = DBG_DMA_MASK;
    DBG_DMA_CH->CCR &= ~DMA_CCR_EN;

#endif
}

#endif
