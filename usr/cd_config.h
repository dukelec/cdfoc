/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#ifndef __CD_CONFIG_H__
#define __CD_CONFIG_H__

#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>     // provide offsetof, NULL
#include <stdint.h>

#define ARCH_SPI
#define CD_LIST_IT

#define DEBUG
#define VERBOSE
#define DBG_STR_LEN         100
#define DBG_LEN             100
#define DBG_TX_IT
//#define LIST_DEBUG

#define CDUART_IRQ_SAFE
#define CDUART_IDLE_TIME    (500000 / SYSTICK_US_DIV) // 500 ms

#define SEQ_TIMEOUT         (500000 / SYSTICK_US_DIV) // 500 ms

#include "main.h"
#include "arch_wrapper.h"
#include "debug_config.h"
#include "cd_debug.h"

#endif
