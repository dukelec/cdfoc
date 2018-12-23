/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#include "app_main.h"

static cdnet_socket_t pos_sock = { .port = 20 };
static cdnet_socket_t speed_sock = { .port = 21 };

void app_motor_init(void)
{
}


static void pos_mode_service_routine(void)
{
}

static void speed_mode_service_routine(void)
{
}


void app_motor(void)
{
    pos_mode_service_routine();
    speed_mode_service_routine();

}
