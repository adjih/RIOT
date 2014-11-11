/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cebit_demo
 * @{
 *
 * @file        viz.c
 * @brief       CeBIT 2014 demo application - router node
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>

#include "radio/types.h"

typedef enum {
    PARENT_SELECT = 0,
    PARENT_DELETE = 1,
    DIO_RCVD = 2,
    DTA_RCVD = 3,
    ALARM = 4,
    CONFIRM = 5,
    WARN = 6,
    DISARMED = 7,
    RESET = 8
} evt_t;

extern radio_address_t id;

void viz_udp_pkt(uint8_t src)
{
    printf("VIZ: UPD packet from %i\n", src);
    printf("fw %i %i %i\n", src, DTA_RCVD, id);
}

void viz_udp_snd(uint8_t dst)
{
    if (id == 32 || id == 33) {
        printf("VIZ: UDP send packet to %i\n", dst);
        printf("fw %i %i %i\n", id, DTA_RCVD, dst);
    }
}

void viz_parent_select(uint8_t parent)
{
    printf("VIZ: RPL %i selected parent: %i\n", id, parent);
}

void viz_parent_deselect(uint8_t parent)
{
    printf("VIZ: RPL %i deleted parent: %i\n", id, parent);
}

void viz_dio(uint8_t src)
{
    printf("VIZ: RPL dio from %i\n", src);
    printf("fw %i %i %i\n", id, DIO_RCVD, src);
}
