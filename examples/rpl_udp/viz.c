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
    EVT_RESET = 8
} evt_t;

extern radio_address_t id;

void viz_udp_pkt(uint16_t src)
{
    printf("VIZ: UPD packet from %04x\n", src);
    printf("fw %04x %i %04x\n", src, DTA_RCVD, id);
}

void viz_udp_snd(uint16_t dst)
{
  //if (id == 32 || id == 33) {
  printf("VIZ: UDP send packet to %04x\n", dst);
  printf("fw %04x %i %04x\n", id, DTA_RCVD, dst);
  //}
}

void viz_parent_select(uint16_t parent)
{
    printf("VIZ: RPL %04x selected parent: %04x\n", id, parent);
}

void viz_parent_deselect(uint16_t parent)
{
    printf("VIZ: RPL %04x deleted parent: %04x\n", id, parent);
}

void viz_dio(uint16_t src)
{
    printf("VIZ: RPL dio from %04x\n", src);
    printf("fw %04x %d %04x\n", id, DIO_RCVD, src);
}
