/*
 * Texas Instruments TMP102 Temperature Sensor
 *
 * Browse the data sheet:
 *
 *    http://www.ti.com/lit/gpn/tmp102
 *
 * Copyright (C) 2021 Mykola Kostenok <c_mykolak@nvidia.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 */
#ifndef QEMU_TMP102_H
#define QEMU_TMP102_H

#include "hw/i2c/i2c.h"

#define TYPE_TMP102 "tmp102"
#define TMP102(obj) OBJECT_CHECK(TMP102State, (obj), TYPE_TMP102)

typedef enum TMP102Reg {
    TMP102_REG_TEMPERATURE = 0,
    TMP102_REG_CONFIG,
    TMP102_REG_T_LOW,
    TMP102_REG_T_HIGH,
} TMP102Reg;


/**
 * TMP102State:
 * @config: Bits 5 and 6 (value 32 and 64) determine the precision of the
 * temperature. See Table 8 in the data sheet.
 *
 * @see_also: http://www.ti.com/lit/gpn/tmp102
 */
typedef struct TMP102State {
    /*< private >*/
    I2CSlave i2c;
    /*< public >*/

    uint8_t len;
    uint8_t buf[2];
    qemu_irq pin;

    uint8_t pointer;
    uint8_t config[2];
    int16_t temperature;
    int16_t limit[2];
    int faults;
    uint8_t alarm;
} TMP102State;

#endif
