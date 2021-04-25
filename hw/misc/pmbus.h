/*
 * PMBUS device
 *
 *
 * Copyright (C) 2021 Mykola Kostenok <c_mykolak@nvidia.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#ifndef QEMU_PMBUS_H
#define QEMU_PMBUS_H

#include "hw/i2c/i2c.h"
#include <math.h>

#include "sysemu/block-backend.h"
#include "hw/i2c/smbus_slave.h"
/*
 * Registers
 */
enum pmbus_regs {
    PMBUS_PAGE          = 0x00,
    PMBUS_OPERATION         = 0x01,
    PMBUS_ON_OFF_CONFIG     = 0x02,
    PMBUS_CLEAR_FAULTS      = 0x03,
    PMBUS_PHASE         = 0x04,

    PMBUS_CAPABILITY        = 0x19,
    PMBUS_QUERY         = 0x1A,

    PMBUS_VOUT_MODE         = 0x20,
    PMBUS_VOUT_COMMAND      = 0x21,
    PMBUS_VOUT_TRIM         = 0x22,
    PMBUS_VOUT_CAL_OFFSET       = 0x23,
    PMBUS_VOUT_MAX          = 0x24,
    PMBUS_VOUT_MARGIN_HIGH      = 0x25,
    PMBUS_VOUT_MARGIN_LOW       = 0x26,
    PMBUS_VOUT_TRANSITION_RATE  = 0x27,
    PMBUS_VOUT_DROOP        = 0x28,
    PMBUS_VOUT_SCALE_LOOP       = 0x29,
    PMBUS_VOUT_SCALE_MONITOR    = 0x2A,

    PMBUS_COEFFICIENTS      = 0x30,
    PMBUS_POUT_MAX          = 0x31,

    PMBUS_FAN_CONFIG_12     = 0x3A,
    PMBUS_FAN_COMMAND_1     = 0x3B,
    PMBUS_FAN_COMMAND_2     = 0x3C,
    PMBUS_FAN_CONFIG_34     = 0x3D,
    PMBUS_FAN_COMMAND_3     = 0x3E,
    PMBUS_FAN_COMMAND_4     = 0x3F,

    PMBUS_VOUT_OV_FAULT_LIMIT   = 0x40,
    PMBUS_VOUT_OV_FAULT_RESPONSE    = 0x41,
    PMBUS_VOUT_OV_WARN_LIMIT    = 0x42,
    PMBUS_VOUT_UV_WARN_LIMIT    = 0x43,
    PMBUS_VOUT_UV_FAULT_LIMIT   = 0x44,
    PMBUS_VOUT_UV_FAULT_RESPONSE    = 0x45,
    PMBUS_IOUT_OC_FAULT_LIMIT   = 0x46,
    PMBUS_IOUT_OC_FAULT_RESPONSE    = 0x47,
    PMBUS_IOUT_OC_LV_FAULT_LIMIT    = 0x48,
    PMBUS_IOUT_OC_LV_FAULT_RESPONSE = 0x49,
    PMBUS_IOUT_OC_WARN_LIMIT    = 0x4A,
    PMBUS_IOUT_UC_FAULT_LIMIT   = 0x4B,
    PMBUS_IOUT_UC_FAULT_RESPONSE    = 0x4C,

    PMBUS_OT_FAULT_LIMIT        = 0x4F,
    PMBUS_OT_FAULT_RESPONSE     = 0x50,
    PMBUS_OT_WARN_LIMIT     = 0x51,
    PMBUS_UT_WARN_LIMIT     = 0x52,
    PMBUS_UT_FAULT_LIMIT        = 0x53,
    PMBUS_UT_FAULT_RESPONSE     = 0x54,
    PMBUS_VIN_OV_FAULT_LIMIT    = 0x55,
    PMBUS_VIN_OV_FAULT_RESPONSE = 0x56,
    PMBUS_VIN_OV_WARN_LIMIT     = 0x57,
    PMBUS_VIN_UV_WARN_LIMIT     = 0x58,
    PMBUS_VIN_UV_FAULT_LIMIT    = 0x59,

    PMBUS_IIN_OC_FAULT_LIMIT    = 0x5B,
    PMBUS_IIN_OC_WARN_LIMIT     = 0x5D,

    PMBUS_POUT_OP_FAULT_LIMIT   = 0x68,
    PMBUS_POUT_OP_WARN_LIMIT    = 0x6A,
    PMBUS_PIN_OP_WARN_LIMIT     = 0x6B,

    PMBUS_STATUS_BYTE       = 0x78,
    PMBUS_STATUS_WORD       = 0x79,
    PMBUS_STATUS_VOUT       = 0x7A,
    PMBUS_STATUS_IOUT       = 0x7B,
    PMBUS_STATUS_INPUT      = 0x7C,
    PMBUS_STATUS_TEMPERATURE    = 0x7D,
    PMBUS_STATUS_CML        = 0x7E,
    PMBUS_STATUS_OTHER      = 0x7F,
    PMBUS_STATUS_MFR_SPECIFIC   = 0x80,
    PMBUS_STATUS_FAN_12     = 0x81,
    PMBUS_STATUS_FAN_34     = 0x82,

    PMBUS_READ_VIN          = 0x88,
    PMBUS_READ_IIN          = 0x89,
    PMBUS_READ_VCAP         = 0x8A,
    PMBUS_READ_VOUT         = 0x8B,
    PMBUS_READ_IOUT         = 0x8C,
    PMBUS_READ_TEMPERATURE_1    = 0x8D,
    PMBUS_READ_TEMPERATURE_2    = 0x8E,
    PMBUS_READ_TEMPERATURE_3    = 0x8F,
    PMBUS_READ_FAN_SPEED_1      = 0x90,
    PMBUS_READ_FAN_SPEED_2      = 0x91,
    PMBUS_READ_FAN_SPEED_3      = 0x92,
    PMBUS_READ_FAN_SPEED_4      = 0x93,
    PMBUS_READ_DUTY_CYCLE       = 0x94,
    PMBUS_READ_FREQUENCY        = 0x95,
    PMBUS_READ_POUT         = 0x96,
    PMBUS_READ_PIN          = 0x97,

    PMBUS_REVISION          = 0x98,
    PMBUS_MFR_ID            = 0x99,
    PMBUS_MFR_MODEL         = 0x9A,
    PMBUS_MFR_REVISION      = 0x9B,
    PMBUS_MFR_LOCATION      = 0x9C,
    PMBUS_MFR_DATE          = 0x9D,
    PMBUS_MFR_SERIAL        = 0x9E,

    VPD_POINTER_ADDR        = 0xE8,
    CHSUM_FIELD             = 0xEB,
    PN_VPD_FIELD            = 0xE2,
    SN_VPD_FIELD            = 0xE3,
    REV_VPD_FIELD           = 0xE4,
    MFG_DATE_VPD_FIELD      = 0xE5,
    CAP_VPD_FIELD           = 0xE6,
    RSRVD_VPD_FIELD         = 0xE7,
    RSRVD0_VPD_FIELD        = 0xE9,
    RSRVD1_VPD_FIELD        = 0xEA,
    WP_FIELD                = 0xEC,

/*
 * Virtual registers.
 * Useful to support attributes which are not supported by standard PMBus
 * registers but exist as manufacturer specific registers on individual chips.
 * Must be mapped to real registers in device specific code.
 *
 * Semantics:
 * Virtual registers are all word size.
 * READ registers are read-only; writes are either ignored or return an error.
 * RESET registers are read/write. Reading reset registers returns zero
 * (used for detection), writing any value causes the associated history to be
 * reset.
 * Virtual registers have to be handled in device specific driver code. Chip
 * driver code returns non-negative register values if a virtual register is
 * supported, or a negative error code if not. The chip driver may return
 * -ENODATA or any other error code in this case, though an error code other
 * than -ENODATA is handled more efficiently and thus preferred. Either case,
 * the calling PMBus core code will abort if the chip driver returns an error
 * code when reading or writing virtual registers.
 */
    PMBUS_VIRT_BASE         = 0x100,
    PMBUS_VIRT_READ_TEMP_AVG,
    PMBUS_VIRT_READ_TEMP_MIN,
    PMBUS_VIRT_READ_TEMP_MAX,
    PMBUS_VIRT_RESET_TEMP_HISTORY,
    PMBUS_VIRT_READ_VIN_AVG,
    PMBUS_VIRT_READ_VIN_MIN,
    PMBUS_VIRT_READ_VIN_MAX,
    PMBUS_VIRT_RESET_VIN_HISTORY,
    PMBUS_VIRT_READ_IIN_AVG,
    PMBUS_VIRT_READ_IIN_MIN,
    PMBUS_VIRT_READ_IIN_MAX,
    PMBUS_VIRT_RESET_IIN_HISTORY,
    PMBUS_VIRT_READ_PIN_AVG,
    PMBUS_VIRT_READ_PIN_MIN,
    PMBUS_VIRT_READ_PIN_MAX,
    PMBUS_VIRT_RESET_PIN_HISTORY,
    PMBUS_VIRT_READ_POUT_AVG,
    PMBUS_VIRT_READ_POUT_MIN,
    PMBUS_VIRT_READ_POUT_MAX,
    PMBUS_VIRT_RESET_POUT_HISTORY,
    PMBUS_VIRT_READ_VOUT_AVG,
    PMBUS_VIRT_READ_VOUT_MIN,
    PMBUS_VIRT_READ_VOUT_MAX,
    PMBUS_VIRT_RESET_VOUT_HISTORY,
    PMBUS_VIRT_READ_IOUT_AVG,
    PMBUS_VIRT_READ_IOUT_MIN,
    PMBUS_VIRT_READ_IOUT_MAX,
    PMBUS_VIRT_RESET_IOUT_HISTORY,
    PMBUS_VIRT_READ_TEMP2_AVG,
    PMBUS_VIRT_READ_TEMP2_MIN,
    PMBUS_VIRT_READ_TEMP2_MAX,
    PMBUS_VIRT_RESET_TEMP2_HISTORY,

    PMBUS_VIRT_READ_VMON,
    PMBUS_VIRT_VMON_UV_WARN_LIMIT,
    PMBUS_VIRT_VMON_OV_WARN_LIMIT,
    PMBUS_VIRT_VMON_UV_FAULT_LIMIT,
    PMBUS_VIRT_VMON_OV_FAULT_LIMIT,
    PMBUS_VIRT_STATUS_VMON,

    /*
     * RPM and PWM Fan control
     *
     * Drivers wanting to expose PWM control must define the behaviour of
     * PMBUS_VIRT_PWM_[1-4] and PMBUS_VIRT_PWM_ENABLE_[1-4] in the
     * {read,write}_word_data callback.
     *
     * pmbus core provides a default implementation for
     * PMBUS_VIRT_FAN_TARGET_[1-4].
     *
     * TARGET, PWM and PWM_ENABLE members must be defined sequentially;
     * pmbus core uses the difference between the provided register and
     * it's _1 counterpart to calculate the FAN/PWM ID.
     */
    PMBUS_VIRT_FAN_TARGET_1,
    PMBUS_VIRT_FAN_TARGET_2,
    PMBUS_VIRT_FAN_TARGET_3,
    PMBUS_VIRT_FAN_TARGET_4,
    PMBUS_VIRT_PWM_1,
    PMBUS_VIRT_PWM_2,
    PMBUS_VIRT_PWM_3,
    PMBUS_VIRT_PWM_4,
    PMBUS_VIRT_PWM_ENABLE_1,
    PMBUS_VIRT_PWM_ENABLE_2,
    PMBUS_VIRT_PWM_ENABLE_3,
    PMBUS_VIRT_PWM_ENABLE_4,
    PMBUS_MAX_REG_NUM
};

/*
 * STATUS_BYTE, STATUS_WORD (lower)
 */
#define PB_STATUS_NONE_ABOVE        BIT(0)
#define PB_STATUS_CML           BIT(1)
#define PB_STATUS_TEMPERATURE       BIT(2)
#define PB_STATUS_VIN_UV        BIT(3)
#define PB_STATUS_IOUT_OC       BIT(4)
#define PB_STATUS_VOUT_OV       BIT(5)
#define PB_STATUS_OFF           BIT(6)
#define PB_STATUS_BUSY          BIT(7)

/*
 * CML_FAULT_STATUS
 */
#define PB_CML_FAULT_OTHER_MEM_LOGIC    BIT(0)
#define PB_CML_FAULT_OTHER_COMM     BIT(1)
#define PB_CML_FAULT_PROCESSOR      BIT(3)
#define PB_CML_FAULT_MEMORY     BIT(4)
#define PB_CML_FAULT_PACKET_ERROR   BIT(5)
#define PB_CML_FAULT_INVALID_DATA   BIT(6)
#define PB_CML_FAULT_INVALID_COMMAND    BIT(7)


typedef struct {
    SMBusDevice parent_obj;

    uint8_t ret_len;
    uint8_t cmd_len;

    uint8_t page;
    uint8_t crc_pec;
    BlockBackend *blk;
    uint8_t pmbus_regmap_external[2][256][34];

} PMBUSState;

#define TYPE_PMBUS "pmbus"

#define PMBUS(obj) \
    OBJECT_CHECK(PMBUSState, (obj), TYPE_PMBUS)

#endif

#define L16_EXPONENT    -12
float _linear11_to_float (u_int16_t data);
u_int16_t _float_to_linear11 (float data);
