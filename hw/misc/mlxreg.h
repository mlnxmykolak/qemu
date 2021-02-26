/*
 * mlxreg - mellanox platform driver
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

#ifndef MLXREG_H
#define MLXREG_H

#include "sysemu/block-backend.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_bridge.h"
#include "mlxreg-hotplug.h"
#include "hw/acpi/aml-build.h"
#include "hw/i386/x86.h"
#include "mlxreg-hotplug.h"


/* LPC I2C registers */
#define MLXCPLD_LPCI2C_CPBLTY_REG   0x0
#define MLXCPLD_LPCI2C_CTRL_REG     0x1
#define MLXCPLD_LPCI2C_HALF_CYC_REG 0x4
#define MLXCPLD_LPCI2C_I2C_HOLD_REG 0x5
#define MLXCPLD_LPCI2C_CMD_REG      0x6
#define MLXCPLD_LPCI2C_NUM_DAT_REG  0x7
#define MLXCPLD_LPCI2C_NUM_ADDR_REG 0x8
#define MLXCPLD_LPCI2C_STATUS_REG   0x9
#define MLXCPLD_LPCI2C_DATA_REG     0xa

/* LPC bus IO offsets */
#define MLXPLAT_CPLD_LPC_I2C_BASE_ADRR      0x2000
#define MLXPLAT_CPLD_LPC_REG_BASE_ADRR      0x2500
#define MLXPLAT_CPLD_LPC_REG_CPLD1_VER_OFFSET   0x00
#define MLXPLAT_CPLD_LPC_REG_CPLD2_VER_OFFSET   0x01
#define MLXPLAT_CPLD_LPC_REG_CPLD3_VER_OFFSET   0x02
#define MLXPLAT_CPLD_LPC_REG_CPLD4_VER_OFFSET   0x03
#define MLXPLAT_CPLD_LPC_REG_CPLD1_PN_OFFSET    0x04
#define MLXPLAT_CPLD_LPC_REG_CPLD2_PN_OFFSET    0x06
#define MLXPLAT_CPLD_LPC_REG_CPLD3_PN_OFFSET    0x08
#define MLXPLAT_CPLD_LPC_REG_CPLD4_PN_OFFSET    0x0a
#define MLXPLAT_CPLD_LPC_REG_RESET_GP4_OFFSET   0x1c
#define MLXPLAT_CPLD_LPC_REG_RESET_CAUSE_OFFSET 0x1d
#define MLXPLAT_CPLD_LPC_REG_RST_CAUSE1_OFFSET  0x1e
#define MLXPLAT_CPLD_LPC_REG_RST_CAUSE2_OFFSET  0x1f
#define MLXPLAT_CPLD_LPC_REG_LED1_OFFSET    0x20
#define MLXPLAT_CPLD_LPC_REG_LED2_OFFSET    0x21
#define MLXPLAT_CPLD_LPC_REG_LED3_OFFSET    0x22
#define MLXPLAT_CPLD_LPC_REG_LED4_OFFSET    0x23
#define MLXPLAT_CPLD_LPC_REG_LED5_OFFSET    0x24
#define MLXPLAT_CPLD_LPC_REG_FAN_DIRECTION  0x2a
#define MLXPLAT_CPLD_LPC_REG_GP0_RO_OFFSET  0x2b
#define MLXPLAT_CPLD_LPC_REG_GP0_OFFSET     0x2e
#define MLXPLAT_CPLD_LPC_REG_GP_RST_OFFSET  0x2f
#define MLXPLAT_CPLD_LPC_REG_GP1_OFFSET     0x30
#define MLXPLAT_CPLD_LPC_REG_WP1_OFFSET     0x31
#define MLXPLAT_CPLD_LPC_REG_GP2_OFFSET     0x32
#define MLXPLAT_CPLD_LPC_REG_WP2_OFFSET     0x33
#define MLXPLAT_CPLD_LPC_REG_PWM_CONTROL_OFFSET 0x37
#define MLXPLAT_CPLD_LPC_REG_AGGR_OFFSET    0x3a
#define MLXPLAT_CPLD_LPC_REG_AGGR_MASK_OFFSET   0x3b
#define MLXPLAT_CPLD_LPC_REG_AGGRLO_OFFSET  0x40
#define MLXPLAT_CPLD_LPC_REG_AGGRLO_MASK_OFFSET 0x41
#define MLXPLAT_CPLD_LPC_REG_AGGRCO_OFFSET  0x42
#define MLXPLAT_CPLD_LPC_REG_AGGRCO_MASK_OFFSET 0x43
#define MLXPLAT_CPLD_LPC_REG_AGGRCX_OFFSET  0x44
#define MLXPLAT_CPLD_LPC_REG_AGGRCX_MASK_OFFSET 0x45
#define MLXPLAT_CPLD_LPC_REG_ASIC_HEALTH_OFFSET 0x50
#define MLXPLAT_CPLD_LPC_REG_ASIC_EVENT_OFFSET  0x51
#define MLXPLAT_CPLD_LPC_REG_ASIC_MASK_OFFSET   0x52
#define MLXPLAT_CPLD_LPC_REG_AGGRLC_OFFSET  0x56
#define MLXPLAT_CPLD_LPC_REG_AGGRLC_MASK_OFFSET 0x57
#define MLXPLAT_CPLD_LPC_REG_PSU_OFFSET     0x58
#define MLXPLAT_CPLD_LPC_REG_PSU_EVENT_OFFSET   0x59
#define MLXPLAT_CPLD_LPC_REG_PSU_MASK_OFFSET    0x5a
#define MLXPLAT_CPLD_LPC_REG_PWR_OFFSET     0x64
#define MLXPLAT_CPLD_LPC_REG_PWR_EVENT_OFFSET   0x65
#define MLXPLAT_CPLD_LPC_REG_PWR_MASK_OFFSET    0x66
#define MLXPLAT_CPLD_LPC_REG_LC_IN_OFFSET   0x70
#define MLXPLAT_CPLD_LPC_REG_LC_IN_EVENT_OFFSET 0x71
#define MLXPLAT_CPLD_LPC_REG_LC_IN_MASK_OFFSET  0x72
#define MLXPLAT_CPLD_LPC_REG_FAN_OFFSET     0x88
#define MLXPLAT_CPLD_LPC_REG_FAN_EVENT_OFFSET   0x89
#define MLXPLAT_CPLD_LPC_REG_FAN_MASK_OFFSET    0x8a
#define MLXPLAT_CPLD_LPC_REG_LC_VR_OFFSET   0x9a
#define MLXPLAT_CPLD_LPC_REG_LC_VR_EVENT_OFFSET 0x9b
#define MLXPLAT_CPLD_LPC_REG_LC_VR_MASK_OFFSET  0x9c
#define MLXPLAT_CPLD_LPC_REG_LC_PG_OFFSET   0x9d
#define MLXPLAT_CPLD_LPC_REG_LC_PG_EVENT_OFFSET 0x9e
#define MLXPLAT_CPLD_LPC_REG_LC_PG_MASK_OFFSET  0x9f
#define MLXPLAT_CPLD_LPC_REG_LC_RD_OFFSET   0xa0
#define MLXPLAT_CPLD_LPC_REG_LC_RD_EVENT_OFFSET 0xa1
#define MLXPLAT_CPLD_LPC_REG_LC_RD_MASK_OFFSET  0xa2
#define MLXPLAT_CPLD_LPC_REG_LC_SN_OFFSET   0xa3
#define MLXPLAT_CPLD_LPC_REG_LC_SN_EVENT_OFFSET 0xa4
#define MLXPLAT_CPLD_LPC_REG_LC_SN_MASK_OFFSET  0xa5
#define MLXPLAT_CPLD_LPC_REG_LC_OK_OFFSET   0xa6
#define MLXPLAT_CPLD_LPC_REG_LC_OK_EVENT_OFFSET 0xa7
#define MLXPLAT_CPLD_LPC_REG_LC_OK_MASK_OFFSET  0xa8
#define MLXPLAT_CPLD_LPC_REG_LC_SD_OFFSET   0xa9
#define MLXPLAT_CPLD_LPC_REG_LC_SD_EVENT_OFFSET 0xaa
#define MLXPLAT_CPLD_LPC_REG_LC_SD_MASK_OFFSET  0xab
#define MLXPLAT_CPLD_LPC_REG_LC_PWR_ON      0xb2
#define MLXPLAT_CPLD_LPC_REG_WD_CLEAR_OFFSET    0xc7
#define MLXPLAT_CPLD_LPC_REG_WD_CLEAR_WP_OFFSET 0xc8
#define MLXPLAT_CPLD_LPC_REG_WD1_TMR_OFFSET 0xc9
#define MLXPLAT_CPLD_LPC_REG_WD1_ACT_OFFSET 0xcb
#define MLXPLAT_CPLD_LPC_REG_WD2_TMR_OFFSET 0xcd
#define MLXPLAT_CPLD_LPC_REG_WD2_TLEFT_OFFSET   0xce
#define MLXPLAT_CPLD_LPC_REG_WD2_ACT_OFFSET 0xcf
#define MLXPLAT_CPLD_LPC_REG_WD3_TMR_OFFSET 0xd1
#define MLXPLAT_CPLD_LPC_REG_WD3_TLEFT_OFFSET   0xd2
#define MLXPLAT_CPLD_LPC_REG_WD3_ACT_OFFSET 0xd3
#define MLXPLAT_CPLD_LPC_REG_CPLD1_MVER_OFFSET  0xde
#define MLXPLAT_CPLD_LPC_REG_CPLD2_MVER_OFFSET  0xdf
#define MLXPLAT_CPLD_LPC_REG_CPLD3_MVER_OFFSET  0xe0
#define MLXPLAT_CPLD_LPC_REG_CPLD4_MVER_OFFSET  0xe1
#define MLXPLAT_CPLD_LPC_REG_UFM_VERSION_OFFSET 0xe2
#define MLXPLAT_CPLD_LPC_REG_PWM1_OFFSET    0xe3
#define MLXPLAT_CPLD_LPC_REG_TACHO1_OFFSET  0xe4
#define MLXPLAT_CPLD_LPC_REG_TACHO2_OFFSET  0xe5
#define MLXPLAT_CPLD_LPC_REG_TACHO3_OFFSET  0xe6
#define MLXPLAT_CPLD_LPC_REG_TACHO4_OFFSET  0xe7
#define MLXPLAT_CPLD_LPC_REG_TACHO5_OFFSET  0xe8
#define MLXPLAT_CPLD_LPC_REG_TACHO6_OFFSET  0xe9
#define MLXPLAT_CPLD_LPC_REG_TACHO_HOLE_OFFSET  0xea
#define MLXPLAT_CPLD_LPC_REG_TACHO7_OFFSET  0xeb
#define MLXPLAT_CPLD_LPC_REG_TACHO8_OFFSET  0xec
#define MLXPLAT_CPLD_LPC_REG_TACHO9_OFFSET  0xed
#define MLXPLAT_CPLD_LPC_REG_TACHO10_OFFSET 0xee
#define MLXPLAT_CPLD_LPC_REG_TACHO11_OFFSET 0xef
#define MLXPLAT_CPLD_LPC_REG_TACHO12_OFFSET 0xf0
#define MLXPLAT_CPLD_LPC_REG_FAN_CAP1_OFFSET    0xf5
#define MLXPLAT_CPLD_LPC_REG_FAN_CAP2_OFFSET    0xf6
#define MLXPLAT_CPLD_LPC_REG_FAN_DRW_CAP_OFFSET 0xf7
#define MLXPLAT_CPLD_LPC_REG_TACHO_SPEED_OFFSET 0xf8
#define MLXPLAT_CPLD_LPC_REG_PSU_I2C_CAP_OFFSET 0xf9
#define MLXPLAT_CPLD_LPC_REG_SLOT_QTY_OFFSET    0xfa
#define MLXPLAT_CPLD_LPC_REG_CONFIG1_OFFSET 0xfb
#define MLXPLAT_CPLD_LPC_REG_CONFIG2_OFFSET 0xfc
#define MLXPLAT_CPLD_LPC_IO_RANGE       0x100
#define MLXPLAT_CPLD_LPC_I2C_CH1_OFF        0xdb
#define MLXPLAT_CPLD_LPC_I2C_CH2_OFF        0xda
#define MLXPLAT_CPLD_LPC_I2C_CH3_OFF        0xdc
#define MLXPLAT_CPLD_LPC_I2C_CH4_OFF        0xdd

/* LPC I2C masks and parametres */
#define MLXCPLD_LPCI2C_RST_SEL_MASK 0x1
#define MLXCPLD_LPCI2C_TRANS_END    0x1
#define MLXCPLD_LPCI2C_STATUS_NACK  0x10
#define MLXCPLD_LPCI2C_NO_IND       0
#define MLXCPLD_LPCI2C_ACK_IND      1
#define MLXCPLD_LPCI2C_NACK_IND     2
#define MLXCPLD_I2C_SMBUS_BLK_BIT   BIT(7)

#define MLXREG_SIZE       0x100

#define MASK(n)        ((1 << (n)) - 1)        /* make an n-bit mask */

typedef struct mlxregState {
    PCIDevice parent_obj;

    MemoryRegion io_regmap;
    MemoryRegion io_i2c;
    uint8_t io_regmap_buf[MLXREG_SIZE];
    uint8_t* io_i2c_buf;
    uint32_t regio_base;
    uint32_t i2cio_base;

    void *opaque;
    qemu_irq irq;

    I2CBus **i2c_bus;
    uint8_t i2c_bus_maxnum;
    uint8_t mux_num;

    BlockBackend *blk;
    mlxreg_hotplug *mbus;
} mlxregState;

#define TYPE_MLXREG_DEV "mlxreg"

#define MLXREG_DEV(obj) \
    OBJECT_CHECK(mlxregState, (obj), TYPE_MLXREG_DEV)

#endif

Aml *build_mlxreg_device_aml(void);
