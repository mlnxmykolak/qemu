/*
 * QEMU mlxreg emulation
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


#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_bridge.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "net/checksum.h"
#include "sysemu/sysemu.h"
#include "sysemu/dma.h"
#include "qemu/iov.h"
#include "qemu/module.h"
#include "qemu/range.h"
#include "hw/pci/msi.h"
#include "qapi/qmp/qerror.h"
#include "qapi/visitor.h"
#include "qapi/error.h"

#include "hw/irq.h"
#include "hw/i386/pc.h"

#include "trace.h"
#include "qom/object.h"

#include "hw/sysbus.h"
#include "hw/i2c/smbus_master.h"

#define _DEBUG_MLXREG

#ifdef DEBUG_MLXREG
#define DPRINTK(FMT, ...) printf(TYPE_PCI_MLXREG_DEV ": " FMT, ## __VA_ARGS__)
#else
#define DPRINTK(FMT, ...) do {} while (0)
#endif

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

typedef struct mlxregState {
    PCIDevice parent_obj;

    MemoryRegion io_mem;
    MemoryRegion io_regmap;
    MemoryRegion io_i2c;
    MemoryRegion io_dummy[4];
    uint8_t* io_regmap_buf;
    uint8_t* io_i2c_buf;

    void *opaque;
    qemu_irq irq;
    qemu_irq pc_irq;

    I2CBus **bus;
    uint8_t i2c_bus_maxnum;
    uint8_t mux_num;
} mlxregState;

#define TYPE_PCI_MLXREG_DEV "mlxreg"

#define PCI_MLXREG_DEV(obj) \
    OBJECT_CHECK(mlxregState, (obj), TYPE_PCI_MLXREG_DEV)



uint8_t io_regmap_buf[MLXREG_SIZE] = { 0x02, 0x09, 0x01, 0x00, 0x78, 0x00, 0xa2, 0x00, 0x6a, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,
                                 0xff, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x04, 0x04, 0x00,
                                 0x5d, 0xdd, 0xdd, 0xdd, 0x0d, 0x09, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x93, 0x00, 0x0d, 0x05, 0xff,
                                 0x10, 0xff, 0x27, 0xff, 0x00, 0x0c, 0xff, 0x00, 0x00, 0xff, 0xff, 0x05, 0xff, 0xff, 0xff, 0xff,
                                 0xff, 0xc1, 0xbf, 0x00, 0xff, 0xff, 0xff, 0xfe, 0x20, 0x00, 0xff, 0x80, 0x00, 0xfe, 0x00, 0x00,
                                 0xfe, 0x30, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x03, 0xff, 0xff, 0xff, 0x03, 0x00,
                                 0x00, 0xff, 0xff, 0xff, 0x03, 0x00, 0x03, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0xff, 0xff, 0xff,
                                 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff,
                                 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x1e, 0x1e, 0x00,
                                 0x00, 0x1e, 0x1e, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0x17, 0x00,
                                 0x00, 0x00, 0x33, 0xff, 0x0d, 0x14, 0x0d, 0x14, 0x0c, 0x13, 0xff, 0x0d, 0x15, 0x0d, 0x13, 0x0d,
                                 0x14, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x3f, 0x04, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

uint8_t io_i2c_buf[MLXREG_SIZE] = { MLXCPLD_I2C_SMBUS_BLK_BIT, 0x01, 0x00, 0x00, 0x29, 0x29, 0xb2, 0x02, 0x00, 0x01, 0x03, 0x6c, 0x62, 0x00, 0x00, 0x00 };

static uint64_t
mlxreg_io_regmap_read(void *opaque, hwaddr addr, unsigned size)
{
    mlxregState *s = opaque;
    uint64_t ret_val=0;

    if (addr < MLXREG_SIZE-size) {
        if(size <= sizeof(ret_val)) {
            memcpy((uint8_t*)&ret_val, &s->io_regmap_buf[addr], size);
            DPRINTK("io regmap addr:%x :size:%d val:0x%x\n", (uint32_t)addr, size, (uint32_t)ret_val);
            return ret_val;
        }
    }
    return 0;

}

static void
mlxreg_io_regmap_write(void *opaque, hwaddr addr, uint64_t val,
                       unsigned size)
{
    mlxregState *s = opaque;

    DPRINTK("io write regmap addr:%x :size:%d val:%x\n", (uint32_t)addr, size, (uint32_t)val);
    if (addr < MLXREG_SIZE-size) {
        switch (size) {
            case 1:
                switch (addr) {
                   case MLXPLAT_CPLD_LPC_I2C_CH1_OFF:
                       if(val+1 < s->i2c_bus_maxnum) {
                           s->mux_num=val+1;
                           DPRINTK("mlxi2c mux i2c-%d\n", s->mux_num);
                       }
                       break;
                   case MLXPLAT_CPLD_LPC_I2C_CH2_OFF:
                       if(val+9 < s->i2c_bus_maxnum) {
                           s->mux_num=val+9;
                           DPRINTK("mlxi2c mux i2c-%d\n", s->mux_num);
                       }
                       break;
                   case MLXPLAT_CPLD_LPC_I2C_CH3_OFF:
                       if(val+17 < s->i2c_bus_maxnum) {
                           s->mux_num=val+17;
                           DPRINTK("mlxi2c mux i2c-%d\n", s->mux_num);
                       }
                       break;
                   case MLXPLAT_CPLD_LPC_I2C_CH4_OFF:
                       if(val+9 < s->i2c_bus_maxnum) {
                           s->mux_num=val+25;
                           DPRINTK("mlxi2c mux i2c-%d\n", s->mux_num);
                       }
                       break;
                }
                break;
            case 2:
                break;
            case 4:
                break;
            default:
                return;
        }
    }
    else {
        return;
    }
    return;
}
static const MemoryRegionOps mlxreg_io_regmap_ops = {
    .read = mlxreg_io_regmap_read,
    .write = mlxreg_io_regmap_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};


static int mlxreg_write_i2c_block(I2CBus *bus, uint8_t addr, uint8_t *data,
                      int len)
{
    int i;

    DPRINTK("i2c block read addr:%x, send_len:%d, len:%d\n", addr, send_len, len);

    if (len > 32) {
        len = 32;
    }

    if (i2c_start_transfer(bus, addr, 0)) {
        return -1;
    }

    for (i = 0; i < len; i++) {
        i2c_send(bus, data[i]);
    }
    i2c_end_transfer(bus);
    return 0;
}

static int mlxreg_read_i2c_block(I2CBus *bus, uint8_t addr, uint8_t *data, int send_len,
                     int rlen)
{
    int i;

    DPRINTK("i2c block read addr:0x%x, send_len:%d, rlen:%d\n", addr, send_len, rlen);

    if (send_len) {
        if (i2c_start_transfer(bus, addr, 0)) {
            return -1;
        }
        for (i = 0; i < send_len; i++) {
            i2c_send(bus, data[i]);
        }
    }
    if (i2c_start_transfer(bus, addr, 1)) {
        if (send_len) {
            i2c_end_transfer(bus);
        }
        return -1;
    }

    for (i = 0; i < rlen; i++) {
        data[i] = i2c_recv(bus);
    }
    i2c_nack(bus);
    i2c_end_transfer(bus);
    return rlen;
}

static uint64_t mlxreg_io_i2c_read(void *opaque, hwaddr addr,
                              unsigned size)
{
    mlxregState *s = opaque;
    uint64_t ret_val=0;

    if (addr < MLXREG_SIZE-size) {
        DPRINTK("io i2c read addr:0x%x size:%d ",(uint32_t)addr, size);
        if(size <= sizeof(ret_val)) {
            memcpy((uint8_t*)&ret_val, &s->io_i2c_buf[addr], size);
            DPRINTK("val:0x%x\n", (uint32_t)ret_val);
            return ret_val;
        }
    }
    return 0;
}

static void mlxreg_io_i2c_write(void *opaque, hwaddr addr,
                           uint64_t val, unsigned size)
{
    mlxregState *s = opaque;
    int ret_len=0;
    DPRINTK("io i2c write addr:0x%x size:%d val:0x%x\n", (uint32_t)addr, size, (uint32_t)val);
    if (addr < MLXREG_SIZE-size) {
        memcpy(&s->io_i2c_buf[addr], (uint8_t*)&val, size);
        switch (addr) {
            case MLXCPLD_LPCI2C_CPBLTY_REG:
                break;
            case MLXCPLD_LPCI2C_CTRL_REG:
                break;
            case MLXCPLD_LPCI2C_HALF_CYC_REG:
                break;
            case MLXCPLD_LPCI2C_I2C_HOLD_REG:
                break;
            case MLXCPLD_LPCI2C_CMD_REG:
                s->io_i2c_buf[MLXCPLD_LPCI2C_STATUS_REG]=MLXCPLD_LPCI2C_NO_IND;

                if(!(s->io_i2c_buf[MLXCPLD_LPCI2C_CMD_REG]&0x1)) {
                    mlxreg_write_i2c_block(s->bus[s->mux_num],
                                       s->io_i2c_buf[MLXCPLD_LPCI2C_CMD_REG]>>1,
                                       &s->io_i2c_buf[MLXCPLD_LPCI2C_DATA_REG],
                                       s->io_i2c_buf[MLXCPLD_LPCI2C_NUM_DAT_REG]+s->io_i2c_buf[MLXCPLD_LPCI2C_NUM_ADDR_REG]);
                }
                else
                {
                    ret_len=mlxreg_read_i2c_block(s->bus[s->mux_num],
                                          s->io_i2c_buf[MLXCPLD_LPCI2C_CMD_REG]>>1,
                                          &s->io_i2c_buf[MLXCPLD_LPCI2C_DATA_REG],
                                          s->io_i2c_buf[MLXCPLD_LPCI2C_NUM_ADDR_REG],
                                          s->io_i2c_buf[MLXCPLD_LPCI2C_NUM_DAT_REG]);
                    if (ret_len > 4) {
                        s->io_i2c_buf[MLXCPLD_LPCI2C_NUM_ADDR_REG]=MLXCPLD_I2C_SMBUS_BLK_BIT;
                        s->io_i2c_buf[MLXCPLD_LPCI2C_NUM_DAT_REG]=ret_len;
                    }
                    else {
                        s->io_i2c_buf[MLXCPLD_LPCI2C_NUM_ADDR_REG]=0;
                    }
                }
                break;
            case MLXCPLD_LPCI2C_NUM_DAT_REG:
                break;
            case MLXCPLD_LPCI2C_NUM_ADDR_REG:
                break;
            case MLXCPLD_LPCI2C_STATUS_REG:
                break;
            case MLXCPLD_LPCI2C_DATA_REG:
                break;
            default:
                break;
        }
    }
    s->io_i2c_buf[MLXCPLD_LPCI2C_STATUS_REG]=MLXCPLD_LPCI2C_TRANS_END;
}

static const MemoryRegionOps mlxreg_io_i2c_ops = {
    .read = mlxreg_io_i2c_read,
    .write = mlxreg_io_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void mlxreg_write_config(PCIDevice *pci_dev, uint32_t address,
                                uint32_t val, int len)
{
    //mlxregState *s = PCI_MLXREG_DEV(pci_dev);

    pci_default_write_config(pci_dev, address, val, len);

}

/* IRQ handling */

static void mlxreg_irq_handler(void *opaque, int irq, int level)
{
    DPRINTK("mlxreg_irq_handler irq:%d, level:% d\n", irq, level);
}


static void mlxreg_realize(PCIDevice *pci_dev, Error **errp)
{
    mlxregState *d = PCI_MLXREG_DEV(pci_dev);
    char bus_name[12]="mlxi2c-1";

    pci_dev->config_write = mlxreg_write_config;

    //pci_dev->config[PCI_INTERRUPT_LINE] = 17;
    //pci_dev->config[PCI_INTERRUPT_PIN] = 3;

    //d->irq = pci_allocate_irq(pci_dev);

    d->irq = qemu_allocate_irq(mlxreg_irq_handler, pci_dev, 1);

    memory_region_init_io(&d->io_i2c, OBJECT(d), &mlxreg_io_i2c_ops, d, "mlxplat_cpld_lpc_i2c_ctrl", 0x100);

    memory_region_init_io(&d->io_regmap, OBJECT(d), &mlxreg_io_regmap_ops, d, "mlxplat_cpld_lpc_regs", 0x100);

    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_IO, &d->io_i2c);

    pci_register_bar(pci_dev, 4, PCI_BASE_ADDRESS_SPACE_IO, &d->io_regmap);

    d->bus=g_malloc0(d->i2c_bus_maxnum * sizeof(I2CBus *));
    for (int i=0; i<d->i2c_bus_maxnum; i++) {
        snprintf(bus_name,12,"mlxi2c-%d",i);
        d->bus[i] = i2c_init_bus(DEVICE(d), bus_name);
    }

    return;
}

static void
mlxreg_uninit(PCIDevice *dev)
{
    mlxregState *d = PCI_MLXREG_DEV(dev);
    g_free(d->bus);
    DPRINTK("unloaded mlxreg pci\n");
}

static void qdev_mlxreg_reset(DeviceState *dev)
{
    //mlxregState *d = PCI_MLXREG_DEV(dev);
}

static void mlxreg_set_i2cbusnum(Object *obj, Visitor *v, const char *name,
                                void *opaque, Error **errp)
{
    mlxregState *d = PCI_MLXREG_DEV(obj);
    int64_t value;
    Error *local_err = NULL;

    visit_type_int(v, name, &value, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    d->i2c_bus_maxnum=value;
}

static void mlxreg_get_i2cbusnum(Object *obj, Visitor *v, const char *name,
                                 void *opaque, Error **errp)
{
    mlxregState *d = PCI_MLXREG_DEV(obj);
    int64_t value = d->i2c_bus_maxnum;
    visit_type_int(v, name, &value, errp);
}


static void mlxreg_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = mlxreg_realize;
    k->exit = mlxreg_uninit;
    k->vendor_id = PCI_VENDOR_ID_NVIDIA;
    k->device_id = 0x0101;
    k->revision = 0x01;
    k->class_id = PCI_CLASS_SYSTEM_OTHER;
    dc->desc = "mlxreg PCI";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->reset = qdev_mlxreg_reset;
}

static void mlxreg_initfn(Object *obj)
{
    mlxregState *d = PCI_MLXREG_DEV(obj);
    d->i2c_bus_maxnum=0;
    d->mux_num=0;
    d->io_regmap_buf=io_regmap_buf;
    d->io_i2c_buf=io_i2c_buf;

    object_property_add(obj, "i2c-busnum", "int",
                        mlxreg_get_i2cbusnum,
                        mlxreg_set_i2cbusnum, NULL, NULL, NULL);
}

static const TypeInfo pci_mlxreg_info = {
    .name          = TYPE_PCI_MLXREG_DEV,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(mlxregState),
    .class_init    = mlxreg_class_init,
    .instance_init = mlxreg_initfn,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void pci_mlxreg_register_types(void)
{
    type_register_static(&pci_mlxreg_info);
}

type_init(pci_mlxreg_register_types)
