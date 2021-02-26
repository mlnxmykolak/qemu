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

#include "hw/qdev-properties.h"
#include "migration/vmstate.h"

#include "qemu/module.h"
#include "qemu/range.h"
#include "qapi/qmp/qerror.h"
#include "qapi/visitor.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "trace.h"
#include "qom/object.h"
#include "hw/i2c/smbus_master.h"

#include "mlxreg.h"

#define _DEBUG_MLXREG

#ifdef DEBUG_MLXREG
#define DPRINTK(FMT, ...) printf(TYPE_MLXREG_DEV ": " FMT, ## __VA_ARGS__)
#else
#define DPRINTK(FMT, ...) do {} while (0)
#endif

#define ERR(FMT, ...) fprintf(stderr, TYPE_MLXREG_DEV " : " FMT, \
                            ## __VA_ARGS__)

uint8_t io_i2c_buf[MLXREG_SIZE] = { MLXCPLD_I2C_SMBUS_BLK_BIT, 0x01, 0x00, 0x00, 0x29, 0x29, 0xb2, 0x02, 0x00, 0x01, 0x03, 0x6c, 0x62, 0x00, 0x00, 0x00 };

Aml *build_mlxreg_device_aml(void)
{
    Aml *dev;
    Aml *crs;
    uint32_t irqs=0;
    Aml *method;

    dev = aml_device("MLX");
    aml_append(dev, aml_name_decl("_HID", aml_eisaid("MLX0001")));

    crs = aml_resource_template();
    aml_append(crs, aml_word_io(AML_MIN_FIXED, AML_MAX_FIXED,
                                AML_POS_DECODE, AML_ENTIRE_RANGE,
                                0x0000, 0x2000, 0x20ff, 0x0000, 0x0100));
    aml_append(crs, aml_word_io(AML_MIN_FIXED, AML_MAX_FIXED,
                                AML_POS_DECODE, AML_ENTIRE_RANGE,
                                0x0000, 0x2500, 0x25ff, 0x0000, 0x0100));
    irqs = 17;
    aml_append(crs, aml_interrupt(AML_CONSUMER, AML_EDGE,
                                  AML_ACTIVE_HIGH, AML_SHARED,
                                  &irqs, 1));

    aml_append(dev, aml_name_decl("_PRS", crs));
    aml_append(dev, aml_name_decl("_CRS", crs));

    method = aml_method("_SRS", 1, AML_NOTSERIALIZED);
        aml_append(dev, method);

    return dev;
}

static uint64_t
mlxreg_io_regmap_read(void *opaque, hwaddr addr, unsigned size)
{
    mlxregState *s = opaque;
    uint64_t ret_val = 0;

    if (addr < MLXREG_SIZE-size) {
        if (size <= sizeof(ret_val)) {
            memcpy((uint8_t*)&ret_val, &s->io_regmap_buf[addr], size);
            DPRINTK("io read regmap addr:%x :size:%d val:0x%x\n", (uint32_t)addr, size, (uint32_t)ret_val);
            //printf("io read regmap addr:%x :size:%d val:0x%x\n", (uint32_t)addr, size, (uint32_t)ret_val);
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
    uint16_t fan_cap = 0;
    uint8_t i = 0,j = 0;
    uint8_t tacho = 0;

    DPRINTK("io write regmap addr:%x :size:%d val:%x\n", (uint32_t)addr, size, (uint32_t)val);
    /*if(addr != MLXPLAT_CPLD_LPC_I2C_CH1_OFF)
        printf("io write regmap addr:%x :size:%d val:%x\n", (uint32_t)addr, size, (uint32_t)val);*/
    if (addr < MLXREG_SIZE-size) {
        switch (size) {
            case 1:
                switch (addr) {
                   case MLXPLAT_CPLD_LPC_REG_ASIC_EVENT_OFFSET:
                   case MLXPLAT_CPLD_LPC_REG_PSU_EVENT_OFFSET:
                   case MLXPLAT_CPLD_LPC_REG_PWR_EVENT_OFFSET:
                   case MLXPLAT_CPLD_LPC_REG_FAN_EVENT_OFFSET:
                       s->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_AGGR_OFFSET]=0xff;
                       break;
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
                   case MLXPLAT_CPLD_LPC_REG_PWM1_OFFSET:
                       s->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_PWM1_OFFSET]=val;
                       fan_cap=*(uint16_t*)&s->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_FAN_CAP1_OFFSET];
                       for(i=0,j=0; i<17; i++) {
                           if(MLXPLAT_CPLD_LPC_REG_TACHO1_OFFSET+i==MLXPLAT_CPLD_LPC_REG_TACHO_HOLE_OFFSET)
                               continue;

                           if((uint16_t)BIT(j++)&fan_cap) {
                               tacho=(val-((rand() % (14 - 8 + 1)) + 8));
                               DPRINTK("tacho set off:0x%x, val\n", MLXPLAT_CPLD_LPC_REG_TACHO1_OFFSET+i, ~tacho);
                               s->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_TACHO1_OFFSET+i]=~tacho;
                           }
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
    uint64_t ret_val = 0;

    if (addr < MLXREG_SIZE-size) {
        if(size > sizeof(ret_val)) {
            size=sizeof(ret_val);
        }
        memcpy((uint8_t*)&ret_val, &s->io_i2c_buf[addr], size);
        DPRINTK("io i2c read addr:0x%x size:%d val:0x%x\n",(uint32_t)addr, size, (uint32_t)ret_val);
        //printf("io i2c read addr:0x%x size:%d val:0x%x\n",(uint32_t)addr, size, (uint32_t)ret_val);
    }
    return ret_val;
}

static void mlxreg_io_i2c_write(void *opaque, hwaddr addr,
                           uint64_t val, unsigned size)
{
    mlxregState *s = opaque;
    int ret_val = 0;
    DPRINTK("io i2c write addr:0x%x size:%d val:0x%x\n", (uint32_t)addr, size, (uint32_t)val);
    //printf("io i2c write addr:0x%x size:%d val:0x%x\n", (uint32_t)addr, size, (uint32_t)val);
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
                    ret_val=mlxreg_write_i2c_block(s->i2c_bus[s->mux_num],
                                       s->io_i2c_buf[MLXCPLD_LPCI2C_CMD_REG]>>1,
                                       &s->io_i2c_buf[MLXCPLD_LPCI2C_DATA_REG],
                                       s->io_i2c_buf[MLXCPLD_LPCI2C_NUM_DAT_REG]+s->io_i2c_buf[MLXCPLD_LPCI2C_NUM_ADDR_REG]);
                    if (ret_val < 0) {
                        s->io_i2c_buf[MLXCPLD_LPCI2C_STATUS_REG]|=MLXCPLD_LPCI2C_STATUS_NACK;
                    }
                }
                else
                {
                    ret_val=mlxreg_read_i2c_block(s->i2c_bus[s->mux_num],
                                          s->io_i2c_buf[MLXCPLD_LPCI2C_CMD_REG]>>1,
                                          &s->io_i2c_buf[MLXCPLD_LPCI2C_DATA_REG],
                                          s->io_i2c_buf[MLXCPLD_LPCI2C_NUM_ADDR_REG],
                                          s->io_i2c_buf[MLXCPLD_LPCI2C_NUM_DAT_REG]);
                    if (ret_val < 0) {
                        s->io_i2c_buf[MLXCPLD_LPCI2C_STATUS_REG]|=MLXCPLD_LPCI2C_STATUS_NACK;
                    } else {
                        s->io_i2c_buf[MLXCPLD_LPCI2C_NUM_DAT_REG]=ret_val;
                        if (ret_val > 4) {
                             s->io_i2c_buf[MLXCPLD_LPCI2C_NUM_ADDR_REG]|=MLXCPLD_I2C_SMBUS_BLK_BIT;
                        }
                    }
                }
                s->io_i2c_buf[MLXCPLD_LPCI2C_STATUS_REG]|=MLXCPLD_LPCI2C_TRANS_END;
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

static void mlxreg_realize(PCIDevice *pci_dev, Error **errp)
{
    mlxregState *d = MLXREG_DEV(pci_dev);
    X86MachineState *x86ms = X86_MACHINE(qdev_get_machine());

    memset(d->io_regmap_buf, 0, MLXREG_SIZE);

    if (d->blk) {
        int len = blk_pread(d->blk, 0, d->io_regmap_buf, MLXREG_SIZE);

        if (len != MLXREG_SIZE) {
            ERR(TYPE_MLXREG_DEV
                    " : Failed initial sync with backing cpld file\n");
        }
        DPRINTK("Reset read backing cpld file\n");
    }

    // Initially we dont have hotplug devices. If we want to have devices on start attach them by config.
    d->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_FAN_OFFSET] = 0xff;
    d->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_PSU_OFFSET] = 0xff;
    d->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_PWR_OFFSET] = 0x00;
    d->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_ASIC_HEALTH_OFFSET] = 0xff;


    pci_dev->config[PCI_INTERRUPT_LINE] = 17;
    pci_dev->config[PCI_INTERRUPT_PIN] = 2;

    memory_region_init_io(&d->io_i2c, OBJECT(d), &mlxreg_io_i2c_ops, d, "mlxplat_cpld_lpc_i2c_ctrl", MLXREG_SIZE);
    memory_region_init_io(&d->io_regmap, OBJECT(d), &mlxreg_io_regmap_ops, d, "mlxplat_cpld_lpc_regs", MLXREG_SIZE);

    memory_region_add_subregion(get_system_io(), d->i2cio_base, &d->io_i2c);
    memory_region_add_subregion(get_system_io(), d->regio_base, &d->io_regmap);

    d->irq = x86ms->gsi[17];

    char bus_name[12]="mlxi2c-1";
    d->i2c_bus=g_malloc0(d->i2c_bus_maxnum * sizeof(I2CBus *));
    for (int i=0; i<d->i2c_bus_maxnum; i++) {
        snprintf(bus_name, 12, "mlxi2c-%d", i);
        d->i2c_bus[i] = i2c_init_bus(DEVICE(d), bus_name);
    }

    d->mbus = mlxreg_create_bus(DEVICE(d), "mlxreg-hotplug");

    return;
}

static void
mlxreg_uninit(PCIDevice *dev)
{
    mlxregState *d = MLXREG_DEV(dev);
    g_free(d->i2c_bus);
    DPRINTK("unloaded mlxreg pci\n");
}

static void mlxreg_set_i2cbusnum(Object *obj, Visitor *v, const char *name,
                                void *opaque, Error **errp)
{
    mlxregState *d = MLXREG_DEV(obj);
    int64_t value;
    Error *local_err = NULL;

    visit_type_int(v, name, &value, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    d->i2c_bus_maxnum = value;
}

static void mlxreg_get_i2cbusnum(Object *obj, Visitor *v, const char *name,
                                 void *opaque, Error **errp)
{
    mlxregState *d = MLXREG_DEV(obj);
    int64_t value = d->i2c_bus_maxnum;
    visit_type_int(v, name, &value, errp);
}


static Property mlxreg_props[] = {
    DEFINE_PROP_DRIVE("cpld", mlxregState, blk),
    DEFINE_PROP_UINT32("regio_base", mlxregState, regio_base, 0x2500),
    DEFINE_PROP_UINT32("i2cio_base", mlxregState, i2cio_base, 0x2000),
    DEFINE_PROP_END_OF_LIST()
};

static void mlxreg_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = mlxreg_realize;
    k->exit = mlxreg_uninit;
    k->vendor_id = PCI_VENDOR_ID_NVIDIA;
    k->device_id = 0x1001;
    k->revision = 0x01;
    k->class_id = PCI_CLASS_SYSTEM_OTHER;
    dc->desc = "mlxreg";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    device_class_set_props(dc, mlxreg_props);
}

static void mlxreg_initfn(Object *obj)
{
    mlxregState *d = MLXREG_DEV(obj);
    d->i2c_bus_maxnum = 0;
    d->mux_num = 0;
    d->io_i2c_buf = io_i2c_buf;

    object_property_add(obj, "i2c-busnum", "int",
                        mlxreg_get_i2cbusnum,
                        mlxreg_set_i2cbusnum, NULL, NULL, NULL);
}

static const TypeInfo mlxreg_info = {
    .name          = TYPE_MLXREG_DEV,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(mlxregState),
    .class_init    = mlxreg_class_init,
    .instance_init = mlxreg_initfn,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void mlxreg_register_types(void)
{
    type_register_static(&mlxreg_info);
}

type_init(mlxreg_register_types)
