/*
 * pmbus device emulation.
 *
 *
 * Copyright (C) 2021 Mykola Kostenok <c_mykolak@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "qapi/visitor.h"
#include "qapi/error.h"
#include "pmbus.h"

#define POLY    (0x1070U << 3)
static uint8_t crc8(uint16_t data)
{
    int i;

    for (i = 0; i < 8; i++) {
        if (data & 0x8000)
            data = data ^ POLY;
        data = data << 1;
    }
    return (uint8_t)(data >> 8);
}

/* Incremental CRC8 over count bytes in the array pointed to by p */
static uint8_t i2c_smbus_pec(uint8_t crc, uint8_t *p, size_t count)
{
    int i;

    //printf("crc_init: 0x%02x \n", crc);
    for (i = 0; i < count; i++) {
        crc = crc8((crc ^ p[i]) << 8);
        //printf("crc_data: 0x%02x \n", p[i]);
    }
    //printf("crc: 0x%02x \n", crc);
    return crc;
}

#define I2C_WR_BIT 0x01


static void pmbus_realize(DeviceState *dev, Error **errp)
{
    PMBUSState *s = PMBUS(dev);

    if (s->blk) {
            int64_t size = blk_getlength(s->blk);
            if(size>sizeof(s->pmbus_regmap_external)) {
                size=sizeof(s->pmbus_regmap_external);
            }
            int len = blk_pread(s->blk, 0, s->pmbus_regmap_external, size);
            if (len != size) {
                printf("pmbus data read error \n");
            }
    }

}

static void quick_cmd(SMBusDevice *dev, uint8_t read) {
    printf("quick_cmd \n");
}


static int write_data(SMBusDevice *dev, uint8_t *buf, uint8_t len) {
    PMBUSState *s = PMBUS(dev);

    //printf("write_data cmd:0x%x, len:0x%x\n", buf[0], len);
    s->ret_len=0;
    int i=0;

    for (i=0; i<len;i++) {
        switch(buf[0]) {
            case PMBUS_PAGE:
                 if (i==1) {
                     s->page=buf[1];
                     s->pmbus_regmap_external[s->page][dev->data_buf[0]][0]=1;
                     s->pmbus_regmap_external[s->page][dev->data_buf[0]][1]=buf[1];
                 }
                break;
            case PMBUS_CLEAR_FAULTS:
                s->pmbus_regmap_external[0][PMBUS_STATUS_BYTE][1]=0;
                s->pmbus_regmap_external[0][PMBUS_STATUS_CML][1]=0;
                s->pmbus_regmap_external[0][PMBUS_STATUS_FAN_12][1]=0;
                s->pmbus_regmap_external[0][PMBUS_STATUS_FAN_34][1]=0;
                s->pmbus_regmap_external[0][PMBUS_STATUS_INPUT][1]=0;
                s->pmbus_regmap_external[0][PMBUS_STATUS_IOUT][1]=0;
                s->pmbus_regmap_external[0][PMBUS_STATUS_MFR_SPECIFIC][1]=0;
                s->pmbus_regmap_external[0][PMBUS_STATUS_OTHER][1]=0;
                s->pmbus_regmap_external[0][PMBUS_STATUS_TEMPERATURE][1]=0;
                s->pmbus_regmap_external[0][PMBUS_STATUS_WORD][1]=0;
                s->pmbus_regmap_external[0][PMBUS_STATUS_WORD][2]=0;
                s->pmbus_regmap_external[0][PMBUS_STATUS_VOUT][1]=0;
                s->pmbus_regmap_external[0][PMBUS_VIRT_STATUS_VMON][1]=0;
                break;
            default:
                break;
        }
    }
    return 0;
}

static uint8_t receive_byte(SMBusDevice *dev) {
    uint8_t ret=0;
    PMBUSState *s = PMBUS(dev);
    //I2CBus *bus = I2C_BUS(qdev_get_parent_bus(DEVICE(dev)));
    s->cmd_len=s->pmbus_regmap_external[s->page][dev->data_buf[0]][0];

    // Send length if block command
    uint8_t idx=s->cmd_len>2?s->ret_len:s->ret_len+1;
    s->cmd_len=s->cmd_len>2?s->cmd_len+1:s->cmd_len;

    if (!s->cmd_len) {
        ret=0xff;
        //i2c_nack(bus);
        //s->pmbus_regmap_external[0][PMBUS_STATUS_BYTE][1]|=PB_STATUS_CML;
        //s->pmbus_regmap_external[0][PMBUS_STATUS_CML][1]|=PB_CML_FAULT_INVALID_COMMAND;
    }
    else
    {
        if (!s->ret_len) {
            uint8_t i2c_addr=(dev->i2c.address<<1);
            /* Add partial PEC. Check PEC if last message is a write */
            s->crc_pec=i2c_smbus_pec(0, &i2c_addr, 1);
            s->crc_pec=i2c_smbus_pec(s->crc_pec, &dev->data_buf[0], 1);
            i2c_addr=(dev->i2c.address<<1)|I2C_WR_BIT;
            s->crc_pec=i2c_smbus_pec(s->crc_pec, &i2c_addr, 1);
        }
        if(s->ret_len == s->cmd_len) {
            ret=s->crc_pec;
        }
        else
        {
            ret=s->pmbus_regmap_external[s->page][dev->data_buf[0]][idx];
            s->crc_pec=i2c_smbus_pec(s->crc_pec, &ret, 1);
        }
    }
    //printf("receive_byte cmd:0x%x(%d), cmd_len:%d, ret_len:%d 0x%x\n",dev->data_buf[0],dev->data_buf[0],s->cmd_len ,s->ret_len, ret);
    s->ret_len++;
    return ret;
}

static Property pmbus_class_properties[] = {
    DEFINE_PROP_DRIVE("drive", PMBUSState, blk),
    DEFINE_PROP_END_OF_LIST(),
};

static void pmbus_class_init(ObjectClass *klass, void *data)
{
    SMBusDeviceClass *k = SMBUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = pmbus_realize;
    k->quick_cmd = quick_cmd;
    k->write_data = write_data;
    k->receive_byte = receive_byte;
    device_class_set_props(dc, pmbus_class_properties);
}

static const TypeInfo pmbus_info = {
    .name          = TYPE_PMBUS,
    .parent        = TYPE_SMBUS_DEVICE,
    .instance_size = sizeof(PMBUSState),
    .class_init    = pmbus_class_init,
};

static void pmbus_register_types(void)
{
    type_register_static(&pmbus_info);
}

type_init(pmbus_register_types)
