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

#define CRCPOLY (0x1070U << 3)
static u_int8_t pec_crc8(uint16_t data)
{
    int i;

    for (i = 0; i < 8; i++) {
        if (data & 0x8000) {
            data = data ^ CRCPOLY;
        }
        data = data << 1;
    }
    return (u_int8_t)(data >> 8);
}

/* Incremental CRC8 over count bytes in the array pointed to by p */
static u_int8_t i2c_pec_crc8(uint8_t crc, uint8_t *p, int count)
{
    int i;

    for (i = 0; i < count; i++) {
        crc = pec_crc8((crc ^ p[i]) << 8);
    }
    return crc;
}

static int pmbus_send(I2CSlave *i2c, uint8_t value)
{
    PMBUSState *s = PMBUS(i2c);
    printf("pmbus_send value:0x%x\n", value);
    s->cmd=value;
    switch(s->cmd) {
        case PMBUS_PAGE:
            s->ret_len=0;
            s->cmd_len=0;
            s->page=value;
            break;
        case PMBUS_CLEAR_FAULTS:
            s->ret_len=0;
            s->cmd_len=0;
            break;
        case PMBUS_CAPABILITY:
            s->ret_len=0;
            s->cmd_len=1;
            break;
        case PMBUS_STATUS_BYTE:
            s->ret_len=0;
            s->cmd_len=1;
            break;
        default:
            s->ret_len=0;
            s->cmd_len=2;
            s->crc_pec=0;
            break;
    }
    return 0;
}

static uint8_t pmbus_recv(I2CSlave *i2c)
{
    uint8_t ret=0;
    PMBUSState *s = PMBUS(i2c);
    switch(s->cmd) {
        default:
            ret=s->pmbus_regmap[s->cmd]>>(s->ret_len*8);
            break;
    }
    if(s->ret_len==s->cmd_len) {
        ret=s->crc_pec;
    }
    else
    {
        s->crc_pec=i2c_pec_crc8(s->crc_pec, &ret, 1);
    }
    s->ret_len++;
    printf("pmbus_recv 0x%x\n", ret);
    return ret;
}

static const VMStateDescription vmstate_pmbus = {
    .name = "pmbus",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, PMBUSState),
        VMSTATE_END_OF_LIST()
    }
};

static int pmbus_init(I2CSlave *d)
{
    DeviceState *dev = DEVICE(d);
    PMBUSState *s = PMBUS(dev);

    vmstate_register(VMSTATE_IF(dev), VMSTATE_INSTANCE_ID_ANY,
                     &vmstate_pmbus, s);
    s->pmbus_regmap[PMBUS_PAGE]=0xff;
    s->pmbus_regmap[PMBUS_STATUS_BYTE]=0x01;
    s->pmbus_regmap[PMBUS_STATUS_WORD]=0xffff;
    s->pmbus_regmap[PMBUS_STATUS_VOUT]=0x0;
    s->pmbus_regmap[PMBUS_STATUS_IOUT]=0x0;
    s->pmbus_regmap[PMBUS_STATUS_INPUT]=0x0;
    s->pmbus_regmap[PMBUS_STATUS_TEMPERATURE]=0x0;
    s->pmbus_regmap[PMBUS_STATUS_FAN_12]=0x08;
    s->pmbus_regmap[PMBUS_STATUS_FAN_34]=0xff;
    s->pmbus_regmap[PMBUS_CAPABILITY]=0x10;
    s->pmbus_regmap[PMBUS_VOUT_MODE]=0x6f22;

    s->pmbus_regmap[PMBUS_READ_VIN] = 0xf393;
    s->pmbus_regmap[PMBUS_READ_IIN] = 0xb293;
    s->pmbus_regmap[PMBUS_READ_VCAP] = 0xffff;
    s->pmbus_regmap[PMBUS_READ_VOUT] = 0x1807;
    s->pmbus_regmap[PMBUS_READ_IOUT] = 0xd292;
    s->pmbus_regmap[PMBUS_READ_TEMPERATURE_1] = 0xe25c;
    s->pmbus_regmap[PMBUS_READ_TEMPERATURE_2] = 0xe25c;
    s->pmbus_regmap[PMBUS_READ_TEMPERATURE_3] = 0xe25c;
    s->pmbus_regmap[PMBUS_READ_FAN_SPEED_1] = 0x237b;
    s->pmbus_regmap[PMBUS_READ_FAN_SPEED_2] = 0x237b;
    s->pmbus_regmap[PMBUS_READ_FAN_SPEED_3] = 0x237b;
    s->pmbus_regmap[PMBUS_READ_FAN_SPEED_4] = 0x237b;
    //s->pmbus_regmap[PMBUS_READ_DUTY_CYCLE] = 0x94;
    //s->pmbus_regmap[PMBUS_READ_FREQUENCY] = 0x95;
    s->pmbus_regmap[PMBUS_READ_POUT] = 0xebc4;
    s->pmbus_regmap[PMBUS_READ_PIN] =0xf227;

    s->pmbus_regmap[PMBUS_OT_FAULT_LIMIT] = 0xea08;
    s->pmbus_regmap[PMBUS_OT_FAULT_RESPONSE] = 0x50;
    s->pmbus_regmap[PMBUS_OT_WARN_LIMIT] = 0xe3c0;
    s->pmbus_regmap[PMBUS_UT_WARN_LIMIT] = 0xffff;
    s->pmbus_regmap[PMBUS_UT_FAULT_LIMIT] = 0xdd80;
    /*s->pmbus_regmap[PMBUS_UT_FAULT_RESPONSE     = 0x54,
    s->pmbus_regmap[PMBUS_VIN_OV_FAULT_LIMIT    = 0x55,
    s->pmbus_regmap[PMBUS_VIN_OV_FAULT_RESPONSE = 0x56,
    s->pmbus_regmap[PMBUS_VIN_OV_WARN_LIMIT     = 0x57,
    s->pmbus_regmap[PMBUS_VIN_UV_WARN_LIMIT     = 0x58,
    s->pmbus_regmap[PMBUS_VIN_UV_FAULT_LIMIT    = 0x59,*/

    return 0;
}

static void pmbus_realize(DeviceState *dev, Error **errp)
{
    I2CSlave *i2c = I2C_SLAVE(dev);
    pmbus_init(i2c);
}


static void pmbus_initfn(Object *obj)
{
//    PMBUSState* s=PMBUS(obj);
}

static void pmbus_class_init(ObjectClass *klass, void *data)
{
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = pmbus_realize;
    k->recv = pmbus_recv;
    k->send = pmbus_send;
}

static const TypeInfo pmbus_info = {
    .name          = TYPE_PMBUS,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(PMBUSState),
    .instance_init = pmbus_initfn,
    .class_init    = pmbus_class_init,
};

static void pmbus_register_types(void)
{
    type_register_static(&pmbus_info);
}

type_init(pmbus_register_types)
