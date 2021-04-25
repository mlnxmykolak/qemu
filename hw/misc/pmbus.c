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

/**
 * DSCR: Converts a LinearFloat11 formatted word into float value
 * RECV: data
 * RTRN: converted data
 */
float _linear11_to_float (u_int16_t data) {
    int8_t  exponent = data >> 11;                  /* extract exponent as MS 5 bits */
    int16_t mantissa = data & 0x7ff;                /* extract mantissa as LS 11 bits */
    if( exponent > 0x0F )   exponent |= 0xE0;       /* sign extend exponent from 5 to 8 bits */
    if( mantissa > 0x03FF ) mantissa |= 0xF800;     /* sign extend mantissa from 11 to 16 bits */
    return mantissa * pow(2, exponent);             /* compute value as mantissa * 2^(exponent) */
}


/**
 * DSCR: Converts a float value into a LinearFloat11 formatted word
 * RECV: data
 * RTRN: converted data
 */
u_int16_t _float_to_linear11 (float data) {
    int exponent = -16;                                 /* set exponent to -16 */
    int mantissa = (int)(data / pow(2.0, exponent));    /* extract mantissa from input value */

    /* Search for an exponent that produces a valid 11-bit mantissa */
    do {
        if ((mantissa >= -1024) && (mantissa <= +1023)) {
            break;  /* stop if mantissa valid */
        }
        exponent++;
        mantissa = (int)(data / pow(2.0, exponent));
    } while (exponent < +15);

    /* Format the exponent of the L11 */
    u_int16_t uExponent = exponent << 11;

    /* Format the mantissa of the L11 uint16 uMantissa = mantissa & 0x07FF; */
    u_int16_t uMantissa = mantissa & 0x07FF;

    /* Compute value as exponent | mantissa return uExponent | uMantissa;   */
    return uExponent | uMantissa;

    return (u_int32_t)(data / pow(2, L16_EXPONENT));
}

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

    for (i = 0; i < count; i++)
        crc = crc8((crc ^ p[i]) << 8);
    //printf ("crc: 0x%x\n",crc);
    return crc;
}

#define I2C_WR_BIT 0x01
static int pmbus_recv(I2CSlave *i2c, uint8_t value)
{
    PMBUSState *s = PMBUS(i2c);
    //printf("pmbus_rcv value:0x%x\n", value);
    s->ret_len=0;
    s->cmd_len=0;

    s->cmd=value;
    s->cmd_len=s->pmbus_regmap_external[s->page][s->cmd][0];
    // If len > 2 its pmbus block and need len to be included to first byte.
    if(s->cmd_len>2) {
        s->cmd_len++;
    }

    switch(s->cmd) {
        case PMBUS_PAGE:
             if (s->rcv_len == 1) {
                 s->page=value;
             }
            break;
        default:
            break;
    }
    s->rcv_len++;
    return 0;
}

static uint8_t pmbus_send(I2CSlave *i2c)
{
    uint8_t ret=0;
    PMBUSState *s = PMBUS(i2c);
    s->rcv_len=0;
    if (s->ret_len == 0 ) {
        uint8_t i2c_addr=(i2c->address<<1);
        /* Add partial PEC. Check PEC if last message is a write */
        s->crc_pec=i2c_smbus_pec(0, &i2c_addr, 1);
        s->crc_pec=i2c_smbus_pec(s->crc_pec, &s->cmd, 1);
        i2c_addr=(i2c->address<<1)|I2C_WR_BIT;
        s->crc_pec=i2c_smbus_pec(s->crc_pec, &i2c_addr, 1);
    }
    if((s->ret_len == s->cmd_len) && (s->cmd_len)) {
        ret=s->crc_pec;
    }
    else
    {
        if(s->cmd_len>2)
        {
            // If len > 2 its pmbus block and need len to be included to first byte.
            ret=s->pmbus_regmap_external[s->page][s->cmd][s->ret_len];
        }
        else
        {
            ret=s->pmbus_regmap_external[s->page][s->cmd][s->ret_len+1];
        }
        s->crc_pec=i2c_smbus_pec(s->crc_pec, &ret, 1);
    }
    printf("pmbus_send cmd:0x%x(%d), %d: 0x%x\n",s->cmd,s->cmd,s->ret_len, ret);
    s->ret_len++;
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

    return 0;
}

static void pmbus_realize(DeviceState *dev, Error **errp)
{
    I2CSlave *i2c = I2C_SLAVE(dev);
    PMBUSState *s = PMBUS(dev);
    //Error *local_err = NULL;

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


    pmbus_init(i2c);
}


static void pmbus_initfn(Object *obj)
{
//    PMBUSState* s=PMBUS(obj);
}

static Property pmbus_class_properties[] = {
    DEFINE_PROP_DRIVE("drive", PMBUSState, blk),
    DEFINE_PROP_END_OF_LIST(),
};

static void pmbus_class_init(ObjectClass *klass, void *data)
{
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = pmbus_realize;
    k->recv = pmbus_send;
    k->send = pmbus_recv;
    device_class_set_props(dc, pmbus_class_properties);
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
