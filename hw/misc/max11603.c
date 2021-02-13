/*
 * Maxim max11603 ADC chip emulation.
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
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "qapi/visitor.h"
#include "qapi/error.h"

typedef struct {
    I2CSlave parent_obj;

    uint8_t tb1, rb;

    uint8_t input[8];
    uint32_t inputs;
    int com;
} MAX11603State;

#define TYPE_MAX_11603 "max11603"

#define MAX_11603(obj) \
    OBJECT_CHECK(MAX11603State, (obj), TYPE_MAX_11603)


#define MAX1363_SCAN_MASK           0x60
#define MAX1363_SE_DE_MASK          0x01

#define CHANNEL_NUM(v)	(((v^MAX1363_SE_DE_MASK)^MAX1363_SCAN_MASK)/2)

static uint32_t max11603_read(MAX11603State *s)
{
    if (!s->tb1)
        return 0;

    return s->rb;
}

/* Interpret a control-byte */

static int max11603_write(I2CSlave *i2c, uint8_t value)
{
    MAX11603State *s = MAX_11603(i2c);
    int measure, chan;

    // Ignore the value with no scan mask
    if (!(value & MAX1363_SCAN_MASK))
        return 0;

    s->tb1 = value;

    chan = CHANNEL_NUM(value);

    if (value & MAX1363_SE_DE_MASK)
        measure = s->input[chan] - s->com;
    else
        measure = s->input[chan] - s->input[chan ^ 1];

    s->rb = measure;

    return 0;
}


static int max11603_send(I2CSlave *i2c, uint8_t value)
{
    max11603_write(i2c, value);
    return 0;
}

static uint8_t max11603_recv(I2CSlave *i2c)
{
    MAX11603State *s = MAX_11603(i2c);
    return max11603_read(s);
}

static const VMStateDescription vmstate_max11603 = {
    .name = "max11603",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, MAX11603State),
        VMSTATE_UINT8(tb1, MAX11603State),
        VMSTATE_UINT8(rb, MAX11603State),
        VMSTATE_UINT32_EQUAL(inputs, MAX11603State, NULL),
        VMSTATE_INT32(com, MAX11603State),
        VMSTATE_UINT8_ARRAY(input, MAX11603State, 8),
        VMSTATE_END_OF_LIST()
    }
};

static int max11603_init(I2CSlave *d, uint8_t inputs)
{
    DeviceState *dev = DEVICE(d);
    MAX11603State *s = MAX_11603(dev);

    s->inputs = inputs;
    s->com = 0;

    vmstate_register(VMSTATE_IF(dev), VMSTATE_INSTANCE_ID_ANY,
                     &vmstate_max11603, s);
    return 0;
}

static void max11603_realize(DeviceState *dev, Error **errp)
{
    I2CSlave *i2c = I2C_SLAVE(dev);
    max11603_init(i2c, 8);
}

static void max11603_set_chan(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    Error *local_err = NULL;
    uint8_t* chan_ptr=(uint8_t*)opaque;
    uint8_t value;
    visit_type_uint8(v, name, &value, &local_err);
    *chan_ptr=value;
}
static void max11603_initfn(Object *obj)
{
    MAX11603State* s=MAX_11603(obj);
    object_property_add(obj, "chan1", "uint8_t",
                        NULL,
                        max11603_set_chan, NULL, &(s->input[0]), NULL);
    object_property_add(obj, "chan2", "uint8_t",
                        NULL,
                        max11603_set_chan, NULL, &s->input[1], NULL);
    object_property_add(obj, "chan3", "uint8_t",
                        NULL,
                        max11603_set_chan, NULL, &s->input[2], NULL);
    object_property_add(obj, "chan4", "uint8_t",
                        NULL,
                        max11603_set_chan, NULL, &s->input[3], NULL);
    object_property_add(obj, "chan5", "uint8_t",
                        NULL,
                        max11603_set_chan, NULL, &s->input[4], NULL);
    object_property_add(obj, "chan6", "uint8_t",
                        NULL,
                        max11603_set_chan, NULL, &s->input[5], NULL);
    object_property_add(obj, "chan7", "uint8_t",
                        NULL,
                        max11603_set_chan, NULL, &s->input[6], NULL);
    object_property_add(obj, "chan8", "uint8_t",
                        NULL,
                        max11603_set_chan, NULL, &s->input[7], NULL);

}

static void max11603_class_init(ObjectClass *klass, void *data)
{
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = max11603_realize;
    k->recv = max11603_recv;
    k->send = max11603_send;
}

static const TypeInfo max11603_info = {
    .name          = TYPE_MAX_11603,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(MAX11603State),
    .instance_init = max11603_initfn,
    .class_init    = max11603_class_init,
};

static void max11603_register_types(void)
{
    type_register_static(&max11603_info);
}

type_init(max11603_register_types)
