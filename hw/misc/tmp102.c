/*
 * Texas Instruments TMP102 temperature sensor.
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
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "tmp102.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/module.h"

#define _DEBUG_TMP102

#ifdef DEBUG_TMP102
#define DPRINTK(FMT, ...) printf(TYPE_TMP102 ": " FMT, ## __VA_ARGS__)
#else
#define DPRINTK(FMT, ...) do {} while (0)
#endif

/* convert left adjusted 13-bit TMP102 register value to milliCelsius */
static inline int tmp102_reg_to_mC(uint16_t val)
{
    return ((val & ~0x01) * 1000) / 128;
}

/* convert milliCelsius to left adjusted 13-bit TMP102 register value */
static inline uint16_t tmp102_mC_to_reg(int val)
{
    return (val * 128) / 1000;
}

static void tmp102_interrupt_update(TMP102State *s)
{
    qemu_set_irq(s->pin, s->alarm ^ ((~s->config[0] >> 2) & 1));	/* POL */
}

static void tmp102_alarm_update(TMP102State *s)
{
    if ((s->config[0] >> 0) & 1) {					/* SD */
        if ((s->config[0] >> 7) & 1)				/* OS */
            s->config[0] &= ~(1 << 7);				/* OS */
        else
            return;
    }

    if ((s->config[0] >> 1) & 1) {					/* TM */
        if (s->temperature >= s->limit[1])
            s->alarm = 1;
        else if (s->temperature < s->limit[0])
            s->alarm = 1;
    } else {
        if (s->temperature >= s->limit[1])
            s->alarm = 1;
        else if (s->temperature < s->limit[0])
            s->alarm = 0;
    }

    tmp102_interrupt_update(s);
}

static void tmp102_get(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    TMP102State *s = TMP102(obj);
    int64_t value=0;
    if (strcmp(name, "temperature") == 0) {
        value = s->temperature * 1000 / 128;
    } else if (strcmp(name, "limit_high") == 0) {
        value = s->limit[0] * 1000 / 128;
    }
    else if (strcmp(name, "limit_hyst") == 0) {
        value = s->limit[1] * 1000 / 128;
    }

    visit_type_int(v, name, &value, errp);
}


static void tmp102_set(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    TMP102State *s = TMP102(obj);
    Error *local_err = NULL;
    int64_t temp;

    visit_type_int(v, name, &temp, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }
    if (temp >= 128000 || temp < -128000) {
        error_setg(errp, "value %" PRId64 ".%03" PRIu64 " C is out of range",
                   temp / 1000, temp % 1000);
        return;
    }
    if (strcmp(name, "temperature") == 0) {
        s->temperature = (int16_t) (temp * 128 / 1000);
    } else if (strcmp(name, "limit_high") == 0) {
        s->limit[0] = (int16_t) (temp * 128 / 1000);
    }
    else if (strcmp(name, "limit_hyst") == 0) {
        s->limit[1] = (int16_t) (temp * 128 / 1000);
    }

    tmp102_alarm_update(s);
}

static const int tmp102_faultq[4] = { 1, 2, 4, 6 };

static void tmp102_read(TMP102State *s)
{
    s->len = 0;

    if ((s->config[0] >> 1) & 1) {					/* TM */
        s->alarm = 0;
        tmp102_interrupt_update(s);
    }

    switch (s->pointer & 3) {
    case TMP102_REG_TEMPERATURE:
        s->buf[s->len ++] = (((uint16_t) s->temperature) >> 8);
        s->buf[s->len ++] = (((uint16_t) s->temperature) >> 0) &
                (0xf0 << ((~s->config[0] >> 5) & 3));		/* R */
        break;

    case TMP102_REG_CONFIG:
        DPRINTK("config: %x\n", *((uint16_t*)s->config));
        s->buf[s->len ++] = s->config[0];
        s->buf[s->len ++] = s->config[1];
        break;

    case TMP102_REG_T_LOW:
        s->buf[s->len ++] = ((uint16_t) s->limit[0]) >> 8;
        s->buf[s->len ++] = ((uint16_t) s->limit[0]) >> 0;
        break;

    case TMP102_REG_T_HIGH:
        s->buf[s->len ++] = ((uint16_t) s->limit[1]) >> 8;
        s->buf[s->len ++] = ((uint16_t) s->limit[1]) >> 0;
        break;
    }
}

static void tmp102_write(TMP102State *s)
{
    switch (s->pointer & 3) {
    case TMP102_REG_TEMPERATURE:
        break;

    case TMP102_REG_CONFIG:
        if (s->buf[0] & ~s->config[0] & (1 << 0))			/* SD */
            DPRINTK("%s: TMP102 shutdown\n", __func__);
        s->config[0] = s->buf[0];
        s->faults = tmp102_faultq[(s->config[0] >> 3) & 3];	/* F */
        tmp102_alarm_update(s);
        break;

    case TMP102_REG_T_LOW:
    case TMP102_REG_T_HIGH:
        if (s->len >= 3)
            s->limit[s->pointer & 1] = (int16_t)
                    ((((uint16_t) s->buf[0]) << 8) | s->buf[1]);
        tmp102_alarm_update(s);
        break;
    }
}

static uint8_t tmp102_rx(I2CSlave *i2c)
{
    TMP102State *s = TMP102(i2c);

    if (s->len < 2) {
        DPRINTK("tmp102_rx:0x%x\n",s->buf[s->len]);
        return s->buf[s->len ++];
    } else {
        return 0xff;
    }
}

static int tmp102_tx(I2CSlave *i2c, uint8_t data)
{
    TMP102State *s = TMP102(i2c);

    if (s->len == 0) {
        s->pointer = data;
        s->len++;
    } else {
        if (s->len <= 2) {
            s->buf[s->len - 1] = data;
        }
        s->len++;
        tmp102_write(s);
    }

    return 0;
}

static int tmp102_event(I2CSlave *i2c, enum i2c_event event)
{
    TMP102State *s = TMP102(i2c);

    if (event == I2C_START_RECV) {
        tmp102_read(s);
    }

    s->len = 0;
    return 0;
}

static int tmp102_post_load(void *opaque, int version_id)
{
    TMP102State *s = opaque;

    s->faults = tmp102_faultq[(s->config[0] >> 3) & 3];		/* F */

    tmp102_interrupt_update(s);
    return 0;
}

static const VMStateDescription vmstate_tmp102 = {
    .name = "TMP102",
    .version_id = 0,
    .minimum_version_id = 0,
    .post_load = tmp102_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(len, TMP102State),
        VMSTATE_UINT8_ARRAY(buf, TMP102State, 2),
        VMSTATE_UINT8(pointer, TMP102State),
        VMSTATE_UINT8_ARRAY(config, TMP102State, 2),
        VMSTATE_INT16(temperature, TMP102State),
        VMSTATE_INT16_ARRAY(limit, TMP102State, 2),
        VMSTATE_UINT8(alarm, TMP102State),
        VMSTATE_I2C_SLAVE(i2c, TMP102State),
        VMSTATE_END_OF_LIST()
    }
};

static void tmp102_reset(I2CSlave *i2c)
{
    TMP102State *s = TMP102(i2c);

    s->pointer = 0;
    s->config[0] = 0x60; /* R0 R1*/
    s->config[1] = 0;
    s->faults = tmp102_faultq[(s->config[0] >> 3) & 3];
    s->alarm = 0;

    tmp102_interrupt_update(s);
}

static void tmp102_realize(DeviceState *dev, Error **errp)
{
    I2CSlave *i2c = I2C_SLAVE(dev);
    TMP102State *s = TMP102(i2c);

    qdev_init_gpio_out(&i2c->qdev, &s->pin, 1);

    tmp102_reset(&s->i2c);
}

static void tmp102_initfn(Object *obj)
{
    object_property_add(obj, "temperature", "int",
                        tmp102_get,
                        tmp102_set, NULL, NULL, NULL);
    object_property_add(obj, "limit_high", "int",
                        tmp102_get,
                        tmp102_set, NULL, NULL, NULL);
    object_property_add(obj, "limit_hyst", "int",
                        tmp102_get,
                        tmp102_set, NULL, NULL, NULL);
}

static void tmp102_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = tmp102_realize;
    k->event = tmp102_event;
    k->recv = tmp102_rx;
    k->send = tmp102_tx;
    dc->vmsd = &vmstate_tmp102;
}

static const TypeInfo tmp102_info = {
    .name          = TYPE_TMP102,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(TMP102State),
    .instance_init = tmp102_initfn,
    .class_init    = tmp102_class_init,
};

static void tmp102_register_types(void)
{
    type_register_static(&tmp102_info);
}

type_init(tmp102_register_types)
