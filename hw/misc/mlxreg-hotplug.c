/*
 * QEMU mlxreg-hotplug
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
#include "net/checksum.h"
#include "qemu/module.h"
#include "qemu/range.h"
#include "qapi/qmp/qerror.h"
#include "qapi/visitor.h"
#include "qapi/error.h"
#include "sysemu/block-backend.h"
#include "mlxreg-hotplug.h"
#include "mlxreg.h"
#include "hw/irq.h"

#define _DEBUG_MLXREG_HOTPLUG

#ifdef DEBUG_MLXREG_HOTPLUG
#define DPRINTK(FMT, ...) printf(TYPE_MLXREG_HOTPLUG ": " FMT, ## __VA_ARGS__)
#else
#define DPRINTK(FMT, ...) do {} while (0)
#endif

#define ERR(FMT, ...) fprintf(stderr, TYPE_MLXREG_HOTPLUG " : " FMT, \
                            ## __VA_ARGS__)

static void mlxreg_hotplug_realize(BusState *hotplug, Error **errp)
{
    //mlxregState *d = PCI_MLXREG_DEV(hotplug->parent);
}

static void mlxreg_hotplug_class_init(ObjectClass *klass, void *data)
{
    BusClass *k = BUS_CLASS(klass);
    //mlxreg_hotplug_class *pbc = MLXREG_HOTPLUG_CLASS(klass);

    k->realize = mlxreg_hotplug_realize;
    //k->unrealize = mlxreg_hotplug_unrealize;
    //k->reset = pcibus_reset;
}

static void mlxreg_hotplug_initfn(Object *obj)
{

}

static const TypeInfo mlxreg_hotplug_info = {
    .name = TYPE_MLXREG_HOTPLUG,
    .parent = TYPE_BUS,
    .instance_size = sizeof(mlxreg_hotplug),
    .class_size = sizeof(mlxreg_hotplug_class),
    .class_init = mlxreg_hotplug_class_init,
    .instance_init = mlxreg_hotplug_initfn,
    .interfaces = (InterfaceInfo[]) {
        { TYPE_HOTPLUG_HANDLER },
        { }
    }
};

static const VMStateDescription vmstate_mlxreg_hotplug = {
    .name = "mlxreg_hotplug",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};


mlxreg_hotplug *mlxreg_create_bus(DeviceState *parent, const char *name)
{
    mlxreg_hotplug *bus;
    Error *error_abort;

    bus = MLXREG_HOTPLUG(qbus_create(TYPE_MLXREG_HOTPLUG, parent, name));
    qbus_set_bus_hotplug_handler(BUS(bus), &error_abort);
    //QLIST_INIT(&bus->current_devs);
    vmstate_register(NULL, VMSTATE_INSTANCE_ID_ANY, &vmstate_mlxreg_hotplug, bus);
    return bus;
}


static void mlxreg_hotplug_device_class_base_init(ObjectClass *klass, void *data)
{

}

static Property mlxreg_hotplug_device_props[] = {
    DEFINE_PROP_UINT8("slot", mlxreg_hotplug_device, slot, 0),
    DEFINE_PROP_END_OF_LIST()
};

static void mlxreg_hotplug_device_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *k = DEVICE_CLASS(klass);

    //k->realize = mlxreg_hotplug_device_hotplug_realize;
    //k->unrealize = mlxreg_qdev_unrealize;
    k->bus_type = TYPE_MLXREG_HOTPLUG;
    device_class_set_props(k, mlxreg_hotplug_device_props);
}

static const TypeInfo mlxreg_hotplug_device_type_info = {
    .name = TYPE_MLXREG_HOTPLUG_DEVICE,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(mlxreg_hotplug_device),
    .abstract = true,
    .class_size = sizeof(mlxreg_hotplug_device_class),
    .class_init = mlxreg_hotplug_device_class_init,
    .class_base_init = mlxreg_hotplug_device_class_base_init,
};

static void mlxreg_register_types(void)
{
    type_register_static(&mlxreg_hotplug_info);
    type_register_static(&mlxreg_hotplug_device_type_info);
}

type_init(mlxreg_register_types)


typedef struct mlxreg_fan_State {

} mlxreg_fan_State;
#define TYPE_MLXREG_FAN "mlx_hotplug_fan"
#define MLXREG_FAN(obj) OBJECT_CHECK(mlxreg_fan_State, (obj), TYPE_MLXREG_FAN)

static const VMStateDescription vmstate_mlxreg_fan = {
    .name = TYPE_MLXREG_FAN,
};

static Property mlxreg_fan_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

#define MLXREG_SEC1_INT_AGG BIT(2)
#define MLXREG_FAN_PRSNT_INT_L BIT(6)

static void mlxreg_fan_realize(DeviceState *ds, Error **errp)
{
    mlxreg_hotplug_device * hd = MLXREG_HOTPLUG_DEVICE(ds);
    BusState *bus = qdev_get_parent_bus(DEVICE(ds));
    mlxregState *mlxreg = PCI_MLXREG_DEV(bus->parent);
    uint8_t* fan_drw_cap;
    uint8_t* fan_status;
    uint8_t* aggr;
    uint8_t* aggrlo;
    fan_drw_cap = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_FAN_DRW_CAP_OFFSET];
    fan_status = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_FAN_OFFSET];
    aggr = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_AGGR_OFFSET];
    aggrlo = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_AGGRLO_OFFSET];

    if((hd->slot<1) || !(*fan_drw_cap&BIT(hd->slot-1))) {
        error_setg(errp, "mlxreg_fan: slot %d out of range. \n",
                           hd->slot);
    }

    if(*fan_status&BIT(hd->slot-1)) {
        error_setg(errp, "mlxreg_fan: slot %d already occupied cannot be exposed to guest. \n",
                   hd->slot);
    }
    *fan_status |= BIT(hd->slot-1);
    //printf("MLXPLAT_CPLD_LPC_REG_FAN_CAP_OFFSET %x %x \n",mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_FAN_CAP1_OFFSET], mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_FAN_CAP2_OFFSET]);
    printf("fan drw cap: %x \n",*fan_drw_cap);
    printf("fan status: %x \n",*fan_status);
    printf("slot: %d\n", hd->slot);

    *aggr &= ~MLXREG_SEC1_INT_AGG;
    *aggrlo &= ~MLXREG_FAN_PRSNT_INT_L;
    qemu_irq_pulse(mlxreg->irq);

}

static void mlxreg_fan_unrealize(DeviceState *ds, Error **errp)
{
    mlxreg_hotplug_device * hd = MLXREG_HOTPLUG_DEVICE(ds);
    BusState *bus = qdev_get_parent_bus(DEVICE(ds));
    mlxregState *mlxreg = PCI_MLXREG_DEV(bus->parent);
    uint8_t* fan_status = 0;
    fan_status = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_FAN_OFFSET];

    *fan_status &= ~BIT(hd->slot-1);
    mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_AGGR_OFFSET]&=~MLXREG_SEC1_INT_AGG;
    mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_AGGRLO_OFFSET]&=~MLXREG_FAN_PRSNT_INT_L;
    qemu_irq_pulse(mlxreg->irq);
}

static void mlxreg_fan_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_mlxreg_fan;
    dc->realize = mlxreg_fan_realize;
    dc->unrealize = mlxreg_fan_unrealize;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = "mlxreg_fan";
    dc->hotpluggable = true;
    device_class_set_props(dc, mlxreg_fan_properties);
}

static void mlxreg_fan_initfn(Object *obj)
{
    //mlxreg_fan_State *d = MLXREG_FAN(obj);
}

static const TypeInfo mlxreg_fan_info = {
    .name = TYPE_MLXREG_FAN,
    .parent = TYPE_MLXREG_HOTPLUG_DEVICE,
    .instance_size = sizeof(mlxreg_fan_State),
    .class_init = mlxreg_fan_class_init,
    .instance_init = mlxreg_fan_initfn,
};

static void mlxreg_fan_register_types(void)
{
    type_register_static(&mlxreg_fan_info);
}

type_init(mlxreg_fan_register_types)
