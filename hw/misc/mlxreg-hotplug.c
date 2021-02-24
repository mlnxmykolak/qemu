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


#include "mlxreg-hotplug.h"

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
    //mlxregState *d = MLXREG_DEV(hotplug->parent);
}

static void mlxreg_hotplug_class_init(ObjectClass *klass, void *data)
{
    BusClass *k = BUS_CLASS(klass);
    //mlxreg_hotplug_class *pbc = MLXREG_HOTPLUG_CLASS(klass);

    k->realize = mlxreg_hotplug_realize;
    //k->unrealize = mlxreg_hotplug_unrealize;
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
