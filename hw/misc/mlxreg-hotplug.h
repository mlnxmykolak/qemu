/*
 * QEMU mlxreg-hotplug
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

#ifndef MLXREG_HOTPLUG_H
#define MLXREG_HOTPLUG_H

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
#include "hw/irq.h"
#include "monitor/qdev.h"
#include "qapi/qmp/qdict.h"
#include "qapi/qmp/qstring.h"

struct mlxreg_hotplug {
    BusState qbus;
};

typedef struct mlxreg_hotplug mlxreg_hotplug;

typedef struct mlxreg_hotplug_class {
    /*< private >*/
    BusClass parent_class;
    /*< public >*/

    int (*bus_num)(mlxreg_hotplug *bus);
} mlxreg_hotplug_class;

#define TYPE_MLXREG_HOTPLUG "mlxreg-hotplug"
#define MLXREG_HOTPLUG(obj) OBJECT_CHECK(mlxreg_hotplug, (obj), TYPE_MLXREG_HOTPLUG)
#define MLXREG_HOTPLUG_CLASS(klass) OBJECT_CLASS_CHECK(mlxreg_hotplug_class, (klass), TYPE_MLXREG_HOTPLUG)
#define MLXREG_HOTPLUG_GET_CLASS(obj) OBJECT_GET_CLASS(mlxreg_hotplug_class, (obj), TYPE_MLXREG_HOTPLUG)

mlxreg_hotplug *mlxreg_create_bus(DeviceState *parent, const char *name);

struct mlxreg_hotplug_device {
    DeviceState qdev;
    uint8_t slot;
};

typedef struct mlxreg_hotplug_device mlxreg_hotplug_device;
#define TYPE_MLXREG_HOTPLUG_DEVICE "mlxreg-device"
#define MLXREG_HOTPLUG_DEVICE(obj) \
     OBJECT_CHECK(mlxreg_hotplug_device, (obj), TYPE_MLXREG_HOTPLUG_DEVICE)
#define MLXREG_HOTPLUG_DEVICE_CLASS(klass) \
     OBJECT_CLASS_CHECK(mlxreg_hotplug_device_class, (klass), TYPE_MLXREG_HOTPLUG_DEVICE)
#define MLXREG_HOTPLUG_DEVICE_GET_CLASS(obj) \
     OBJECT_GET_CLASS(mlxreg_hotplug_device_class, (obj), TYPE_MLXREG_HOTPLUG_DEVICE)

typedef struct mlxreg_hotplug_device_class {
    DeviceClass parent_class;

    void (*realize)(mlxreg_hotplug_device *dev, Error **errp);
} mlxreg_hotplug_device_class;


#endif
