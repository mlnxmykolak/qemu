/*
 * QEMU mlx-hotplug-fan
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
#include "mlxreg.h"

#define _DEBUG_MLX_HOTPLUG_FAN

#ifdef DEBUG_MLX_HOTPLUG_FAN
#define DPRINTK(FMT, ...) printf(TYPE_MLX_HOTPLUG_FAN ": " FMT, ## __VA_ARGS__)
#else
#define DPRINTK(FMT, ...) do {} while (0)
#endif

#define ERR(FMT, ...) fprintf(stderr, TYPE_MLX_HOTPLUG_FAN " : " FMT, \
                            ## __VA_ARGS__)

typedef struct mlx_hotplug_fan_state {

} mlx_hotplug_fan_state;
#define TYPE_MLX_HOTPLUG_FAN "mlx_hotplug_fan"
#define MLX_FAN(obj) OBJECT_CHECK(mlx_hotplug_fan_state, (obj), TYPE_MLX_HOTPLUG_FAN)

static const VMStateDescription vmstate_mlx_hotplug_fan = {
    .name = TYPE_MLX_HOTPLUG_FAN,
};

static Property mlx_hotplug_fan_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

#define MLXREG_SEC1_INT_AGG BIT(2)
#define MLXREG_FAN_PRSNT_INT_L BIT(6)

static void mlx_hotplug_fan_realize(DeviceState *ds, Error **errp)
{
    mlxreg_hotplug_device * hd = MLXREG_HOTPLUG_DEVICE(ds);
    BusState *bus = qdev_get_parent_bus(DEVICE(ds));
    mlxregState *mlxreg = MLXREG_DEV(bus->parent);
    uint8_t* fan_drw_cap;
    uint8_t* fan_status;
    uint8_t* fan_event;
    uint8_t* aggr;
    uint8_t* aggrlo;
    fan_drw_cap = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_FAN_DRW_CAP_OFFSET];
    fan_status = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_FAN_OFFSET];
    fan_event = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_FAN_EVENT_OFFSET];
    aggr = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_AGGR_OFFSET];
    aggrlo = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_AGGRLO_OFFSET];

    if((hd->slot<1) || !(*fan_drw_cap&BIT(hd->slot-1))) {
        error_setg(errp, "mlxreg_fan: slot %d out of range. \n",
                           hd->slot);
    }

    if(!(*fan_status&BIT(hd->slot-1))) {
        error_setg(errp, "mlxreg_fan: slot %d already occupied cannot be exposed to guest. \n",
                   hd->slot);
    }

    *aggr &= ~MLXREG_SEC1_INT_AGG;
    *aggrlo &= ~MLXREG_FAN_PRSNT_INT_L;
    *fan_event |= BIT(hd->slot-1);
    *fan_status &= ~BIT(hd->slot-1);
    qemu_irq_pulse(mlxreg->irq);
}

static void mlx_hotplug_fan_unrealize(DeviceState *ds, Error **errp)
{
    mlxreg_hotplug_device * hd = MLXREG_HOTPLUG_DEVICE(ds);
    BusState *bus = qdev_get_parent_bus(DEVICE(ds));
    mlxregState *mlxreg = MLXREG_DEV(bus->parent);
    uint8_t* fan_status;
    uint8_t* fan_event;
    uint8_t* aggr;
    uint8_t* aggrlo;
    fan_status = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_FAN_OFFSET];
    fan_event = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_FAN_EVENT_OFFSET];
    aggr = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_AGGR_OFFSET];
    aggrlo = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_AGGRLO_OFFSET];

    *aggr &= ~MLXREG_SEC1_INT_AGG;
    *aggrlo &= ~MLXREG_FAN_PRSNT_INT_L;
    *fan_event |= BIT(hd->slot-1);
    *fan_status |= BIT(hd->slot-1);
    qemu_irq_pulse(mlxreg->irq);
}

static void mlx_hotplug_fan_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_mlx_hotplug_fan;
    dc->realize = mlx_hotplug_fan_realize;
    dc->unrealize = mlx_hotplug_fan_unrealize;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = "mlxreg_fan";
    dc->hotpluggable = true;
    device_class_set_props(dc, mlx_hotplug_fan_properties);
}

static void mlx_hotplug_fan_initfn(Object *obj)
{
    //mlx_fan_State *d = MLXREG_FAN(obj);
}

static const TypeInfo mlx_hotplug_fan_info = {
    .name = TYPE_MLX_HOTPLUG_FAN,
    .parent = TYPE_MLXREG_HOTPLUG_DEVICE,
    .instance_size = sizeof(mlx_hotplug_fan_state),
    .class_init = mlx_hotplug_fan_class_init,
    .instance_init = mlx_hotplug_fan_initfn,
};

static void mlx_hotplug_fan_register_types(void)
{
    type_register_static(&mlx_hotplug_fan_info);
}

type_init(mlx_hotplug_fan_register_types)
