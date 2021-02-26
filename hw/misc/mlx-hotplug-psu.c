/*
 * QEMU mlx-hotplug-psu
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
#include "pmbus.h"

#define _DEBUG_MLX_HOTPLUG_PSU

#ifdef DEBUG_MLX_HOTPLUG_PSU
#define DPRINTK(FMT, ...) printf(TYPE_MLX_HOTPLUG_PSU ": " FMT, ## __VA_ARGS__)
#else
#define DPRINTK(FMT, ...) do {} while (0)
#endif

#define ERR(FMT, ...) fprintf(stderr, TYPE_MLX_HOTPLUG_PSU " : " FMT, \
                            ## __VA_ARGS__)

typedef struct mlx_hotplug_psu_state {
    DeviceState qdev;
    uint8_t slot;
    DeviceState *dev;

} mlx_hotplug_psu_state;
#define TYPE_MLX_HOTPLUG_PSU "mlx_hotplug_psu"
#define MLX_PSU(obj) OBJECT_CHECK(mlx_hotplug_psu_state, (obj), TYPE_MLX_HOTPLUG_PSU)

static const VMStateDescription vmstate_mlx_hotplug_psu = {
    .name = TYPE_MLX_HOTPLUG_PSU,
};

static Property mlx_hotplug_psu_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

#define MLXREG_SEC1_INT_AGG BIT(2)
#define MLXREG_PSU_PRSNT_INT_L BIT(6)

static void mlx_hotplug_psu_realize(DeviceState *ds, Error **errp)
{
    mlxreg_hotplug_device * hd = MLXREG_HOTPLUG_DEVICE(ds);
    mlx_hotplug_psu_state * psu_hdev = MLX_PSU(ds);
    BusState *bus = qdev_get_parent_bus(DEVICE(ds));
    mlxregState *mlxreg = MLXREG_DEV(bus->parent);
    uint8_t* psu_status;
    uint8_t* psu_event;
    uint8_t* psu_pwr;
    uint8_t* aggr;
    uint8_t* aggrlo;
    psu_status = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_PSU_OFFSET];
    psu_event = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_PSU_EVENT_OFFSET];
    psu_pwr = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_PWR_OFFSET];
    aggr = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_AGGR_OFFSET];
    aggrlo = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_AGGRLO_OFFSET];

    if((hd->slot<1) || (hd->slot>2)) {
        error_setg(errp, "mlxreg_psu: slot %d out of range. \n",
                           hd->slot);
    }

    if(!(*psu_status&BIT(hd->slot-1))) {
        error_setg(errp, "mlxreg_psu: slot %d already occupied cannot be exposed to guest. \n",
                   hd->slot);
    }

    psu_hdev->dev = qdev_create(BUS(mlxreg->i2c_bus[4]), TYPE_PMBUS);
    if(!psu_hdev->dev) {
        error_setg(errp, "mlxreg_psu: slot %d failed to create device. \n",
                                   hd->slot);
    }
    qdev_prop_set_uint8(psu_hdev->dev, "address", 0x5a-hd->slot);
    qdev_init_nofail(psu_hdev->dev);

    *aggr &= ~MLXREG_SEC1_INT_AGG;
    *aggrlo &= ~MLXREG_PSU_PRSNT_INT_L;
    *psu_event |= BIT(hd->slot-1);
    *psu_status &= ~BIT(hd->slot-1);
    *psu_pwr |= BIT(hd->slot-1);
    qemu_irq_pulse(mlxreg->irq);

}

static void mlx_hotplug_psu_unrealize(DeviceState *ds, Error **errp)
{
    mlxreg_hotplug_device * hd = MLXREG_HOTPLUG_DEVICE(ds);
    mlx_hotplug_psu_state * psu_hdev = MLX_PSU(ds);
    BusState *bus = qdev_get_parent_bus(DEVICE(ds));
    mlxregState *mlxreg = MLXREG_DEV(bus->parent);
    uint8_t* psu_status;
    uint8_t* psu_event;
    uint8_t* psu_pwr;
    uint8_t* aggr;
    uint8_t* aggrlo;

    psu_status = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_PSU_OFFSET];
    psu_event = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_PSU_EVENT_OFFSET];
    psu_pwr = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_PWR_OFFSET];
    aggr = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_AGGR_OFFSET];
    aggrlo = &mlxreg->io_regmap_buf[MLXPLAT_CPLD_LPC_REG_AGGRLO_OFFSET];

    *aggr &= ~MLXREG_SEC1_INT_AGG;
    *aggrlo &= ~MLXREG_PSU_PRSNT_INT_L;
    *psu_event |= BIT(hd->slot-1);
    *psu_status |= BIT(hd->slot-1);
    *psu_pwr &= ~BIT(hd->slot-1);
    qemu_irq_pulse(mlxreg->irq);

    if (psu_hdev->dev) {
        object_unparent(OBJECT(psu_hdev->dev));
    }
}

static void mlx_hotplug_psu_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_mlx_hotplug_psu;
    dc->realize = mlx_hotplug_psu_realize;
    dc->unrealize = mlx_hotplug_psu_unrealize;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = "mlxreg_psu";
    dc->hotpluggable = true;
    device_class_set_props(dc, mlx_hotplug_psu_properties);
}

static void mlx_hotplug_psu_initfn(Object *obj)
{
    //mlx_psu_State *d = MLXREG_PSU(obj);
}

static const TypeInfo mlx_hotplug_psu_info = {
    .name = TYPE_MLX_HOTPLUG_PSU,
    .parent = TYPE_MLXREG_HOTPLUG_DEVICE,
    .instance_size = sizeof(mlx_hotplug_psu_state),
    .class_init = mlx_hotplug_psu_class_init,
    .instance_init = mlx_hotplug_psu_initfn,
};

static void mlx_hotplug_psu_register_types(void)
{
    type_register_static(&mlx_hotplug_psu_info);
}

type_init(mlx_hotplug_psu_register_types)
