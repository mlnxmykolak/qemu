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
    //printf("pmbus_send value:0x%x\n", value);
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
        default:
            s->ret_len=0;
            s->cmd_len=s->pmbus_regmap_field_len[s->cmd];
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
            if(s->pmbus_regmap[s->cmd]) {
                ret=s->pmbus_regmap[s->cmd][s->ret_len];
            }
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
    //printf("pmbus_recv 0x%x\n", ret);
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


uint8_t mfr_id[10] = { 0x09, 0x4d, 0x75, 0x72, 0x61, 0x74, 0x61, 0x2d, 0x50, 0x53 };
uint8_t vpd_pointer[17] = { 0x10, 0xeb, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe9, 0xea, 0xec, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01 };
uint8_t vpd_crc[2] = { 0x13, 0xc3 };
uint8_t vpd_pn[31] = { 0x1e, 0x4d, 0x54, 0x45, 0x46, 0x2d, 0x50, 0x53, 0x52, 0x2d, 0x41, 0x43, 0x2d, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };
uint8_t vpd_sn[25] = { 0x18, 0x4d, 0x54, 0x32, 0x30, 0x32, 0x37, 0x58, 0x32, 0x31, 0x30, 0x30, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t vpd_rev[9] = { 0x08, 0x41, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t vpd_mfg[9] = { 0x08, 0x30, 0x33, 0x2f, 0x30, 0x37, 0x2f, 0x32, 0x30 };
uint8_t vpd_cap[9] = { 0x08, 0xdc, 0x05, 0x00, 0x00, 0x6c, 0x6b, 0x01, 0x00 };
uint8_t vpd_rsrv[9] = { 0x08, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
//uint8_t vpd_wp[2];
//uint8_t vpd_ver;


static int pmbus_init(I2CSlave *d)
{
    DeviceState *dev = DEVICE(d);
    PMBUSState *s = PMBUS(dev);

    vmstate_register(VMSTATE_IF(dev), VMSTATE_INSTANCE_ID_ANY,
                     &vmstate_pmbus, s);
    s->pmbus_page = 0xff;
    s->pmbus_status_byte = 0x00;
    *((uint16_t*)&s->pmbus_status_word) = 0x00;
    s->pmbus_status_vout = 0x0;
    s->pmbus_status_iout = 0x0;
    s->pmbus_status_input = 0x0;
    s->pmbus_status_temperature = 0x0;
    s->pmbus_status_fan12 = 0x08;
    s->pmbus_status_fan34 = 0xff;
    s->pmbus_capability = 0x10;
    *((uint16_t*)&s->pmbus_vout_mode) =  0xc11a;
    s->pmbus_regmap[PMBUS_PAGE] = &s->pmbus_page;
    s->pmbus_regmap[PMBUS_STATUS_BYTE] = &s->pmbus_status_byte;
    s->pmbus_regmap[PMBUS_STATUS_WORD] = s->pmbus_status_word;
    s->pmbus_regmap[PMBUS_STATUS_VOUT] = &s->pmbus_status_vout;
    s->pmbus_regmap[PMBUS_STATUS_IOUT] = &s->pmbus_status_iout;
    s->pmbus_regmap[PMBUS_STATUS_INPUT] = &s->pmbus_status_input;
    s->pmbus_regmap[PMBUS_STATUS_TEMPERATURE] = &s->pmbus_status_temperature;
    s->pmbus_regmap[PMBUS_STATUS_FAN_12] = &s->pmbus_status_fan12;
    s->pmbus_regmap[PMBUS_STATUS_FAN_34] = &s->pmbus_status_fan34;
    s->pmbus_regmap[PMBUS_CAPABILITY] = &s->pmbus_capability;
    s->pmbus_regmap[PMBUS_VOUT_MODE] = s->pmbus_vout_mode;
    s->pmbus_regmap_field_len[PMBUS_PAGE] = 1;
    s->pmbus_regmap_field_len[PMBUS_STATUS_BYTE] = 1;
    s->pmbus_regmap_field_len[PMBUS_STATUS_WORD] = 2;
    s->pmbus_regmap_field_len[PMBUS_STATUS_VOUT] = 1;
    s->pmbus_regmap_field_len[PMBUS_STATUS_IOUT] = 1;
    s->pmbus_regmap_field_len[PMBUS_STATUS_INPUT] = 1;
    s->pmbus_regmap_field_len[PMBUS_STATUS_TEMPERATURE] = 1;
    s->pmbus_regmap_field_len[PMBUS_STATUS_FAN_12] = 1;
    s->pmbus_regmap_field_len[PMBUS_STATUS_FAN_34] = 1;
    s->pmbus_regmap_field_len[PMBUS_CAPABILITY] = 1;
    s->pmbus_regmap_field_len[PMBUS_VOUT_MODE] = 1;


    *((uint16_t*)&s->pmbus_read_vin) = 0xf393;
    *((uint16_t*)&s->pmbus_read_iin) = 0xb293;
    *((uint16_t*)&s->pmbus_read_vcap) = 0xfb18;
    *((uint16_t*)&s->pmbus_read_vout) = 0x0302;
    *((uint16_t*)&s->pmbus_read_iout) = 0xd292;
    *((uint16_t*)&s->pmbus_read_temp1) = 0xe25c;
    *((uint16_t*)&s->pmbus_read_temp2) = 0xe25c;
    *((uint16_t*)&s->pmbus_read_temp3) = 0xe25c;
    *((uint16_t*)&s->pmbus_read_fan_speed1) = 0x237b;
    *((uint16_t*)&s->pmbus_read_fan_speed2) = 0x237b;
    *((uint16_t*)&s->pmbus_read_fan_speed3) = 0x237b;
    *((uint16_t*)&s->pmbus_read_fan_speed4) = 0x237b;
    *((uint16_t*)&s->pmbus_read_pout) = 0xebc4;
    *((uint16_t*)&s->pmbus_read_pin) = 0xf227;

    s->pmbus_regmap[PMBUS_READ_VIN] = s->pmbus_read_vin;
    s->pmbus_regmap[PMBUS_READ_IIN] = s->pmbus_read_iin;
    s->pmbus_regmap[PMBUS_READ_VCAP] = s->pmbus_read_vcap;
    s->pmbus_regmap[PMBUS_READ_VOUT] = s->pmbus_read_vout;
    s->pmbus_regmap[PMBUS_READ_IOUT] = s->pmbus_read_iout;
    s->pmbus_regmap[PMBUS_READ_TEMPERATURE_1] = s->pmbus_read_temp1;
    s->pmbus_regmap[PMBUS_READ_TEMPERATURE_2] = s->pmbus_read_temp2;
    s->pmbus_regmap[PMBUS_READ_TEMPERATURE_3] = s->pmbus_read_temp3;
    s->pmbus_regmap[PMBUS_READ_FAN_SPEED_1] = s->pmbus_read_fan_speed1;
    s->pmbus_regmap[PMBUS_READ_FAN_SPEED_2] = s->pmbus_read_fan_speed2;
    s->pmbus_regmap[PMBUS_READ_FAN_SPEED_3] = s->pmbus_read_fan_speed3;
    s->pmbus_regmap[PMBUS_READ_FAN_SPEED_4] = s->pmbus_read_fan_speed4;
    s->pmbus_regmap[PMBUS_READ_POUT] = s->pmbus_read_pout;
    s->pmbus_regmap[PMBUS_READ_PIN] = s->pmbus_read_pin;

    s->pmbus_regmap_field_len[PMBUS_READ_VIN] = 2;
    s->pmbus_regmap_field_len[PMBUS_READ_IIN] = 2;
    s->pmbus_regmap_field_len[PMBUS_READ_VCAP] = 2;
    s->pmbus_regmap_field_len[PMBUS_READ_VOUT] = 2;
    s->pmbus_regmap_field_len[PMBUS_READ_IOUT] = 2;
    s->pmbus_regmap_field_len[PMBUS_READ_TEMPERATURE_1] = 2;
    s->pmbus_regmap_field_len[PMBUS_READ_TEMPERATURE_2] = 2;
    s->pmbus_regmap_field_len[PMBUS_READ_TEMPERATURE_3] = 2;
    s->pmbus_regmap_field_len[PMBUS_READ_FAN_SPEED_1] = 2;
    s->pmbus_regmap_field_len[PMBUS_READ_FAN_SPEED_2] = 2;
    s->pmbus_regmap_field_len[PMBUS_READ_FAN_SPEED_3] = 2;
    s->pmbus_regmap_field_len[PMBUS_READ_FAN_SPEED_4] = 2;
    s->pmbus_regmap_field_len[PMBUS_READ_POUT] = 2;
    s->pmbus_regmap_field_len[PMBUS_READ_PIN] = 2;

    *((uint16_t*)&s->pmbus_ot_fault_limit) = 0xea08;
    s->pmbus_ot_fault_response = 0x50;
    *((uint16_t*)&s->pmbus_ot_warn_limit) = 0xe3c0;
    *((uint16_t*)&s->pmbus_ut_warn_limit) = 0xffff;
    *((uint16_t*)&s->pmbus_ut_fault_limit) = 0xdd80;

    s->pmbus_regmap[PMBUS_OT_FAULT_LIMIT] = s->pmbus_ot_fault_limit;
    s->pmbus_regmap[PMBUS_OT_FAULT_RESPONSE] = &s->pmbus_ot_fault_response;
    s->pmbus_regmap[PMBUS_OT_WARN_LIMIT] = s->pmbus_ot_warn_limit;
    s->pmbus_regmap[PMBUS_UT_WARN_LIMIT] = s->pmbus_ut_warn_limit;
    s->pmbus_regmap[PMBUS_UT_FAULT_LIMIT] = s->pmbus_ut_fault_limit;

    s->pmbus_regmap_field_len[PMBUS_OT_FAULT_LIMIT] = 2;
    s->pmbus_regmap_field_len[PMBUS_OT_FAULT_RESPONSE] = 1;
    s->pmbus_regmap_field_len[PMBUS_OT_WARN_LIMIT] = 2;
    s->pmbus_regmap_field_len[PMBUS_UT_WARN_LIMIT] = 2;
    s->pmbus_regmap_field_len[PMBUS_UT_FAULT_LIMIT] = 2;

    *((uint16_t*)&s->pmbus_vin_ov_fault_limit) = 0xfa26;
    //*((uint16_t*)&s->pmbus_vin_ov_fault_response)= 0x56;
    *((uint16_t*)&s->pmbus_vin_ov_warn_limit)= 0xfa1c;
    *((uint16_t*)&s->pmbus_vin_uv_warn_limit)= 0xf8a0;
    *((uint16_t*)&s->pmbus_vin_uv_fault_limit)= 0xf892;

    s->pmbus_regmap[PMBUS_VIN_OV_FAULT_LIMIT] = s->pmbus_vin_ov_fault_limit;
    //s->pmbus_regmap[PMBUS_OT_FAULT_RESPONSE] = &s->pmbus_vin_ov_fault_response;
    s->pmbus_regmap[PMBUS_VIN_OV_WARN_LIMIT] = s->pmbus_vin_ov_warn_limit;
    s->pmbus_regmap[PMBUS_VIN_UV_WARN_LIMIT] = s->pmbus_vin_uv_warn_limit;
    s->pmbus_regmap[PMBUS_VIN_UV_FAULT_LIMIT] = s->pmbus_vin_uv_fault_limit;

    s->pmbus_regmap_field_len[PMBUS_VIN_OV_FAULT_LIMIT] = 2;
    //s->pmbus_regmap_field_len[PMBUS_OT_FAULT_RESPONSE] = 1;
    s->pmbus_regmap_field_len[PMBUS_VIN_OV_WARN_LIMIT] = 2;
    s->pmbus_regmap_field_len[PMBUS_VIN_UV_WARN_LIMIT] = 2;
    s->pmbus_regmap_field_len[PMBUS_VIN_UV_FAULT_LIMIT] = 2;

    *((uint16_t*)&s->pmbus_vout_ov_fault_limit) = 0x0380;
    *((uint16_t*)&s->pmbus_vout_ov_warn_limit)= 0x0346;
    *((uint16_t*)&s->pmbus_vout_uv_warn_limit)= 0x02ba;
    *((uint16_t*)&s->pmbus_vout_uv_fault_limit)= 0x02da;

    s->pmbus_regmap[PMBUS_VOUT_OV_FAULT_LIMIT] = s->pmbus_vout_ov_fault_limit;
    s->pmbus_regmap[PMBUS_VOUT_OV_WARN_LIMIT] = s->pmbus_vout_ov_warn_limit;
    s->pmbus_regmap[PMBUS_VOUT_UV_WARN_LIMIT] = s->pmbus_vout_uv_warn_limit;
    s->pmbus_regmap[PMBUS_VOUT_UV_FAULT_LIMIT] = s->pmbus_vout_uv_fault_limit;

    s->pmbus_regmap_field_len[PMBUS_VOUT_OV_FAULT_LIMIT] = 2;
    s->pmbus_regmap_field_len[PMBUS_VOUT_OV_WARN_LIMIT] = 2;
    s->pmbus_regmap_field_len[PMBUS_VOUT_UV_WARN_LIMIT] = 2;
    s->pmbus_regmap_field_len[PMBUS_VOUT_UV_FAULT_LIMIT] = 2;

    *((uint16_t*)&s->pmbus_iout_oc_fault_limit) = 0xf230;
    *((uint16_t*)&s->pmbus_iout_oc_warn_limit)= 0xf214;
    *((uint16_t*)&s->pmbus_iout_uc_fault_limit)= 0xffff;

    s->pmbus_regmap[PMBUS_IOUT_OC_FAULT_LIMIT] = s->pmbus_iout_oc_fault_limit;
    s->pmbus_regmap[PMBUS_IOUT_OC_WARN_LIMIT] = s->pmbus_iout_oc_warn_limit;
    s->pmbus_regmap[PMBUS_IOUT_UC_FAULT_LIMIT] = s->pmbus_iout_uc_fault_limit;

    s->pmbus_regmap_field_len[PMBUS_IOUT_OC_FAULT_LIMIT] = 2;
    s->pmbus_regmap_field_len[PMBUS_IOUT_OC_WARN_LIMIT] = 2;
    s->pmbus_regmap_field_len[PMBUS_IOUT_UC_FAULT_LIMIT] = 2;

    *((uint16_t*)&s->pmbus_pout_op_fault_limit) = 0x0b3e;
    *((uint16_t*)&s->pmbus_pout_op_warn_limit)= 0x0b20;
    *((uint16_t*)&s->pmbus_pin_op_warn_limit)= 0x0b9d;

    s->pmbus_regmap[PMBUS_POUT_OP_FAULT_LIMIT] = s->pmbus_pout_op_fault_limit;
    s->pmbus_regmap[PMBUS_POUT_OP_WARN_LIMIT] = s->pmbus_pout_op_warn_limit;
    s->pmbus_regmap[PMBUS_PIN_OP_WARN_LIMIT] = s->pmbus_pin_op_warn_limit;

    s->pmbus_regmap_field_len[PMBUS_POUT_OP_FAULT_LIMIT] = 2;
    s->pmbus_regmap_field_len[PMBUS_POUT_OP_WARN_LIMIT] = 2;
    s->pmbus_regmap_field_len[PMBUS_PIN_OP_WARN_LIMIT] = 2;


    *((uint16_t*)&s->pmbus_iin_oc_fault_limit) = 0xda46;
    *((uint16_t*)&s->pmbus_iin_oc_warn_limit)= 0xda3a;

    s->pmbus_regmap[PMBUS_IIN_OC_FAULT_LIMIT] = s->pmbus_iin_oc_fault_limit;
    s->pmbus_regmap[PMBUS_IIN_OC_WARN_LIMIT] = s->pmbus_iin_oc_warn_limit;

    s->pmbus_regmap_field_len[PMBUS_IIN_OC_FAULT_LIMIT] = 2;
    s->pmbus_regmap_field_len[PMBUS_IIN_OC_WARN_LIMIT] = 2;


    s->pmbus_regmap[VPD_POINTER_ADDR] = vpd_pointer;
    s->pmbus_regmap[CHSUM_FIELD] = vpd_crc;
    s->pmbus_regmap[PN_VPD_FIELD] = vpd_pn;
    s->pmbus_regmap[SN_VPD_FIELD] = vpd_sn;
    s->pmbus_regmap[REV_VPD_FIELD] = vpd_rev;
    s->pmbus_regmap[MFG_DATE_VPD_FIELD] = vpd_mfg;
    s->pmbus_regmap[CAP_VPD_FIELD] = vpd_cap;
    s->pmbus_regmap[RSRVD_VPD_FIELD] = vpd_rsrv;
    s->pmbus_regmap[RSRVD0_VPD_FIELD] = vpd_rsrv;
    s->pmbus_regmap[RSRVD1_VPD_FIELD] = vpd_rsrv;
    s->pmbus_regmap[PMBUS_MFR_ID] = mfr_id;

    s->pmbus_regmap_field_len[VPD_POINTER_ADDR] = 17;
    s->pmbus_regmap_field_len[CHSUM_FIELD] = 2;
    s->pmbus_regmap_field_len[PN_VPD_FIELD] = 31;
    s->pmbus_regmap_field_len[SN_VPD_FIELD] = 25;
    s->pmbus_regmap_field_len[REV_VPD_FIELD] = 9;
    s->pmbus_regmap_field_len[MFG_DATE_VPD_FIELD] = 9;
    s->pmbus_regmap_field_len[CAP_VPD_FIELD] = 9;
    s->pmbus_regmap_field_len[RSRVD_VPD_FIELD] = 9;
    s->pmbus_regmap_field_len[RSRVD0_VPD_FIELD] = 9;
    s->pmbus_regmap_field_len[RSRVD1_VPD_FIELD] = 9;
    s->pmbus_regmap_field_len[PMBUS_MFR_ID] = 10;


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
