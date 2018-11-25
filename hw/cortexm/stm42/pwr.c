/*
 * STM32 MCU - PWR (power controller) emulation.
 *
 * Copyright (c) 2015 Liviu Ionescu.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <hw/cortexm/stm42/pwr.h>
#include <hw/cortexm/stm42/mcu.h>
#include <hw/cortexm/helper.h>
#include <hw/cortexm/svd.h>

/*
 * This file implements the STM32 PWR.
 *
 * The initial implementation is intended only to pass CMSIS initialisations.
 */

// STM32F407VG, STM32F407ZG, STM32F405RG
// DO NOT EDIT! Automatically generated!
static void stm32f40x_pwr_create_objects(Object *obj, JSON_Object *svd,
        const char *name)
{
    STM32PWRState *state = STM32_PWR_STATE(obj);

    JSON_Object *periph = svd_get_peripheral_by_name(svd, name);
    svd_add_peripheral_properties_and_children(obj, periph, svd);

    // Registers.
    state->u.f4.reg.cr = cm_object_get_child_by_name(obj, "CR");
    state->u.f4.reg.csr = cm_object_get_child_by_name(obj, "CSR");

    // CR bitfields.
    state->u.f4.fld.cr.lpds = cm_object_get_child_by_name(state->u.f4.reg.cr,
            "LPDS");
    state->u.f4.fld.cr.pdds = cm_object_get_child_by_name(state->u.f4.reg.cr,
            "PDDS");
    state->u.f4.fld.cr.cwuf = cm_object_get_child_by_name(state->u.f4.reg.cr,
            "CWUF");
    state->u.f4.fld.cr.csbf = cm_object_get_child_by_name(state->u.f4.reg.cr,
            "CSBF");
    state->u.f4.fld.cr.pvde = cm_object_get_child_by_name(state->u.f4.reg.cr,
            "PVDE");
    state->u.f4.fld.cr.pls = cm_object_get_child_by_name(state->u.f4.reg.cr,
            "PLS");
    state->u.f4.fld.cr.dbp = cm_object_get_child_by_name(state->u.f4.reg.cr,
            "DBP");
    state->u.f4.fld.cr.fpds = cm_object_get_child_by_name(state->u.f4.reg.cr,
            "FPDS");

    // CSR bitfields.
    state->u.f4.fld.csr.wuf = cm_object_get_child_by_name(state->u.f4.reg.csr,
            "WUF");
    state->u.f4.fld.csr.sbf = cm_object_get_child_by_name(state->u.f4.reg.csr,
            "SBF");
    state->u.f4.fld.csr.pvdo = cm_object_get_child_by_name(state->u.f4.reg.csr,
            "PVDO");
    state->u.f4.fld.csr.brr = cm_object_get_child_by_name(state->u.f4.reg.csr,
            "BRR");
    state->u.f4.fld.csr.ewup = cm_object_get_child_by_name(state->u.f4.reg.csr,
            "EWUP");
    state->u.f4.fld.csr.bre = cm_object_get_child_by_name(state->u.f4.reg.csr,
            "BRE");
    state->u.f4.fld.csr.vosrdy = cm_object_get_child_by_name(
            state->u.f4.reg.csr, "VOSRDY");
}

// ----------------------------------------------------------------------------
// ----- Private --------------------------------------------------------------

static void stm32_pwr_instance_init_callback(Object *obj)
{
    qemu_log_function_name();

}

static void stm32_pwr_realize_callback(DeviceState *dev, Error **errp)
{
    qemu_log_function_name();

    // Call parent realize().
    if (!cm_device_parent_realize(dev, errp, TYPE_STM32_PWR)) {
        return;
    }

    STM32MCUState *mcu = stm32_mcu_get();
    CortexMState *cm_state = CORTEXM_MCU_STATE(mcu);

    STM32PWRState *state = STM32_PWR_STATE(dev);
    // First thing first: get capabilities from MCU, needed everywhere.
    state->capabilities = mcu->capabilities;

    const STM32Capabilities *capabilities = state->capabilities;
    assert(capabilities != NULL);

    Object *obj = OBJECT(dev);

    const char *periph_name = "PWR";
    svd_set_peripheral_address_block(cm_state->svd_json, periph_name, obj);
    peripheral_create_memory_region(obj);

    switch (capabilities->family) {

    case STM32_FAMILY_F4:

        if (capabilities->f4.is_40x) {

            stm32f40x_pwr_create_objects(obj, cm_state->svd_json, periph_name);

        } else {
            assert(false);
        }

        // Auto bits.
        cm_object_property_set_str(state->u.f4.fld.csr.brr, "BRE", "follows");

        break;

    default:
        assert(false);
        break;
    }

    peripheral_prepare_registers(obj);
}

static void stm32_pwr_reset_callback(DeviceState *dev)
{
    qemu_log_function_name();

    // Call parent reset().
    cm_device_parent_reset(dev, TYPE_STM32_PWR);
}

static void stm32_pwr_class_init_callback(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32_pwr_reset_callback;
    dc->realize = stm32_pwr_realize_callback;
}

static const TypeInfo stm32_pwr_type_info = {
    .name = TYPE_STM32_PWR,
    .parent = TYPE_STM32_PWR_PARENT,
    .instance_init = stm32_pwr_instance_init_callback,
    .instance_size = sizeof(STM32PWRState),
    .class_init = stm32_pwr_class_init_callback,
    .class_size = sizeof(STM32PWRClass)
/**/
};

static void stm32_pwr_register_types(void)
{
    type_register_static(&stm32_pwr_type_info);
}

type_init(stm32_pwr_register_types);

// ----------------------------------------------------------------------------
