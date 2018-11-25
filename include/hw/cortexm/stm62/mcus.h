/*
 * STM32 Cortex-M devices emulation.
 *
 * Copyright (c) 2014 Liviu Ionescu.
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

#ifndef STM32_MCUS_H_
#define STM32_MCUS_H_

#include "qemu/osdep.h"

#include <hw/cortexm/stm62/mcu.h>

/* ------------------------------------------------------------------------- */



#define TYPE_STM42F407VG "STM42F407VG"

// ----------------------------------------------------------------------------

// Parent definitions.
#define TYPE_STM32_DEVICE_PARENT TYPE_STM32_MCU
typedef STM32MCUClass STM32DeviceParentClass;
typedef STM32MCUState STM32DeviceParentState;

// ----------------------------------------------------------------------------

// Class definitions.
// Warning, this cast must not check the type!
#define STM32_DEVICE_GET_CLASS(obj) \
    ((STM32DeviceClass *)object_get_class(OBJECT(obj)))

// Structure to define the specifics of each MCU. Capabilities are
// split between core & stm32; they care processed by parent class
// constructors.
typedef struct {

    const char *name; // CMSIS device name

    const CortexMCapabilities cortexm;
    const STM32Capabilities *stm32;

} STM32PartInfo;

typedef struct {
    // private:
    STM32DeviceParentClass parent_class;
    // public:

    STM32PartInfo *part_info;
} STM32DeviceClass;

// ----------------------------------------------------------------------------

// Instance definitions.
typedef struct {
    // private:
    STM32DeviceParentState parent_class;
// public:

} STM32DeviceState;

// ----------------------------------------------------------------------------

#endif /* STM32_MCUS_H_ */
