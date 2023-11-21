/*
 * Copyright (c) Antmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARCH_EXPORTS_H_
#define ARCH_EXPORTS_H_

#include <stdint.h>
#include "arch_exports_common.h"

uint32_t tlib_has_el3();

void tlib_set_available_els(bool el2_enabled, bool el3_enabled);
void tlib_set_current_el(uint32_t el);
void tlib_set_mpu_regions_count(uint32_t count);

#endif
