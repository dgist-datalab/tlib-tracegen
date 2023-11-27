/*
 * Copyright (c) Antmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

uint64_t tlib_get_system_register(const char *name);
void tlib_set_system_register(const char *name, uint64_t value);
