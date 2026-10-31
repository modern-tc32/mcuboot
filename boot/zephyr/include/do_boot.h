/*
 * Copyright (c) 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DO_BOOT_H_
#define DO_BOOT_H_

#include <stdbool.h>
#include <bootutil/bootutil.h>

void do_boot(const struct boot_rsp *rsp);

/* True when the host holds the UART receive line low (a break) after the reset. */
bool tc32_serial_break_detect(void);

#endif /* DO_BOOT_H_ */
