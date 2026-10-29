/*
 * Copyright (c) 2026 Ivan Belokobylskiy
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "bootutil/bootutil_log.h"
#include "bootutil/bootutil.h"
#include "bootutil/fault_injection_hardening.h"
#include "flash_map_backend/flash_map_backend.h"
#include "do_boot.h"

BOOT_LOG_MODULE_DECLARE(mcuboot);

/*
 * A TLSR825x image is entered the way the boot ROM enters it: the image head
 * (vectors, .boot_ram_mirror, .ram_code) is copied to the bottom of SRAM,
 * which the CPU sees at address 0, and execution starts there. The head's
 * size is in the boot header the image carries at offset 8: the "KNLT"
 * signature, then the size in 16-byte units in the low half word.
 *
 * The copy overwrites this bootloader's own SRAM mirror, which holds its
 * vectors and .ram_code. Everything else it runs executes from flash, so the
 * copy loop is ordinary flash-resident code and runs with interrupts locked.
 * It cannot be a __ramfunc: this port does not copy .ramfunc to SRAM.
 */
#define TC32_BOOT_SIGNATURE	0x544c4e4bU
#define TC32_BOOT_SIG_OFFSET	8U
#define TC32_BOOT_COPY_UNIT	16U
#define TC32_BOOT_HEAD_MAX	CONFIG_TC32_BOOT_MIRROR_SIZE

static uint32_t tc32_head[TC32_BOOT_HEAD_MAX / sizeof(uint32_t)];

static void __noinline tc32_head_copy_and_enter(const uint32_t *src, size_t words)
{
	volatile uint32_t *dst = (volatile uint32_t *)DT_REG_ADDR(DT_CHOSEN(zephyr_sram));

	while (words > 0U) {
		*dst++ = *src++;
		words--;
	}

	__asm__ volatile("tmov r0, #0\n\t"
			 "tjex r0\n\t" ::: "r0", "memory");
	CODE_UNREACHABLE;
}

void do_boot(const struct boot_rsp *rsp)
{
	const uint8_t *image;
	uintptr_t flash_base;
	uint32_t signature;
	uint32_t head_size;
	int rc;

	rc = flash_device_base(rsp->br_flash_dev_id, &flash_base);
	if (rc != 0) {
		BOOT_LOG_DBG("Error getting flash device base. rc = %d", rc);
		FIH_PANIC;
	}

	image = (const uint8_t *)(flash_base + rsp->br_image_off + rsp->br_hdr->ih_hdr_size);

	memcpy(&signature, image + TC32_BOOT_SIG_OFFSET, sizeof(signature));
	memcpy(&head_size, image + TC32_BOOT_SIG_OFFSET + sizeof(signature), sizeof(head_size));
	head_size = (head_size & 0xffffU) * TC32_BOOT_COPY_UNIT;

	if (signature != TC32_BOOT_SIGNATURE || head_size == 0U || head_size > TC32_BOOT_HEAD_MAX) {
		BOOT_LOG_ERR("Bad TC32 boot header");
		FIH_PANIC;
	}

	memcpy(tc32_head, image, head_size);

	irq_lock();
	tc32_head_copy_and_enter(tc32_head, head_size / sizeof(uint32_t));

	CODE_UNREACHABLE;
}
