/*
 * Copyright (c) 2026 Ivan Belokobylskiy
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>

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

/*
 * The instruction cache scratch area of this loader follows its head in SRAM,
 * and the copy below runs from flash through that cache. An application head
 * larger than this loader's head would overwrite the cache lines of the copy
 * loop, so the loader's head is padded (CONFIG_BOOT_TC32_HEAD_PAD) and an
 * application whose head does not fit under it is refused.
 */
extern char _tc32_boot_copy_size_align_256_[];

#if CONFIG_BOOT_TC32_HEAD_PAD > 0
static const uint8_t tc32_head_pad[CONFIG_BOOT_TC32_HEAD_PAD]
	__attribute__((section(".ram_code"), used, retain)) = {1};
#endif

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

	if (head_size > (uintptr_t)_tc32_boot_copy_size_align_256_) {
		BOOT_LOG_ERR("Application head is larger than the loader's");
		FIH_PANIC;
	}

	memcpy(tc32_head, image, head_size);

#if defined(CONFIG_MCUBOOT_SERIAL) && defined(CONFIG_BOOT_SERIAL_UART)
	/*
	 * Serial recovery receives through DMA into the loader's SRAM. Stop it
	 * before the jump: a byte that arrives later would be written into
	 * memory that belongs to the application by then.
	 */
	uart_irq_rx_disable(DEVICE_DT_GET(DT_CHOSEN(zephyr_uart_mcumgr)));
#endif

	irq_lock();
	tc32_head_copy_and_enter(tc32_head, head_size / sizeof(uint32_t));

	CODE_UNREACHABLE;
}

#ifdef CONFIG_BOOT_TC32_SERIAL_BREAK_ENTRANCE
/* PA0, the receive pin of uart0 (uart0_rx_default in tlsr8258.dtsi). */
#define TC32_SERIAL_RX_GPIO_INPUT	((volatile uint8_t *)DT_REG_ADDR(DT_NODELABEL(gpioa)))
#define TC32_SERIAL_RX_MASK		BIT(0)
#define TC32_SERIAL_BREAK_SAMPLES	20U
#define TC32_SERIAL_BREAK_STEP_US	100U

bool tc32_serial_break_detect(void)
{
	/*
	 * The line has to stay low for the whole 2 ms: a break lasts tens of
	 * milliseconds, a glitch of the host adapter during the reset does not.
	 */
	for (unsigned int i = 0U; i < TC32_SERIAL_BREAK_SAMPLES; i++) {
		if ((*TC32_SERIAL_RX_GPIO_INPUT & TC32_SERIAL_RX_MASK) != 0U) {
			return false;
		}
		k_busy_wait(TC32_SERIAL_BREAK_STEP_US);
	}

	return true;
}
#endif /* CONFIG_BOOT_TC32_SERIAL_BREAK_ENTRANCE */
