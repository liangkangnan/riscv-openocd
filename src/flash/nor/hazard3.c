// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *                                                                         *
 *   Copyright (C) 2024 by Blue Liang                                      *
 *   liangkangnan@163.com                                                  *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>

/* hazard3 flash register locations */

#define FLASH_REG_BASE        0x40008000

#define HAZARD3_FLASH_CSR     0x00
#define HAZARD3_FLASH_TX      0x04
#define HAZARD3_FLASH_RX      0x08

/* FLASH_CSR register bits */

#define FLASH_DIRECT_MODE   (1 << 0)
#define FLASH_BUSY		    (1 << 1)

/* timeout values */

#define FLASH_WRITE_TIMEOUT 100
#define FLASH_ERASE_TIMEOUT 100

#define CMD_SECTOR_ERASE   0x20
#define CMD_CHIP_ERASE     0x60
#define CMD_PAGE_PROG      0x02
#define CMD_READ           0x03
#define CMD_READ_STATUS    0x05
#define CMD_WRITE_ENABLE   0x06
#define CMD_WRITE_DISABLE  0x04


struct hazard3_flash_bank {
	bool probed;
	uint32_t register_base;
};

static int hazard3_mass_erase(struct flash_bank *bank);
static int hazard3_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t address, uint32_t hwords_count);

/* flash bank stm32x <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(hazard3_flash_bank_command)
{
	struct hazard3_flash_bank *hazard3_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	LOG_INFO("%s()", __func__);

	hazard3_info = malloc(sizeof(struct hazard3_flash_bank));

	bank->driver_priv = hazard3_info;
	hazard3_info->probed = false;
	hazard3_info->register_base = FLASH_REG_BASE;

	/* The flash write must be aligned to a byte boundary */
	bank->write_start_alignment = bank->write_end_alignment = 1;

	return ERROR_OK;
}

static inline int hazard3_get_flash_reg(struct flash_bank *bank, uint32_t reg)
{
	struct hazard3_flash_bank *hazard3_info = bank->driver_priv;
	return reg + hazard3_info->register_base;
}

static inline int hazard3_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	return target_read_u32(target, hazard3_get_flash_reg(bank, HAZARD3_FLASH_CSR), status);
}

static int hazard3_wait_status_busy(struct flash_bank *bank, int timeout)
{
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = hazard3_get_flash_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%" PRIx32 "", status);
		if ((status & FLASH_BUSY) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FLASH_BUSY;
		}
		alive_sleep(1);
	}

	return retval;
}

static int hazard3_flash_send_data(struct flash_bank *bank, uint8_t *buf, uint32_t len)
{
	int retval;
	uint32_t i;
	uint32_t data;
	struct target *target = bank->target;

	retval = hazard3_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	for (i = 0; i < len; i++) {
		data = buf[i];
		retval = target_write_u32(target, hazard3_get_flash_reg(bank, HAZARD3_FLASH_TX), data);
		if (retval != ERROR_OK)
			return retval;
		retval = hazard3_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int hazard3_flash_read_data(struct flash_bank *bank, uint8_t *buf, uint32_t len)
{
	int retval;
	uint32_t i;
	uint32_t data;
	struct target *target = bank->target;

	retval = hazard3_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	for (i = 0; i < len; i++) {
		retval = target_write_u32(target, hazard3_get_flash_reg(bank, HAZARD3_FLASH_TX), 0xff);
		if (retval != ERROR_OK)
			return retval;
		retval = hazard3_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;
		retval = target_read_u32(target, hazard3_get_flash_reg(bank, HAZARD3_FLASH_RX), &data);
		if (retval != ERROR_OK)
			return retval;
		buf[i] = data & 0xff;
	}

	return ERROR_OK;
}

static int hazard3_wait_flash_wip(struct flash_bank *bank, int timeout)
{
	int retval;
	uint32_t val;
	struct target *target = bank->target;

	retval = target_read_u32(target, hazard3_get_flash_reg(bank, HAZARD3_FLASH_CSR), &val);
	if (retval != ERROR_OK)
		return retval;

	if (val & 0x1) {
		/* disable direct mode */
		retval = target_write_u32(target, hazard3_get_flash_reg(bank, HAZARD3_FLASH_CSR), 0x0);
		if (retval != ERROR_OK)
			return retval;
		alive_sleep(1);
	}

	/* wait for busy to clear */
	for (;;) {
		/* enable direct mode */
		retval = target_write_u32(target, hazard3_get_flash_reg(bank, HAZARD3_FLASH_CSR), 0x1);
		if (retval != ERROR_OK)
			return retval;

		uint8_t buf = CMD_READ_STATUS;
		retval = hazard3_flash_send_data(bank, &buf, 1);
		if (retval != ERROR_OK)
			return retval;

		retval = hazard3_flash_read_data(bank, &buf, 1);
		if (retval != ERROR_OK)
			return retval;

		/* disable direct mode */
		retval = target_write_u32(target, hazard3_get_flash_reg(bank, HAZARD3_FLASH_CSR), 0x0);
		if (retval != ERROR_OK)
			return retval;

		if (!(buf & 0x1))
			break;

		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash wip");
			return ERROR_FLASH_BUSY;
		}
		alive_sleep(1);
	}

	return ERROR_OK;
}

static int hazard3_flash_write_enable(struct flash_bank *bank, int en)
{
	int retval;
	uint8_t data;
	uint32_t val;
	struct target *target = bank->target;

	retval = target_read_u32(target, hazard3_get_flash_reg(bank, HAZARD3_FLASH_CSR), &val);
	if (retval != ERROR_OK)
		return retval;

	if (val & 0x1) {
		/* disable direct mode */
		retval = target_write_u32(target, hazard3_get_flash_reg(bank, HAZARD3_FLASH_CSR), 0x0);
		if (retval != ERROR_OK)
			return retval;
		alive_sleep(1);
	}

	/* enable direct mode */
	retval = target_write_u32(target, hazard3_get_flash_reg(bank, HAZARD3_FLASH_CSR), 0x1);
	if (retval != ERROR_OK)
		return retval;

	if (en)
		data = CMD_WRITE_ENABLE;
	else
		data = CMD_WRITE_DISABLE;
	retval = hazard3_flash_send_data(bank, &data, 1);
	if (retval != ERROR_OK)
		return retval;

	/* disable direct mode */
	retval = target_write_u32(target, hazard3_get_flash_reg(bank, HAZARD3_FLASH_CSR), 0x0);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int hazard3_protect_check(struct flash_bank *bank)
{
	return ERROR_OK;
}

static int hazard3_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	uint8_t buf[10];
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_INFO("%s(), first = %d, last = %d", __func__, first, last);

	if ((first == 0) && (last == (bank->num_sectors - 1)))
		return hazard3_mass_erase(bank);

	for (unsigned int i = first; i <= last; i++) {
		/* enable flash programming */
		retval = hazard3_flash_write_enable(bank, 1);
		if (retval != ERROR_OK)
			return retval;

		/* enable direct mode */
		retval = target_write_u32(target, hazard3_get_flash_reg(bank, HAZARD3_FLASH_CSR), 0x1);
		if (retval != ERROR_OK)
			return retval;

		buf[0] = CMD_SECTOR_ERASE;
		buf[1] = (i * 4096 >> 16) & 0xff;
		buf[2] = (i * 4096 >> 8) & 0xff;
		buf[3] = (i * 4096 >> 0) & 0xff;
		retval = hazard3_flash_send_data(bank, buf, 4);
		if (retval != ERROR_OK)
			goto flash_lock;

		/* disable direct mode */
		retval = target_write_u32(target, hazard3_get_flash_reg(bank, HAZARD3_FLASH_CSR), 0x0);
		if (retval != ERROR_OK)
			return retval;

		retval = hazard3_wait_flash_wip(bank, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK)
			goto flash_lock;
	}

flash_lock:
	return retval;
}

static int hazard3_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	return ERROR_OK;
}

static int hazard3_write_block_riscv(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t address, uint32_t bytes_count)
{
	struct target *target = bank->target;
	uint32_t buffer_size;
	uint32_t thisrun_bytes;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint8_t write_buffer[256 + 4];
	static const uint8_t hazard3_flash_write_code[] = {
#include "../../../contrib/loaders/flash/hazard3/hazard3.inc"
	};

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(hazard3_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* write algorithm data into memory */
	int retval = target_write_buffer(target, write_algorithm->address,
			sizeof(hazard3_flash_write_code), hazard3_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* memory buffer */
	buffer_size = target_get_working_area_avail(target);
	buffer_size = MIN(bytes_count, MAX(buffer_size, 256));

	retval = target_alloc_working_area(target, buffer_size, &source);
	/* Allocated size is always word aligned */
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		LOG_WARNING("no large enough working area available, can't do block memory writes");
		/* target_alloc_working_area() may return ERROR_FAIL if area backup fails:
		 * convert any error to ERROR_TARGET_RESOURCE_NOT_AVAILABLE
		 */
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	struct reg_param reg_params[4];

	init_reg_param(&reg_params[0], "a0", 32, PARAM_IN_OUT);	/* poiner to FLASH_CSR */
	init_reg_param(&reg_params[1], "a1", 32, PARAM_OUT);	/* count (bytes) */
	init_reg_param(&reg_params[2], "a2", 32, PARAM_OUT);	/* buffer start */
	init_reg_param(&reg_params[3], "a3", 32, PARAM_OUT);	/* poiner to FLASH_TX */

	LOG_INFO("%s(), buffer_size = 0x%x, address = 0x%x, bytes_count = 0x%x",
				__func__, buffer_size, address, bytes_count);

	while (bytes_count > 0) {
		/* enable flash programming */
		retval = hazard3_flash_write_enable(bank, 1);
		if (retval != ERROR_OK)
			return retval;

		/* Limit to the amount of data we actually want to write */
		if (bytes_count > 256)
			thisrun_bytes = 256;
		else
			thisrun_bytes = bytes_count;

		write_buffer[0] = CMD_PAGE_PROG;
		write_buffer[1] = (address >> 16) & 0xff;
		write_buffer[2] = (address >> 8) & 0xff;
		write_buffer[3] = (address >> 0) & 0xff;
		memcpy(&write_buffer[4], buffer, thisrun_bytes);

		/* Write data to buffer */
		retval = target_write_buffer(target, source->address,
					thisrun_bytes + 4, write_buffer);
		if (retval != ERROR_OK)
			break;

		buf_set_u32(reg_params[0].value, 0, 32, hazard3_get_flash_reg(bank, HAZARD3_FLASH_CSR));
		buf_set_u32(reg_params[1].value, 0, 32, thisrun_bytes + 4);
		buf_set_u32(reg_params[2].value, 0, 32, source->address);
		buf_set_u32(reg_params[3].value, 0, 32, hazard3_get_flash_reg(bank, HAZARD3_FLASH_TX));

		// enable dump wave
		//target_write_u32(target, 0x80000018, 0x1);

		retval = target_run_algorithm(target,
				0, NULL,
				ARRAY_SIZE(reg_params), reg_params,
				write_algorithm->address,
				write_algorithm->address + sizeof(hazard3_flash_write_code) - 4,
				10000, NULL);

		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to execute algorithm at 0x%" TARGET_PRIxADDR ": %d",
					write_algorithm->address, retval);
			break;
		}

		/* Actually we just need to check for programming errors
		 * hazard3_wait_flash_wip also reports error and clears status bits
		 */
		retval = hazard3_wait_flash_wip(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK) {
			LOG_ERROR("flash write failed at address 0x%"PRIx32,
					buf_get_u32(reg_params[3].value, 0, 32));
			break;
		}

		/* Update counters */
		buffer += thisrun_bytes;
		address += thisrun_bytes;
		bytes_count -= thisrun_bytes;
	}

	// disable dump wave
	//target_write_u32(target, 0x80000018, 0x0);

	for (unsigned int i = 0; i < ARRAY_SIZE(reg_params); i++)
		destroy_reg_param(&reg_params[i]);

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	return retval;
}

/** Writes a block to flash either using target algorithm.
 *  Flash controller must be unlocked before this call.
 */
static int hazard3_write_block(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t address, uint32_t bytes_count)
{
	int retval;

	retval = hazard3_write_block_riscv(bank, buffer, address, bytes_count);

	return retval;
}

static int hazard3_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	//LOG_INFO("%s(), offset = 0x%x, count = 0x%x", __func__, offset, count);

	int retval;

	/* write to flash */
	retval = hazard3_write_block(bank, buffer, bank->base + offset, count);

	return retval;
}

static int hazard3_read(struct flash_bank *bank,
		uint8_t *buffer, uint32_t offset, uint32_t count)
{
	int retval;

	LOG_INFO("%s(), offset = 0x%x, count = 0x%x", __func__, offset, count);

	/* disable direct mode */
	retval = target_write_u32(bank->target, hazard3_get_flash_reg(bank, HAZARD3_FLASH_CSR), 0x0);
	if (retval != ERROR_OK)
		return retval;

	return default_flash_read(bank, buffer, offset, count);
}

static int hazard3_probe(struct flash_bank *bank)
{
	struct hazard3_flash_bank *hazard3_info = bank->driver_priv;
	uint16_t flash_size_in_kb = 1024;
	int page_size = 4096;
	uint32_t base_address = 0x00000000;

	LOG_INFO("%s(), flash size = %d KiB", __func__, flash_size_in_kb);

	/* calculate numbers of pages */
	int num_pages = flash_size_in_kb * 1024 / page_size;

	/* check that calculation result makes sense */
	assert(num_pages > 0);

	free(bank->sectors);
	bank->sectors = NULL;

	free(bank->prot_blocks);
	bank->prot_blocks = NULL;
	bank->num_prot_blocks = 0;

	bank->base = base_address;
	bank->size = (num_pages * page_size);

	bank->num_sectors = num_pages;
	bank->sectors = alloc_block_array(0, page_size, num_pages);
	if (!bank->sectors)
		return ERROR_FAIL;

	hazard3_info->probed = true;

	return ERROR_OK;
}

static int hazard3_auto_probe(struct flash_bank *bank)
{
	struct hazard3_flash_bank *hazard3_info = bank->driver_priv;
	if (hazard3_info->probed)
		return ERROR_OK;

	return hazard3_probe(bank);
}

static int get_hazard3_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	command_print_sameline(cmd, "Hazard3 SPI norflash\n");

	return ERROR_OK;
}

static int hazard3_mass_erase(struct flash_bank *bank)
{
	uint8_t buf;
	int retval;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* enable flash programming */
	retval = hazard3_flash_write_enable(bank, 1);
	if (retval != ERROR_OK)
		return retval;

	/* enable direct mode */
	retval = target_write_u32(target, hazard3_get_flash_reg(bank, HAZARD3_FLASH_CSR), 0x1);
	if (retval != ERROR_OK)
		return retval;

	buf = CMD_CHIP_ERASE;
	retval = hazard3_flash_send_data(bank, &buf, 1);
	if (retval != ERROR_OK)
		return retval;

	/* disable direct mode */
	retval = target_write_u32(target, hazard3_get_flash_reg(bank, HAZARD3_FLASH_CSR), 0x0);
	if (retval != ERROR_OK)
		return retval;

	retval = hazard3_wait_flash_wip(bank, FLASH_ERASE_TIMEOUT);

	return retval;
}

COMMAND_HANDLER(hazard3_handle_mass_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = hazard3_mass_erase(bank);
	if (retval == ERROR_OK)
		command_print(CMD, "hazard3 mass erase complete");
	else
		command_print(CMD, "hazard3 mass erase failed");

	return retval;
}

static const struct command_registration hazard3_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = hazard3_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration hazard3_command_handlers[] = {
	{
		.name = "hazard3",
		.mode = COMMAND_ANY,
		.help = "hazard3 flash command group",
		.usage = "",
		.chain = hazard3_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver hazard3_flash = {
	.name = "hazard3",
	.commands = hazard3_command_handlers,
	.flash_bank_command = hazard3_flash_bank_command,
	.erase = hazard3_erase,
	.protect = hazard3_protect,
	.write = hazard3_write,
	.read = hazard3_read,
	.probe = hazard3_probe,
	.auto_probe = hazard3_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = hazard3_protect_check,
	.info = get_hazard3_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
