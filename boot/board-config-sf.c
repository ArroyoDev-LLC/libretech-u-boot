// SPDX-License-Identifier: GPL-2.0+
/*
 * Board config block, NOR transport, read through the driver-model SPI flash.
 *
 * boot/board-config.c reads the NOR block through five board hooks
 * (include/board-config.h). Boards whose NOR is only reachable before driver
 * model implement them with a bare controller reader (Meson). A board whose
 * block is read in U-Boot proper, after driver model -- board_config_env_import()
 * from misc_init_r() -- can use the SPI flash uclass instead, which is what
 * this file does: the default SPI flash (CONFIG_SF_DEFAULT_*), probed once.
 * Its size is what board_config_nor_offset() places the block from, the same
 * min(size, 16 MiB) - 64 KiB rule the board's DFU `config` alt writes to.
 */

#include <board-config.h>
#include <errno.h>
#include <spi.h>
#include <spi_flash.h>
#include <linux/types.h>

static struct spi_flash *board_config_sf;

bool board_nor_clk_enabled(void)
{
	return true;	/* the SPI flash uclass brings up its own clock */
}

bool board_nor_init(void)
{
	return true;
}

bool board_nor_probe(void)
{
	if (!board_config_sf)
		board_config_sf = spi_flash_probe(CONFIG_SF_DEFAULT_BUS,
						  CONFIG_SF_DEFAULT_CS,
						  CONFIG_SF_DEFAULT_SPEED,
						  CONFIG_SF_DEFAULT_MODE);
	return board_config_sf && board_config_sf->size;
}

u32 board_nor_size(void)
{
	return board_config_sf ? board_config_sf->size : 0;
}

int board_nor_read(u32 offset, void *buf, u32 len)
{
	if (!board_config_sf)
		return -ENODEV;
	return spi_flash_read(board_config_sf, offset, len, buf) ? -EIO : 0;
}
