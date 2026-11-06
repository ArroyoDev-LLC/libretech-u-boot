// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2014, NVIDIA CORPORATION. All rights reserved.
 */

#include <malloc.h>
#include <errno.h>
#include <div64.h>
#include <dfu.h>
#include <dm/device.h>
#include <dm/device-internal.h>
#include <spi.h>
#include <spi_flash.h>
#include <jffs2/load_kernel.h>
#include <asm/cache.h>
#include <linux/kernel.h>
#include <linux/mtd/mtd.h>
#include <linux/ctype.h>

static int dfu_get_medium_size_sf(struct dfu_entity *dfu, u64 *size)
{
	*size = dfu->data.sf.size;

	return 0;
}

/*
 * The entity's flash, valid for this access.  With DM_SPI_FLASH, `sf probe`
 * removes the flash device and probes it again, which frees the spi_flash
 * the entity was given and allocates a new one: an access through the old
 * pointer reads freed memory (a Synchronous Abort on an S905D3-CC whose
 * console ran `sf probe` with the DFU gadget up).  The device itself stays
 * bound, so probe it (a no-op when active) and take its spi_flash anew.
 */
static struct spi_flash *dfu_sf_flash(struct dfu_entity *dfu)
{
	if (CONFIG_IS_ENABLED(DM_SPI_FLASH) && dfu->data.sf.udev) {
		if (device_probe(dfu->data.sf.udev)) {
			dfu->data.sf.dev = NULL;
			return NULL;
		}
		dfu->data.sf.dev = dev_get_uclass_priv(dfu->data.sf.udev);
	}

	return dfu->data.sf.dev;
}

static int dfu_read_medium_sf(struct dfu_entity *dfu, u64 offset, void *buf,
			      long *len)
{
	struct spi_flash *flash = dfu_sf_flash(dfu);
	long seglen = *len;
	int ret;

	if (!flash)
		return -ENODEV;
	if (seglen > (16 << 20))
		seglen = (16 << 20);

	ret = spi_flash_read(flash, dfu->data.sf.start + offset, seglen, buf);
	if (!ret)
		*len = seglen;

	return ret;
}

static u64 find_sector(struct spi_flash *flash, u64 start, u64 offset)
{
	return (lldiv((start + offset), flash->sector_size)) *
		flash->sector_size;
}

#if IS_ENABLED(CONFIG_XPL_BUILD)
/*
 * spi_flash_update() lives in cmd/sf.c, U-Boot proper's `sf` command, which
 * an xPL phase does not build.  The same update here for an SPL DFU session
 * (SPL_DFU_SF): per sector, read it, leave it if the data is already there,
 * else erase it and write it back with the new bytes in place.  No progress
 * output.  0, or a negative errno.
 */
static int dfu_sf_update(struct spi_flash *flash, u32 offset, size_t len,
			 const char *buf)
{
	u32 sect = flash->sector_size;
	char *cmp = memalign(ARCH_DMA_MINALIGN, sect);
	size_t todo;
	int ret = 0;

	if (!cmp)
		return -ENOMEM;
	for (; len && !ret; buf += todo, offset += todo, len -= todo) {
		u32 in = offset % sect, base = offset - in;

		todo = min_t(size_t, len, sect - in);
		ret = spi_flash_read(flash, base, sect, cmp);
		if (ret || !memcmp(cmp + in, buf, todo))
			continue;
		memcpy(cmp + in, buf, todo);
		ret = spi_flash_erase(flash, base, sect);
		if (!ret)
			ret = spi_flash_write(flash, base, sect, cmp);
	}
	free(cmp);

	return ret;
}
#endif

static int dfu_sf_write(struct spi_flash *flash, u32 offset, size_t len,
			const char *buf)
{
#if IS_ENABLED(CONFIG_XPL_BUILD)
	return dfu_sf_update(flash, offset, len, buf);
#else
	return spi_flash_update(flash, offset, len, buf);
#endif
}

#if CONFIG_IS_ENABLED(DFU_SF_BOOT_SECTOR_LAST)
/*
 * The flash's first erase sector, written last. A BootROM that boots from
 * offset 0 validates a header there and then runs what follows it, so an
 * update written front to back and cut short (a reset, a thermal shutdown,
 * a power cut) leaves a valid header over a broken loader, and the board
 * hangs where it should have fallen through to its next boot source: seen
 * on a ROC-RK3399-PC (rockchip B186 F33). So the first write that touches
 * offset 0 erases sector 0 at once, keeping its old bytes and the new ones
 * in RAM, and the flush writes it back last. Cut short anywhere before that,
 * sector 0 reads erased and the BootROM moves on. One transfer at a time.
 */
static char *sf_head;		/* sector 0 as it will be written */
static u32 sf_head_size;

static int dfu_sf_head_begin(struct spi_flash *flash)
{
	int ret;

	free(sf_head);
	sf_head_size = flash->sector_size;
	sf_head = memalign(ARCH_DMA_MINALIGN, sf_head_size);
	if (!sf_head)
		return -ENOMEM;
	ret = spi_flash_read(flash, 0, sf_head_size, sf_head);
	if (!ret)
		ret = spi_flash_erase(flash, 0, sf_head_size);
	if (ret) {
		free(sf_head);
		sf_head = NULL;
	}

	return ret;
}

static int dfu_sf_head_end(struct spi_flash *flash)
{
	int ret;

	if (!sf_head)
		return 0;
	ret = spi_flash_write(flash, 0, sf_head_size, sf_head);
	free(sf_head);
	sf_head = NULL;

	return ret;
}
#endif

static int dfu_write_medium_sf(struct dfu_entity *dfu,
			       u64 offset, void *buf, long *len)
{
	struct spi_flash *flash = dfu_sf_flash(dfu);
	u64 at = dfu->data.sf.start + offset;
	char *p = buf;
	long n = *len;
	int ret;

	if (!flash)
		return -ENODEV;
#if CONFIG_IS_ENABLED(DFU_SF_BOOT_SECTOR_LAST)
	if (at < flash->sector_size && n) {
		u32 in;

		if (at == 0 || !sf_head) {
			ret = dfu_sf_head_begin(flash);
			if (ret)
				return ret;
		}
		in = min_t(u64, n, sf_head_size - at);
		memcpy(sf_head + at, p, in);
		at += in;
		p += in;
		n -= in;
	}
#endif
	if (!n)
		return 0;
	ret = dfu_sf_write(flash, at, n, p);
	if (ret)
		return ret;

	return 0;
}

static int dfu_flush_medium_sf(struct dfu_entity *dfu)
{
	struct spi_flash *flash;
	u64 off, length;

#if CONFIG_IS_ENABLED(DFU_SF_BOOT_SECTOR_LAST)
	if (sf_head) {
		int ret;

		flash = dfu_sf_flash(dfu);
		if (!flash)
			return -ENODEV;
		ret = dfu_sf_head_end(flash);
		if (ret)
			return ret;
	}
#endif
	if (!CONFIG_IS_ENABLED(DFU_SF_PART) || !dfu->data.sf.ubi)
		return 0;
	flash = dfu_sf_flash(dfu);
	if (!flash)
		return -ENODEV;

	/* in case of ubi partition, erase rest of the partition */
	off = find_sector(flash, dfu->data.sf.start, dfu->offset);
	/* last write ended with unaligned length jump to next */
	if (off != dfu->data.sf.start + dfu->offset)
		off += flash->sector_size;
	length = dfu->data.sf.start + dfu->data.sf.size - off;
	if (length)
		return spi_flash_erase(flash, off, length);

	return 0;
}

static unsigned int dfu_polltimeout_sf(struct dfu_entity *dfu)
{
	/*
	 * Currently, Poll Timeout != 0 is only needed on nand
	 * ubi partition, as sectors which are not used need
	 * to be erased
	 */
	if (CONFIG_IS_ENABLED(DFU_SF_PART) && dfu->data.sf.ubi)
		return DFU_MANIFEST_POLL_TIMEOUT;

	return DFU_DEFAULT_POLL_TIMEOUT;
}

static void dfu_free_entity_sf(struct dfu_entity *dfu)
{
	/*
	 * In the DM case it is not necessary to free the SPI device.
	 * For the non-DM case we must ensure that the the SPI device is only
	 * freed once.
	 */
	if (!CONFIG_IS_ENABLED(DM_SPI_FLASH)) {
		struct spi_flash *dev = dfu->data.sf.dev;

		if (!dev)
			return;
		dfu->data.sf.dev = NULL;
		list_for_each_entry(dfu, &dfu_list, list) {
			if (dfu->data.sf.dev == dev)
				dfu->data.sf.dev = NULL;
		}
		spi_flash_free(dev);
	}
}

static struct spi_flash *parse_dev(char *devstr)
{
	unsigned int bus;
	unsigned int cs;
	unsigned int speed = CONFIG_SF_DEFAULT_SPEED;
	unsigned int mode = CONFIG_SF_DEFAULT_MODE;
	char *s, *endp;
	struct spi_flash *dev;
	bool use_dt = true;

	s = strsep(&devstr, ":");
	if (!s || !*s || (bus = simple_strtoul(s, &endp, 0), *endp)) {
		printf("Invalid SPI bus %s\n", s);
		return NULL;
	}

	s = strsep(&devstr, ":");
	if (!s || !*s || (cs = simple_strtoul(s, &endp, 0), *endp)) {
		printf("Invalid SPI chip-select %s\n", s);
		return NULL;
	}

	s = strsep(&devstr, ":");
	if (s && *s) {
		speed = simple_strtoul(s, &endp, 0);
		if (*endp || !speed) {
			printf("Invalid SPI speed %s\n", s);
			return NULL;
		}
		if (IS_ENABLED(CONFIG_DM_SPI_FLASH))
			use_dt = false;
	}

	s = strsep(&devstr, ":");
	if (s && *s) {
		mode = simple_strtoul(s, &endp, 0);
		if (*endp || mode > 3) {
			printf("Invalid SPI mode %s\n", s);
			return NULL;
		}
		if (IS_ENABLED(CONFIG_DM_SPI_FLASH))
			use_dt = false;
	}

	if (IS_ENABLED(CONFIG_DM_SPI_FLASH) && use_dt) {
		struct udevice *new;

		if (!spi_flash_probe_bus_cs(bus, cs, &new))
			dev = dev_get_uclass_priv(new);
		else
			dev = NULL;
	} else {
		dev = spi_flash_probe(bus, cs, speed, mode);
	}
	if (!dev) {
		printf("Failed to create SPI flash at %u:%u:%u:%u\n",
		       bus, cs, speed, mode);
		return NULL;
	}

	return dev;
}

int dfu_fill_entity_sf(struct dfu_entity *dfu, char *devstr, char **argv, int argc)
{
	char *s;
	char *devstr_bkup = strdup(devstr);

	dfu->data.sf.dev = parse_dev(devstr_bkup);
	free(devstr_bkup);
	if (!dfu->data.sf.dev)
		return -ENODEV;
	if (CONFIG_IS_ENABLED(DM_SPI_FLASH))
		dfu->data.sf.udev = dfu->data.sf.dev->dev;

	dfu->dev_type = DFU_DEV_SF;
	dfu->max_buf_size = dfu->data.sf.dev->sector_size;

	if (argc != 3)
		return -EINVAL;
	if (!strcmp(argv[0], "raw")) {
		dfu->layout = DFU_RAW_ADDR;
		dfu->data.sf.start = hextoul(argv[1], &s);
		if (*s)
			return -EINVAL;
		dfu->data.sf.size = hextoul(argv[2], &s);
		if (*s)
			return -EINVAL;
	} else if (CONFIG_IS_ENABLED(DFU_SF_PART) &&
		   (!strcmp(argv[0], "part") || !strcmp(argv[0], "partubi"))) {
		char mtd_id[32];
		struct mtd_device *mtd_dev;
		u8 part_num;
		struct part_info *pi;
		int ret, dev, part;

		dfu->layout = DFU_RAW_ADDR;

		dev = dectoul(argv[1], &s);
		if (*s)
			return -EINVAL;
		part = dectoul(argv[2], &s);
		if (*s)
			return -EINVAL;

		sprintf(mtd_id, "%s%d,%d", "nor", dev, part - 1);
		printf("using id '%s'\n", mtd_id);

		mtdparts_init();

		ret = find_dev_and_part(mtd_id, &mtd_dev, &part_num, &pi);
		if (ret != 0) {
			printf("Could not locate '%s'\n", mtd_id);
			return -1;
		}
		dfu->data.sf.start = pi->offset;
		dfu->data.sf.size = pi->size;
		if (!strcmp(argv[0], "partubi"))
			dfu->data.sf.ubi = 1;
	} else {
		printf("%s: Memory layout (%s) not supported!\n", __func__, argv[0]);
		spi_flash_free(dfu->data.sf.dev);
		return -1;
	}

	dfu->get_medium_size = dfu_get_medium_size_sf;
	dfu->read_medium = dfu_read_medium_sf;
	dfu->write_medium = dfu_write_medium_sf;
	dfu->flush_medium = dfu_flush_medium_sf;
	dfu->poll_timeout = dfu_polltimeout_sf;
	dfu->free_entity = dfu_free_entity_sf;

	/* initial state */
	dfu->inited = 0;

	return 0;
}
