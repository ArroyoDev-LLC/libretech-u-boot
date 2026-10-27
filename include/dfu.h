/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * dfu.h - DFU flashable area description
 *
 * Copyright (C) 2012 Samsung Electronics
 * authors: Andrzej Pietrasiewicz <andrzej.p@samsung.com>
 *	    Lukasz Majewski <l.majewski@samsung.com>
 */

#ifndef __DFU_ENTITY_H_
#define __DFU_ENTITY_H_

#include <linux/errno.h>
#include <linux/list.h>
#include <mmc.h>
#include <spi_flash.h>
#include <linux/usb/composite.h>

enum dfu_device_type {
	DFU_DEV_MMC = 1,
	DFU_DEV_ONENAND,
	DFU_DEV_NAND,
	DFU_DEV_RAM,
	DFU_DEV_SF,
	DFU_DEV_MTD,
	DFU_DEV_VIRT,
	DFU_DEV_SCSI,
};

enum dfu_layout {
	DFU_RAW_ADDR = 1,
	DFU_FS_FAT,
	DFU_FS_EXT2,
	DFU_FS_EXT3,
	DFU_FS_EXT4,
	DFU_RAM_ADDR,
	DFU_SKIP,
	DFU_SCRIPT,
	DFU_RAM_LZMA,
};

enum dfu_op {
	DFU_OP_READ = 1,
	DFU_OP_WRITE,
	DFU_OP_SIZE,
};

struct mmc_internal_data {
	int dev_num;

	/* RAW programming */
	unsigned int lba_start;
	unsigned int lba_size;
	unsigned int lba_blk_size;

	/* eMMC HW partition access */
	int hw_partition;

	/* FAT/EXT */
	unsigned int dev;
	unsigned int part;
};

struct mtd_internal_data {
	struct mtd_info *info;

	/* RAW programming */
	u64 start;
	u64 size;
	/* for ubi partition */
	unsigned int ubi;
};

struct nand_internal_data {
	/* RAW programming */
	u64 start;
	u64 size;

	unsigned int dev;
	unsigned int part;
	/* for nand/ubi use */
	unsigned int ubi;
};

struct ram_internal_data {
	unsigned long	start;
	unsigned int	size;
	/* DFU_RAM_LZMA: where the compressed download lands, and how much did */
	unsigned long	staging;
	u64		staged;
};

struct sf_internal_data {
	struct spi_flash *dev;

	/* RAW programming */
	u64 start;
	u64 size;
	/* for sf/ubi use */
	unsigned int ubi;
};

struct virt_internal_data {
	int dev_num;
};

struct scsi_internal_data {
	int lun;

	/* RAW programming */
	unsigned int lba_start;
	unsigned int lba_size;
	unsigned int lba_blk_size;

	/* FAT/EXT */
	unsigned int dev; // Always 0???
	unsigned int part;
};

#if defined(CONFIG_DFU_NAME_MAX_SIZE)
#define DFU_NAME_SIZE			CONFIG_DFU_NAME_MAX_SIZE
#else
#define DFU_NAME_SIZE			32
#endif
#ifndef DFU_DEFAULT_POLL_TIMEOUT
#define DFU_DEFAULT_POLL_TIMEOUT 0
#endif
#ifndef DFU_MANIFEST_POLL_TIMEOUT
#define DFU_MANIFEST_POLL_TIMEOUT	DFU_DEFAULT_POLL_TIMEOUT
#endif

struct dfu_entity {
	char			name[DFU_NAME_SIZE];
	int                     alt;
	void                    *dev_private;
	enum dfu_device_type    dev_type;
	enum dfu_layout         layout;
	unsigned long           max_buf_size;

	union {
		struct mmc_internal_data mmc;
		struct mtd_internal_data mtd;
		struct nand_internal_data nand;
		struct ram_internal_data ram;
		struct sf_internal_data sf;
		struct virt_internal_data virt;
		struct scsi_internal_data scsi;
	} data;

	int (*get_medium_size)(struct dfu_entity *dfu, u64 *size);

	int (*read_medium)(struct dfu_entity *dfu,
			u64 offset, void *buf, long *len);

	int (*write_medium)(struct dfu_entity *dfu,
			u64 offset, void *buf, long *len);

	int (*flush_medium)(struct dfu_entity *dfu);
	unsigned int (*poll_timeout)(struct dfu_entity *dfu);

	/*
	 * Optional: where @len bytes at medium @offset live in memory, when
	 * the medium is memory a controller can DMA to and from directly
	 * (dfu_ram's plain layout); NULL when it cannot take this span.
	 */
	void *(*direct_buf)(struct dfu_entity *dfu, u64 offset, long len);

	void (*free_entity)(struct dfu_entity *dfu);

	struct list_head list;

	/* on the fly state */
	u32 crc;
	u64 offset;
	int i_blk_seq_num;
	u8 *i_buf;
	u8 *i_buf_start;
	u8 *i_buf_end;
	u64 r_left;
	long b_left;

	u32 bad_skip;	/* for nand use */

	unsigned int inited:1;
};

extern struct list_head dfu_list;

#ifdef CONFIG_SET_DFU_ALT_INFO
/**
 * set_dfu_alt_info() - set dfu_alt_info environment variable
 *
 * If CONFIG_SET_DFU_ALT_INFO=y, this board specific function is called to set
 * environment variable dfu_alt_info.
 *
 * @interface:	dfu interface, e.g. "mmc" or "nand"
 * @devstr:	device number as string
 */
void set_dfu_alt_info(char *interface, char *devstr);
#endif

/**
 * dfu_alt_init() - initialize buffer for dfu entities
 *
 * @num:	number of entities
 * @dfu:	on return allocated buffer
 * Return:	0 on success
 */
int dfu_alt_init(int num, struct dfu_entity **dfu);

/**
 * dfu_alt_add() - add alternate to dfu entity buffer
 *
 * @dfu:	dfu entity
 * @interface:	dfu interface, e.g. "mmc" or "nand"
 * @devstr:	device number as string
 * @s:		string description of alternate
 * Return:	0 on success
 */
int dfu_alt_add(struct dfu_entity *dfu, char *interface, char *devstr, char *s);

/**
 * dfu_config_entities() - initialize dfu entitities from envirionment
 *
 * Initialize the list of dfu entities from environment variable dfu_alt_info.
 * The list must be freed by calling dfu_free_entities(). This function bypasses
 * set_dfu_alt_info(). So typically you should use dfu_init_env_entities()
 * instead.
 *
 * See function :c:func:`dfu_free_entities`
 * See function :c:func:`dfu_init_env_entities`
 *
 * @s:		string with alternates
 * @interface:	interface, e.g. "mmc" or "nand"
 * @devstr:	device number as string
 * Return:	0 on success, a negative error code otherwise
 */
int dfu_config_entities(char *s, char *interface, char *devstr);

/**
 * dfu_free_entities() - free the list of dfu entities
 *
 * Free the internal list of dfu entities.
 *
 * See function :c:func:`dfu_init_env_entities`
 */
void dfu_free_entities(void);

/**
 * dfu_show_entities() - print DFU alt settings list
 */
void dfu_show_entities(void);

/**
 * dfu_get_alt_number() - get number of alternates
 *
 * Return: number of alternates in the dfu entities list
 */
int dfu_get_alt_number(void);

/**
 * dfu_entities_in_use() - whether an entity list is built
 *
 * dfu_get_alt_number() keeps the last list's count after dfu_free_entities();
 * this answers whether a list is there now, i.e. whether its owner (a DFU
 * session, or the usbacm console's resident function) still holds it.
 *
 * Return:	true while the list has entities
 */
bool dfu_entities_in_use(void);

/**
 * dfu_console_release() - the usbacm console gives up its entity list
 *
 * Called before a list is built (dfu_init_env_entities()): with
 * USB_FUNCTION_ACM_DFU the console's gadget holds the one list while it is
 * up, so it is suspended first and comes back once the list is free again.
 * A no-op otherwise.
 */
void dfu_console_release(void);

/**
 * dfu_set_progress() - whether transfers print their progress
 *
 * The '#' per buffer and the lines that close an upload or download.  On by
 * default.  f_dfu turns it off while the usbacm console's resident function
 * is the one transferring: stdout is then the user's terminal on the same
 * gadget, and every mark would land in it.  Errors print either way.
 *
 * @on:		print progress
 */
void dfu_set_progress(bool on);

/**
 * dfu_get_dev_type() - get string representation for dfu device type
 *
 * @type:	device type
 * Return:	string representation for device type
 */
const char *dfu_get_dev_type(enum dfu_device_type type);

/**
 * dfu_get_layout() - get string describing layout
 *
 * Internally layouts are represented by enum dfu_device_type values. This
 * function translates an enum value to a human readable string, e.g. DFU_FS_FAT
 * is translated to "FAT".
 *
 * @layout:	layout
 * Result:	string representation for the layout
 */
const char *dfu_get_layout(enum dfu_layout layout);

/**
 * dfu_get_entity() - get dfu entity for an alternate id
 *
 * @alt:	alternate id
 * Return:	dfu entity
 */
struct dfu_entity *dfu_get_entity(int alt);

char *dfu_extract_token(char** e, int *n);

/**
 * dfu_get_alt() - get alternate id for filename
 *
 * Environment variable dfu_alt_info defines the write destinations (alternates)
 * for different filenames. This function get the index of the alternate for
 * a filename. If an absolute filename is provided (starting with '/'), the
 * directory path is ignored.
 *
 * @name:	filename
 * Return:	id of the alternate or negative error number (-ENODEV)
 */
int dfu_get_alt(char *name);

/**
 * dfu_init_env_entities() - initialize dfu entitities from envirionment
 *
 * Initialize the list of dfu entities from environment variable dfu_alt_info.
 * The list must be freed by calling dfu_free_entities().
 * @interface and @devstr are used to select the relevant set of alternates
 * from environment variable dfu_alt_info.
 *
 * If environment variable dfu_alt_info specifies the interface and the device,
 * use NULL for @interface and @devstr.
 *
 * See function :c:func:`dfu_free_entities`
 *
 * @interface:	interface, e.g. "mmc" or "nand"
 * @devstr:	device number as string
 * Return:	0 on success, a negative error code otherwise
 */
int dfu_init_env_entities(char *interface, char *devstr);

/**
 * dfu_bare_phase() - DFU runs in a phase built without an environment
 *
 * True only in an xPL phase whose own ENV_SUPPORT symbol is off
 * (CONFIG_IS_ENABLED() is phase-aware: SPL_ENV_SUPPORT in SPL, TPL_/VPL_ in
 * TPL/VPL, and ENV_SUPPORT, which is always y, in U-Boot proper).  Such a
 * phase has no environment, no shell and no filesystem layer for DFU, so the
 * caller hands the alternate list to dfu_alt_init()/dfu_alt_add() in C, the
 * DFU core reads no environment variable, and script and filesystem entities
 * are refused.  Every other build -- U-Boot proper, and an xPL phase with an
 * environment -- compiles the code these tests guard exactly as before.
 *
 * Return:	true in a bare xPL phase, false otherwise (a compile-time constant)
 */
static inline bool dfu_bare_phase(void)
{
	return !CONFIG_IS_ENABLED(ENV_SUPPORT);
}

unsigned char *dfu_get_buf(struct dfu_entity *dfu);
unsigned char *dfu_free_buf(void);
unsigned long dfu_get_buf_size(void);
bool dfu_usb_get_reset(void);

#ifdef CONFIG_DFU_TIMEOUT
unsigned long dfu_get_timeout(void);
void dfu_set_timeout(unsigned long);
/* ms since the timeout was last set */
unsigned long dfu_timeout_elapsed(void);
/* the timeout each host request restarts (0, the default: none once a host is seen) */
void dfu_set_idle_timeout(unsigned long);
/* a host request or bulk progress: restart the timeout as the idle timeout */
void dfu_host_active(void);
#endif

/**
 * dfu_background() - a DFU back end's idle work, from the gadget loop
 *
 * Called on every pass of run_usb_dnl_gadget()'s loop, between USB events,
 * so a back end can process what has arrived while the controller receives
 * the next block (dfu_ram's LZMA layout unpacks this way). Weak no-op.
 */
void dfu_background(void);

/**
 * dfu_read() - read from dfu entity
 *
 * The block sequence number @blk_seq_num is a 16 bit counter that must be
 * incremented with each call for the same dfu entity @de.
 *
 * @de:			dfu entity
 * @buf:		buffer
 * @size:		size of buffer
 * @blk_seq_num:	block sequence number
 * Return:		0 for success, -1 for error
 */
int dfu_read(struct dfu_entity *de, void *buf, int size, int blk_seq_num);

/**
 * dfu_write() - write to dfu entity
 *
 * Write the contents of a buffer @buf to the dfu entity @de. After writing
 * the last block call dfu_flush(). If a file is already loaded completely
 * into memory it is preferable to use dfu_write_from_mem_addr() which takes
 * care of blockwise transfer and flushing.
 *
 * The block sequence number @blk_seq_num is a 16 bit counter that must be
 * incremented with each call for the same dfu entity @de.
 *
 * See function :c:func:`dfu_flush`
 * See function :c:func:`dfu_write_from_mem_addr`
 *
 * @de:			dfu entity
 * @buf:		buffer
 * @size:		size of buffer
 * @blk_seq_num:	block sequence number
 * Return:		0 for success, -1 for error
 */
int dfu_write(struct dfu_entity *de, void *buf, int size, int blk_seq_num);

/**
 * dfu_direct_buf() - where the next transfer block can go without a copy
 *
 * For an entity with a direct_buf op and nothing buffered: the memory that
 * the next @size bytes of a download (@read false) or an upload (@read true)
 * occupy, so a USB controller can DMA the data stage straight to or from it.
 * The caller then reports the block with dfu_write_direct() /
 * dfu_read_direct() instead of dfu_write() / dfu_read().
 *
 * @de:		dfu entity
 * @size:	block size
 * @read:	true for an upload
 * Return:	the memory, or NULL when the block must take the copying path
 */
void *dfu_direct_buf(struct dfu_entity *de, int size, bool read);

/**
 * dfu_write_direct() - account for a download block already in place
 *
 * @de:			dfu entity
 * @size:		bytes the controller wrote at dfu_direct_buf()
 * @blk_seq_num:	block sequence number
 * Return:		0 for success, -1 for error
 */
int dfu_write_direct(struct dfu_entity *de, int size, int blk_seq_num);

/**
 * dfu_read_direct() - account for an upload block sent from its place
 *
 * @de:			dfu entity
 * @size:		requested block size
 * @blk_seq_num:	block sequence number
 * Return:		bytes in this block (less than @size at the end), or -1
 */
int dfu_read_direct(struct dfu_entity *de, int size, int blk_seq_num);

/**
 * dfu_flush() - flush to dfu entity
 *
 * This function has to be called after writing the last block to the dfu
 * entity @de.
 *
 * The block sequence number @blk_seq_num is a 16 bit counter that must be
 * incremented with each call for the same dfu entity @de.
 *
 * See function :c:func:`dfu_write`
 *
 * @de:			dfu entity
 * @buf:		ignored
 * @size:		ignored
 * @blk_seq_num:	block sequence number of last write - ignored
 * Return:		0 for success, -1 for error
 */
int dfu_flush(struct dfu_entity *de, void *buf, int size, int blk_seq_num);

/**
 * dfu_initiated_callback() - weak callback called on DFU transaction start
 *
 * It is a callback function called by DFU stack when a DFU transaction is
 * initiated. This function allows to manage some board specific behavior on
 * DFU targets.
 *
 * @dfu:	pointer to the dfu_entity, which should be initialized
 */
void dfu_initiated_callback(struct dfu_entity *dfu);

/**
 * dfu_flush_callback() - weak callback called at the end of the DFU write
 *
 * It is a callback function called by DFU stack after DFU manifestation.
 * This function allows to manage some board specific behavior on DFU targets
 *
 * @dfu:	pointer to the dfu_entity, which should be flushed
 */
void dfu_flush_callback(struct dfu_entity *dfu);

/**
 * dfu_error_callback() - weak callback called at the DFU write error
 *
 * It is a callback function called by DFU stack after DFU write error.
 * This function allows to manage some board specific behavior on DFU targets
 *
 * @dfu:	pointer to the dfu_entity which cause the error
 * @msg:	the message of the error
 */
void dfu_error_callback(struct dfu_entity *dfu, const char *msg);

/**
 * dfu_board_manifest_check() - weak hook: may this download be manifested?
 *
 * Called at the end of a download (the zero-length DNLOAD that leads to the
 * manifest step), before the buffered bytes are written to the medium (for
 * a script entity, before they run).  A non-zero return refuses the
 * download: the host's GETSTATUS reads bStatus errTARGET, bState dfuERROR,
 * the buffered bytes are discarded, and the session stays in DFU (the host
 * clears the error with CLRSTATUS).  The default accepts everything.
 *
 * @dfu:	the entity downloaded to
 * @buf:	the bytes not yet written to the medium
 * @len:	their count
 * @written:	bytes of this download already written to the medium (0 when
 *		the whole download is still in @buf)
 */
int dfu_board_manifest_check(struct dfu_entity *dfu, const void *buf,
			     long len, u64 written);

/**
 * dfu_manifest_check() - run dfu_board_manifest_check() on @dfu's download
 *
 * Return: 0 to manifest; -EBADMSG when refused, in which case the
 * transaction has been cleaned up and nothing will be written.
 */
int dfu_manifest_check(struct dfu_entity *dfu);

int dfu_transaction_initiate(struct dfu_entity *dfu, bool read);
void dfu_transaction_cleanup(struct dfu_entity *dfu);

/*
 * dfu_defer_flush - pointer to store dfu_entity for deferred flashing.
 *		     It should be NULL when not used.
 */
extern struct dfu_entity *dfu_defer_flush;

/**
 * dfu_get_defer_flush() - get current value of dfu_defer_flush pointer
 *
 * Return:	value of the dfu_defer_flush pointer
 */
static inline struct dfu_entity *dfu_get_defer_flush(void)
{
	return dfu_defer_flush;
}

/**
 * dfu_set_defer_flush() - set the dfu_defer_flush pointer
 *
 * @dfu:	pointer to the dfu_entity, which should be written
 */
static inline void dfu_set_defer_flush(struct dfu_entity *dfu)
{
	dfu_defer_flush = dfu;
}

/**
 * dfu_write_from_mem_addr() - write data from memory to DFU managed medium
 *
 * This function adds support for writing data starting from fixed memory
 * address (like $loadaddr) to dfu managed medium (e.g. NAND, MMC, file system)
 *
 * @dfu:	dfu entity to which we want to store data
 * @buf:	fixed memory address from where data starts
 * @size:	number of bytes to write
 *
 * Return:	0 on success, other value on failure
 */
int dfu_write_from_mem_addr(struct dfu_entity *dfu, void *buf, int size);

/* Device specific */
/* Each entity has 5 arguments in maximum. */
#define DFU_MAX_ENTITY_ARGS	5

#if CONFIG_IS_ENABLED(DFU_MMC)
extern int dfu_fill_entity_mmc(struct dfu_entity *dfu, char *devstr,
			       char **argv, int argc);
#else
static inline int dfu_fill_entity_mmc(struct dfu_entity *dfu, char *devstr,
				      char **argv, int argc)
{
	puts("MMC support not available!\n");
	return -1;
}
#endif

#if CONFIG_IS_ENABLED(DFU_NAND)
extern int dfu_fill_entity_nand(struct dfu_entity *dfu, char *devstr,
				char **argv, int argc);
#else
static inline int dfu_fill_entity_nand(struct dfu_entity *dfu, char *devstr,
				       char **argv, int argc)
{
	puts("NAND support not available!\n");
	return -1;
}
#endif

#if CONFIG_IS_ENABLED(DFU_RAM)
extern int dfu_fill_entity_ram(struct dfu_entity *dfu, char *devstr,
			       char **argv, int argc);
#else
static inline int dfu_fill_entity_ram(struct dfu_entity *dfu, char *devstr,
				      char **argv, int argc)
{
	puts("RAM support not available!\n");
	return -1;
}
#endif

#if CONFIG_IS_ENABLED(DFU_SF)
extern int dfu_fill_entity_sf(struct dfu_entity *dfu, char *devstr,
			      char **argv, int argc);
#else
static inline int dfu_fill_entity_sf(struct dfu_entity *dfu, char *devstr,
				     char **argv, int argc)
{
	puts("SF support not available!\n");
	return -1;
}
#endif

#if CONFIG_IS_ENABLED(DFU_MTD)
extern int dfu_fill_entity_mtd(struct dfu_entity *dfu, char *devstr,
			       char **argv, int argc);
#else
static inline int dfu_fill_entity_mtd(struct dfu_entity *dfu, char *devstr,
				      char **argv, int argc)
{
	puts("MTD support not available!\n");
	return -1;
}
#endif

#if CONFIG_IS_ENABLED(DFU_VIRT)
int dfu_fill_entity_virt(struct dfu_entity *dfu, char *devstr,
			 char **argv, int argc);
int dfu_write_medium_virt(struct dfu_entity *dfu, u64 offset,
			  void *buf, long *len);
int dfu_get_medium_size_virt(struct dfu_entity *dfu, u64 *size);
int dfu_read_medium_virt(struct dfu_entity *dfu, u64 offset,
			 void *buf, long *len);
#else
static inline int dfu_fill_entity_virt(struct dfu_entity *dfu, char *devstr,
				       char **argv, int argc)
{
	puts("VIRT support not available!\n");
	return -1;
}
#endif

#if CONFIG_IS_ENABLED(DFU_SCSI)
int dfu_fill_entity_scsi(struct dfu_entity *dfu, char *devstr,
			 char **argv, int argc);
#else
static inline int dfu_fill_entity_scsi(struct dfu_entity *dfu, char *devstr,
				       char **argv, int argc)
{
	puts("SCSI support not available!\n");
	return -1;
}
#endif

extern bool dfu_reinit_needed;
extern bool dfu_alt_info_changed;

#if CONFIG_IS_ENABLED(DFU_WRITE_ALT)
/**
 * dfu_write_by_name() - write data to DFU medium
 * @dfu_entity_name:	Name of DFU entity to write
 * @addr:		Address of data buffer to write
 * @len:		Number of bytes
 * @interface:		Destination DFU medium (e.g. "mmc")
 * @devstring:		Instance number of destination DFU medium (e.g. "1")
 *
 * This function is storing data received on DFU supported medium which
 * is specified by @dfu_entity_name.
 *
 * Return:		0 - on success, error code - otherwise
 */
int dfu_write_by_name(char *dfu_entity_name, void *addr,
		      unsigned int len, char *interface, char *devstring);

/**
 * dfu_write_by_alt() - write data to DFU medium
 * @dfu_alt_num:	DFU alt setting number
 * @addr:		Address of data buffer to write
 * @len:		Number of bytes
 * @interface:		Destination DFU medium (e.g. "mmc")
 * @devstring:		Instance number of destination DFU medium (e.g. "1")
 *
 * This function is storing data received on DFU supported medium which
 * is specified by @dfu_alt_name.
 *
 * Return:		0 - on success, error code - otherwise
 */
int dfu_write_by_alt(int dfu_alt_num, void *addr, unsigned int len,
		     char *interface, char *devstring);
#else
static inline int dfu_write_by_name(char *dfu_entity_name, void *addr,
				    unsigned int len, char *interface,
				    char *devstring)
{
	puts("write support for DFU not available!\n");
	return -ENOSYS;
}

static inline int dfu_write_by_alt(int dfu_alt_num, void *addr,
				   unsigned int len, char *interface,
				   char *devstring)
{
	puts("write support for DFU not available!\n");
	return -ENOSYS;
}
#endif

int dfu_add(struct usb_configuration *c);
/* USB_FUNCTION_ACM_DFU: the DFU function the usbacm console's gadget carries */
int dfu_add_resident(struct usb_configuration *c);
#endif /* __DFU_ENTITY_H_ */
