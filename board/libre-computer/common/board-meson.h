#include <board-config.h>
#include <linux/types.h>

void meson_set_dfu_alt_info(char *interface, char *devstr);
int meson_gxl_misc_init_r(void);
void meson_setup_serial_env(void);
int board_early_init_r(void);

/*
 * True on a USB BootROM boot of a build whose environment is NOWHERE there
 * (CONFIG_ENV_IS_NOWHERE), i.e. a recovery / flash-session boot that runs on
 * compiled-in env. Always false without CONFIG_ENV_IS_NOWHERE.
 */
bool meson_usb_env_nowhere(void);
