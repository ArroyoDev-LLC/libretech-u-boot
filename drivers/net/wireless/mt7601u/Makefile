# SPDX-License-Identifier: GPL-2.0-only
#
# MediaTek MT7601U USB Wi-Fi background scan (port of the Linux mt7601u driver)

obj-$(CONFIG_WIFI_MT7601U) += usb.o mcu.o init.o eeprom.o phy.o scan.o \
			      connect.o wpa_crypto.o sae.o cmd_wifi.o fw.o

MT7601U_FW_FILE := $(patsubst "%",%,$(CONFIG_WIFI_MT7601U_FIRMWARE_PATH))

asflags-y += -DMT7601U_FW_PATH=\"$(MT7601U_FW_FILE)\"

# .incbin: rebuild when the firmware file changes, and fail early (with a
# "No rule to make target" naming the path) when it is missing.
$(obj)/fw.o: $(MT7601U_FW_FILE)
