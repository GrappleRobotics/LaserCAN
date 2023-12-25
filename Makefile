# Bootloader 0.1.0 had an issue where if byte at offset 0x1400 was 0xFF, the bootloader wouldn't accept any new firmware because of a failure to erase the sector.
# This has since been fixed, but as a precaution, this will prevent us accidentally generating a firmware version with this property.
ifeq ($(OS),Windows_NT)
CP = powershell -noprofile -command cp
else
SHELL := /bin/bash
CP = cp
endif

all: bootloader firmware

bootloader:
	cd lasercan-bootloader && cargo build --release
	$(CP) ./lasercan-bootloader/target/thumbv7m-none-eabi/release/lasercan-bootloader target/lasercan-bootloader.elf
	arm-none-eabi-objcopy -O binary ./lasercan-bootloader/target/thumbv7m-none-eabi/release/lasercan-bootloader target/lasercan-bootloader.grplbt

firmware:
	cd lasercan-firmware && cargo build --release
	arm-none-eabi-objcopy --update-section .firmware_flag=./firmware_ready_patch.bin ./lasercan-firmware/target/thumbv7m-none-eabi/release/lasercan-firmware ./target/lasercan-firmware-update.elf
	arm-none-eabi-objcopy -O binary ./target/lasercan-firmware-update.elf target/lasercan-firmware-update.grplfw
	python check_bootloader_compat.py

flash_bootloader: bootloader
	cargo flash --chip=STM32F103C8 --elf ./lasercan-bootloader/target/thumbv7m-none-eabi/release/lasercan-bootloader

flash_firmware: firmware
	cargo flash --chip=STM32F103C8 --elf ./lasercan-firmware/target/thumbv7m-none-eabi/release/lasercan-firmware

flash: flash_bootloader flash_firmware