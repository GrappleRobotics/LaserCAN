all: bootloader firmware

bootloader:
	cd lasercan-bootloader && cargo build --release
	arm-none-eabi-objcopy -O binary ./lasercan-bootloader/target/thumbv7m-none-eabi/release/lasercan-bootloader target/lasercan-bootloader.bin

firmware:
	cd lasercan-firmware && cargo build --release
	arm-none-eabi-objcopy --update-section .firmware_flag=./firmware_ready_patch.bin ./lasercan-firmware/target/thumbv7m-none-eabi/release/lasercan-firmware ./target/lasercan-firmware-update.elf
	arm-none-eabi-objcopy -O binary ./target/lasercan-firmware-update.elf target/lasercan-firmware-update.grplfw

flash_bootloader: bootloader
	cargo flash --chip=STM32F103C8 --elf ./lasercan-bootloader/target/thumbv7m-none-eabi/release/lasercan-bootloader

flash_firmware: firmware
	cargo flash --chip=STM32F103C8 --elf ./lasercan-firmware/target/thumbv7m-none-eabi/release/lasercan-firmware

flash: flash_bootloader flash_firmware