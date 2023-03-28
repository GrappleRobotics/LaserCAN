// use stm32h7xx_hal::{device::IWDG1, flash::LockedFlashBank};

use stm32f1xx_hal::{pac::IWDG, flash::{FlashWriter, FLASH_START}};

pub const FIRMWARE_MAGIC: u32 = 0xCAFEBABE;

pub const FLASH_USER: usize = 0x0800_0000 + 20*1024;
pub const FLASH_META: usize = FLASH_USER + 0x150;
pub const FLASH_META_FIRMWARE_MAGIC_LOC: usize = FLASH_META;
pub const FLASH_META_FIRMWARE_RESET_LOC: usize = FLASH_META + 0x40;

pub fn is_user_code_present() -> bool {
  unsafe { *(FLASH_META_FIRMWARE_MAGIC_LOC as *const u32) == FIRMWARE_MAGIC }
}

pub fn is_firmware_update_in_progress() -> bool {
  unsafe { *(FLASH_META_FIRMWARE_RESET_LOC as *const u32) == 0xFFFF_FFFF }
}

pub fn feed_watchdog(iwdg: &mut IWDG) {
  iwdg.kr.write(|w| w.key().reset());
}

pub fn start_field_upgrade(flash: &mut FlashWriter<'_>) {
  flash.erase(FLASH_META_FIRMWARE_RESET_LOC as u32 - FLASH_START, 1024).unwrap();

  cortex_m::peripheral::SCB::sys_reset();
}
