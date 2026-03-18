#![no_main]
#![no_std]

extern crate alloc;

use core::mem::MaybeUninit;

use embedded_alloc::Heap;
use grapple_frc_msgs::grapple::device_info::GrappleModelId;
use lasercan_common::bootutil::FIRMWARE_MAGIC;
use panic_halt as _;

#[link_section = ".metadata.magic"]
#[no_mangle]
pub static META_MAGIC: u32 = FIRMWARE_MAGIC;

#[link_section = ".metadata.version"]
#[no_mangle]
pub static META_VERSION: &'static str = "0.4.0";

#[link_section = ".metadata.model_id"]
#[no_mangle]
pub static META_MODEL_ID: GrappleModelId = GrappleModelId::LaserCan;

pub const BOOTLOADER_IMG: &[u8] = include_bytes!("../../target/lasercan-bootloader-0.4.0.grplbt");

#[global_allocator]
static HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = 4096;

fn heap_init() {
  static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
  unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
}

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true)]
mod app {
  use lasercan_common::bootutil::feed_watchdog;
  use stm32f1xx_hal::{flash::{self, FlashExt}, prelude::*};

  use crate::{heap_init, BOOTLOADER_IMG};

  #[shared]
  struct SharedResources { }
  
  #[local]
  struct LocalResources { }

  #[init]
  fn init(mut ctx: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {
    let mut gpioc = ctx.device.GPIOC.split();

    let pc15_board_rev_detect = gpioc.pc15.into_pull_up_input(&mut gpioc.crh);
    
    let mut flash = ctx.device.FLASH.constrain();

    let _clocks = match pc15_board_rev_detect.is_low() {
      false => {
        // Rev 3 or Earlier (w/o Crystal)
        ctx.device.RCC.constrain().cfgr.freeze(&mut flash.acr)
      },
      true => {
        // Rev 4 or Later (w/ Crystal)
        ctx.device.RCC.constrain().cfgr.use_hse(8.MHz()).freeze(&mut flash.acr)
      }
    };

    heap_init();

    feed_watchdog(&mut ctx.device.IWDG);

    /* FLASH BOOTLOADER */

    let mut flash_writer = flash.writer(flash::SectorSize::Sz1K, flash::FlashSize::Sz64K);

    let mut len = BOOTLOADER_IMG.len() / 1024;
    if BOOTLOADER_IMG.len() % 1024 != 0 {
      len += 1;
    }

    for i in 0..len {
      let offset = i * 1024;
      let mut write_payload = [0u8; 1024];

      for i in 0..(BOOTLOADER_IMG.len() - offset) {
        if i >= write_payload.len() {
          break;
        }
        write_payload[i] = BOOTLOADER_IMG[offset + i]
      }

      flash_writer.erase(offset as u32, 1024).ok();
      flash_writer.write(offset as u32, &write_payload[..]).ok();

      feed_watchdog(&mut ctx.device.IWDG);
    }

    /* DROP TO NEW BOOTLOADER */

    lasercan_common::bootutil::start_field_upgrade(&mut flash_writer);

    (
      SharedResources { },
      LocalResources { },
      init::Monotonics()
    )
  }
}