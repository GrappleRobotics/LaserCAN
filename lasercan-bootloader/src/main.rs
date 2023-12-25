#![no_main]
#![no_std]

use core::mem::MaybeUninit;

use embedded_alloc::Heap;
use grapple_frc_msgs::grapple::device_info::GrappleModelId;
use panic_halt as _;

extern crate alloc;

#[link_section = ".metadata.magic"]
#[no_mangle]
pub static META_MAGIC: u32 = 0xABCD1234;

#[link_section = ".metadata.version"]
#[no_mangle]
pub static META_BOOTLOADER_VERSION: &'static str = env!("CARGO_PKG_VERSION");

#[link_section = ".metadata.model_id"]
#[no_mangle]
pub static META_MODEL_ID: GrappleModelId = GrappleModelId::LaserCan;

#[global_allocator]
static HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = 4096;

fn heap_init() {
  static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
  unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
}

const WATCHDOG_COUNTER_ADDR: usize = 0x2000_0000;

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true)]
mod app {
  use alloc::{collections::VecDeque, borrow::ToOwned};
  use bxcan::{ExtendedId, filter::Mask32, Interrupts, Rx0, Tx};
  use core::ops::Deref;
  use cortex_m::peripheral::SCB;
  use grapple_frc_msgs::{can::{CANId, CANMessage, FragmentReassembler, UnparsedCANMessage}, DEVICE_TYPE_FIRMWARE_UPGRADE, grapple::{MANUFACTURER_GRAPPLE, GrappleBroadcastMessage, device_info::GrappleDeviceInfo, GrappleDeviceMessage, firmware::GrappleFirmwareMessage}, DEVICE_ID_BROADCAST, DEVICE_TYPE_BROADCAST, binmarshal::BinMarshal, ManufacturerMessage, Message};
  use lasercan_common::bootutil::{is_firmware_update_in_progress, is_user_code_present, FLASH_USER, FLASH_META_FIRMWARE_RESET_LOC};
  use stm32f1xx_hal::{prelude::*, watchdog::IndependentWatchdog, can::Can, pac::{CAN1, Interrupt}, gpio::{ErasedPin, Output}, flash::{self, FLASH_START}};
  use tiny_rng::Rand;

  use crate::{WATCHDOG_COUNTER_ADDR, heap_init, META_MODEL_ID, META_BOOTLOADER_VERSION};

  #[shared]
  struct SharedResources {
    can_tx_queue: VecDeque<bxcan::Frame>,
    status_led: ErasedPin<Output>,
    device_id: u8,
  }

  #[local]
  struct LocalResources {
    can_rx: Rx0<Can<CAN1>>,
    can_tx: Tx<Can<CAN1>>,
    reassemble: FragmentReassembler,
    flash: flash::Parts,
    offset: usize,
    rng: tiny_rng::Rng
  }

  #[init]
  fn init(mut ctx: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {
    // Check if the reset was due to a watchdog timeout. After repeated timeouts, we'll 
    // drop to a firmware update prompt since it's clear the user program is not working
    // sufficiently.
    // We can't use RCC.reset_reason since that requires us to constrain the RCC, which 
    // we can't do yet. 
    let was_iwdg_reset = ctx.device.RCC.csr.read().iwdgrstf().is_reset();
    ctx.device.RCC.csr.modify(|_, w| w.rmvf().clear());

    let mut has_watchdog_timed_out = false;

    if !was_iwdg_reset {
      unsafe { core::ptr::write_volatile(WATCHDOG_COUNTER_ADDR as *mut u32, 0x00) }
    } else {
      let count = unsafe { core::ptr::read_volatile(WATCHDOG_COUNTER_ADDR as *mut u32) };
      unsafe { core::ptr::write_volatile(WATCHDOG_COUNTER_ADDR as *mut u32, count + 1) }

      if count >= 5 {
        has_watchdog_timed_out = true;
      }
    }

    // Jump to the user code if we don't need to be in the bootloader
    if is_user_code_present() && !is_firmware_update_in_progress() && !has_watchdog_timed_out {
      // Jump to user code, with watchdog
      let mut watchdog = IndependentWatchdog::new(ctx.device.IWDG);
      watchdog.start(3000.millis());
      watchdog.feed();

      jump_to_user_code(&mut ctx.core.SCB);
    }

    // We're in the bootloader :)

    let mut flash = ctx.device.FLASH.constrain();
    let clocks = ctx.device.RCC.constrain().cfgr.freeze(&mut flash.acr);
    
    let mut gpioa = ctx.device.GPIOA.split();

    let mut status_led = gpioa.pa8.into_push_pull_output(&mut gpioa.crh);
    status_led.set_low();

    heap_init();

    let mut afio = ctx.device.AFIO.constrain();

    let can = Can::new(ctx.device.CAN1, ctx.device.USB);
    let rx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
    let tx = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);

    can.assign_pins((tx, rx), &mut afio.mapr);

    let mut can_bus = bxcan::Can::builder(can)
      .set_bit_timing(0x0005_0000)
      .leave_disabled();

    unsafe {
      can_bus.modify_filters()
        .enable_bank(0, bxcan::Fifo::Fifo0, Mask32::frames_with_ext_id(
          ExtendedId::new_unchecked(CANId { device_type: DEVICE_TYPE_FIRMWARE_UPGRADE, manufacturer: MANUFACTURER_GRAPPLE, api_class: 0x00, api_index: 0x00, device_id: 0x00 }.into()),
          ExtendedId::new_unchecked(CANId { device_type: 0xFF, manufacturer: 0xFF, api_class: 0x00, api_index: 0x00, device_id: 0x00 }.into()),
        ))
        .enable_bank(1, bxcan::Fifo::Fifo0, Mask32::frames_with_ext_id(
          ExtendedId::new_unchecked(CANId { device_type: DEVICE_TYPE_BROADCAST, manufacturer: MANUFACTURER_GRAPPLE, api_class: 0x00, api_index: 0x00, device_id: 0x00 }.into()),
          ExtendedId::new_unchecked(CANId { device_type: 0xFF, manufacturer: 0xFF, api_class: 0x00, api_index: 0x00, device_id: 0x00 }.into()),
        ));
    }

    can_bus.enable_interrupts(Interrupts::TRANSMIT_MAILBOX_EMPTY | Interrupts::FIFO0_MESSAGE_PENDING);
    nb::block!(can_bus.enable_non_blocking()).unwrap();

    let (can_tx, can_rx, _) = can_bus.split();

    let mut delay = ctx.device.TIM3.delay_ms(&clocks);
    let mut rng = tiny_rng::Rng::from_seed(lasercan_common::get_serial_hash() as u64);

    // Delay for arbitration
    delay.delay_ms(rng.rand_range_u32(1, 50) * 20);

    // Perform arbitration
    let mut queue = VecDeque::with_capacity(16);

    enqueue(CANMessage::Message(Message::new(
      0,
      ManufacturerMessage::Grapple(GrappleDeviceMessage::Broadcast(
        GrappleBroadcastMessage::DeviceInfo(GrappleDeviceInfo::ArbitrationRequest)
      ))
    )), &mut queue);

    (
      SharedResources {
        device_id: 0,
        can_tx_queue: queue,
        status_led: status_led.erase()
      },
      LocalResources {
        can_rx, can_tx, flash,
        reassemble: FragmentReassembler::new(1000),
        offset: 0,
        rng
      },
      init::Monotonics()
    )
  }

  #[idle(shared = [], local = [])]
  fn idle(ctx: idle::Context) -> ! {
    loop { }
  }

  #[task(binds = USB_HP_CAN_TX, priority = 13, local = [can_tx], shared = [can_tx_queue])]
  fn can_tx(mut ctx: can_tx::Context) {
    let tx = ctx.local.can_tx;

    tx.clear_interrupt_flags();

    ctx.shared.can_tx_queue.lock(|q| {
      while let Some(frame) = q.front() {
        match tx.transmit(&frame) {
          Ok(status) => match status.dequeued_frame() {
            Some(pending) => {
              q.pop_front();
              q.push_back(pending.clone());
              rtic::pend(Interrupt::USB_HP_CAN_TX);
            }
            None => {
              q.pop_front();
            },
          },
          Err(nb::Error::WouldBlock) => break,
          Err(_) => unreachable!()
        }
      }
    });
  }

  #[task(binds = USB_LP_CAN_RX0, shared = [can_tx_queue, status_led, device_id], local = [can_rx, flash, reassemble, offset, rng])]
  fn can_rx(mut ctx: can_rx::Context) {
    let my_serial = lasercan_common::get_serial_hash();

    let my_id = ctx.shared.device_id.lock(|did| *did);

    loop {
      match ctx.local.can_rx.receive() {
        Ok(frame) => match (frame.id(), frame.data()) {
          (bxcan::Id::Extended(ext), data) => {
            let d = data.map(|x| x.deref()).unwrap_or(&[]);
            let unparsed = UnparsedCANMessage::new(ext.as_raw(), d);
            if unparsed.id.device_id == DEVICE_ID_BROADCAST || unparsed.id.device_id == my_id {
              let msg = CANMessage::from(unparsed);
              let reassembled = ctx.local.reassemble.process(0 as i64, d.len() as u8, msg);
              match reassembled {
                Some((_, msg)) => match msg {
                  CANMessage::Message(msg) => {
                    match msg.msg {
                      ManufacturerMessage::Grapple(GrappleDeviceMessage::Broadcast(bmsg)) => match bmsg {
                        GrappleBroadcastMessage::DeviceInfo(di) => match di {
                          GrappleDeviceInfo::ArbitrationRequest if msg.device_id == my_id => {
                            // Reply to the arbitration request with a rejection, since this ID belongs to us.
                            let msg = Message::new(
                              my_id,
                              ManufacturerMessage::Grapple(GrappleDeviceMessage::Broadcast(
                                GrappleBroadcastMessage::DeviceInfo(GrappleDeviceInfo::ArbitrationReject)
                              ))
                            );
                            ctx.shared.can_tx_queue.lock(|q| enqueue(CANMessage::Message(msg), q));
                          },
                          GrappleDeviceInfo::ArbitrationReject if msg.device_id == my_id => {
                            // Retry Arbitration.
                            let msg = ctx.shared.device_id.lock(|did| {
                              *did = ctx.local.rng.rand_u8() & 0b11111;
                              Message::new(
                                *did,
                                ManufacturerMessage::Grapple(GrappleDeviceMessage::Broadcast(
                                  GrappleBroadcastMessage::DeviceInfo(GrappleDeviceInfo::ArbitrationRequest)
                                ))
                              )
                            });
                            
                            ctx.shared.can_tx_queue.lock(|q| enqueue(CANMessage::Message(msg), q));
                          },
                          GrappleDeviceInfo::EnumerateRequest => {
                            let new_msg = Message::new(
                              my_id,
                              ManufacturerMessage::Grapple(GrappleDeviceMessage::Broadcast(
                                GrappleBroadcastMessage::DeviceInfo(GrappleDeviceInfo::EnumerateResponse {
                                  model_id: META_MODEL_ID.clone(),
                                  serial: my_serial,
                                  is_dfu: true,
                                  is_dfu_in_progress: *ctx.local.offset != 0,
                                  version: META_BOOTLOADER_VERSION.to_owned(),
                                  name: "".to_owned()
                                })
                            )
                            ));
                            ctx.shared.can_tx_queue.lock(|q| enqueue(CANMessage::Message(new_msg), q));
                          },
                          _ => ()
                        },
                      },
                      ManufacturerMessage::Grapple(GrappleDeviceMessage::FirmwareUpdate(fwupdate)) => match fwupdate {
                        GrappleFirmwareMessage::StartFieldUpgrade { .. } => (),
                        GrappleFirmwareMessage::UpdatePart(data) => {
                          ctx.shared.status_led.lock(|led| led.toggle());

                          let mut flash_writer = ctx.local.flash.writer(flash::SectorSize::Sz1K, flash::FlashSize::Sz64K);
                          let user_code_offset = FLASH_USER as usize - FLASH_START as usize;

                          // Erase the sector if required. Obviously, this requires our data step size to be a factor of 1024. 
                          if (user_code_offset + *ctx.local.offset) as u32 % 1024 == 0 {
                            // Erase the flash region
                            flash_writer.erase((user_code_offset + *ctx.local.offset) as u32, 1024).unwrap();
                          }

                          // Write the flash data (must be aligned to 16 bits)
                          flash_writer.write((user_code_offset + *ctx.local.offset) as u32, &data[..]).unwrap();

                          *ctx.local.offset += data.len() as usize;

                          // Reply with an ACK
                          let new_msg = Message::new(
                            my_id,
                            ManufacturerMessage::Grapple(GrappleDeviceMessage::FirmwareUpdate(
                              GrappleFirmwareMessage::UpdatePartAck
                            ))
                          );
                          ctx.shared.can_tx_queue.lock(|q| enqueue(CANMessage::Message(new_msg), q));
                          // send_can_message(&mut can_bus, CANMessage::Message(new_msg));
                        },
                        GrappleFirmwareMessage::UpdateDone => {
                          let mut flash_writer = ctx.local.flash.writer(flash::SectorSize::Sz1K, flash::FlashSize::Sz64K);
                          ctx.shared.status_led.lock(|led| led.set_high());

                          // /* Mark the firmware as good to go */
                          flash_writer.write(FLASH_META_FIRMWARE_RESET_LOC as u32 - FLASH_START, &[0xDE, 0xAD, 0xBE, 0xEF]).unwrap();

                          cortex_m::peripheral::SCB::sys_reset();
                        },
                        _ => ()
                      },
                      _ => ()
                    }
                  },
                  _ => ()
                },
                _ => ()
              }
            }
          },
          _ => ()
        },
        Err(nb::Error::WouldBlock) => break,
        Err(_) => {}
      }
    }
  }

  fn jump_to_user_code(scb: &mut SCB) -> ! {
    unsafe {
      // Remap the vector interrupt table
      scb.vtor.write(FLASH_USER as u32);

      // Call the user program
      let rv = *((FLASH_USER + 4) as *const usize);
      let func = core::mem::transmute::<usize, extern "C" fn() -> !>(rv);
      func();
    };
  }

  fn enqueue(mut msg: CANMessage, q: &mut VecDeque<bxcan::Frame>) {
    static FRAG_ID: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);
    msg.update(());
    
    match msg {
      CANMessage::Message(msg) => {
        let frag_id = FRAG_ID.load(core::sync::atomic::Ordering::Relaxed);
        let msgs = grapple_frc_msgs::can::FragmentReassembler::maybe_split(msg, frag_id).unwrap();
        
        for msg in msgs {
          let frame = unsafe {
            bxcan::Frame::new_data(
              ExtendedId::new_unchecked(Into::<u32>::into(msg.id.clone())),
              bxcan::Data::new(&msg.payload[0..msg.len as usize]).unwrap()
            )
          };

          q.push_back(frame);
        }
        rtic::pend(Interrupt::USB_HP_CAN_TX);

        FRAG_ID.store(frag_id.wrapping_add(1), core::sync::atomic::Ordering::Relaxed);
      },
      _ => panic!("Can't send fragments or generics directly - use the FragmentReasssembler.")
    }
  }
}