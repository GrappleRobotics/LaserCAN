#![no_main]
#![no_std]

mod configuration;

extern crate alloc;

use core::{mem::MaybeUninit, sync::atomic::{AtomicUsize, AtomicU32}};

use embedded_alloc::Heap;
use grapple_frc_msgs::grapple::device_info::GrappleModelId;
use lasercan_common::bootutil::FIRMWARE_MAGIC;
use panic_halt as _;
use vl53l1x_uld::{roi::{ROI, ROICenter}, MeasureResult};

#[link_section = ".metadata.magic"]
#[no_mangle]
pub static META_MAGIC: u32 = FIRMWARE_MAGIC;

#[link_section = ".metadata.version"]
#[no_mangle]
pub static META_VERSION: &'static str = env!("CARGO_PKG_VERSION");

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

static TIME_MS: AtomicU32 = AtomicU32::new(0);

// Traits for the sensor, so we can put it in a Box<> and have some generic type erasure.
pub trait Sensor {
  fn data_ready(&mut self) -> Result<bool, ()>;
  fn get_result(&mut self) -> Result<MeasureResult, ()>;
  fn stop_ranging(&mut self) -> Result<(), ()>;
  fn start_ranging(&mut self) -> Result<(), ()>;
  fn set_distance_mode(&mut self, long: bool) -> Result<(), ()>;
  fn set_roi(&mut self, roi: ROI) -> Result<(), ()>;
  fn set_roi_center(&mut self, center: ROICenter) -> Result<(), ()>;
  fn set_timing_budget_ms(&mut self, budget: u16) -> Result<(), ()>;
}

#[inline(always)]
fn erase_result<T, E>(result: Result<T, E>) -> Result<T, ()> {
  match result {
    Ok(v) => Ok(v),
    Err(_) => Err(()),
  }
}

impl<E: core::fmt::Debug, T: vl53l1x_uld::comm::Write<Error = E> + vl53l1x_uld::comm::Read<Error = E>> Sensor for vl53l1x_uld::VL53L1X<T> {
  fn data_ready(&mut self) -> Result<bool, ()> { erase_result(self.is_data_ready()) }
  fn get_result(&mut self) -> Result<MeasureResult, ()> { erase_result(self.get_result()) }
  fn stop_ranging(&mut self) -> Result<(), ()> { erase_result(self.stop_ranging()) }
  fn start_ranging(&mut self) -> Result<(), ()> { erase_result(self.start_ranging()) }
  fn set_distance_mode(&mut self, long: bool) -> Result<(), ()> { erase_result(self.set_distance_mode(if long { vl53l1x_uld::DistanceMode::Long } else { vl53l1x_uld::DistanceMode::Short }))}
  fn set_roi(&mut self, roi: ROI) -> Result<(), ()> { erase_result(self.set_roi(roi)) }
  fn set_roi_center(&mut self, center: ROICenter) -> Result<(), ()> { erase_result(self.set_roi_center(center)) }
  fn set_timing_budget_ms(&mut self, budget: u16) -> Result<(), ()> { erase_result(self.set_timing_budget_ms(budget)) }
}

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true)]
mod app {
  use core::{marker::PhantomData, ops::Deref};

  use alloc::{collections::VecDeque, borrow::ToOwned};
  use bxcan::{ExtendedId, filter::Mask32, Interrupts, Tx, Rx0};
  use shared_bus::new_cortexm;
  use vl53l1x_uld::{VL53L1X, roi::{ROI, ROICenter}, MeasureResult};
  use crate::{configuration::LaserCanConfiguration, META_VERSION, META_MODEL_ID};
  use grapple_m24c64::M24C64;
  use grapple_config::GenericConfigurationProvider;
  use grapple_frc_msgs::{binmarshal::BinMarshal, grapple::lasercan::{self, LaserCanStatusFrame}};
  use lasercan_common::bootutil::feed_watchdog;
  use stm32f1xx_hal::{prelude::*, flash::{FlashExt, self}, pac::{IWDG, TIM1, CAN1, TIM2, TIM3, I2C1, Interrupt, TIM4}, timer::{self, CounterMs, CounterHz, SysCounterHz, SysCounterUs}, gpio::{ErasedPin, Output, Pin, Alternate, OpenDrain}, can::Can, serial::Rx, i2c::{I2c, Mode, BlockingI2c}, time::MonoTimer};
  use grapple_frc_msgs::{DEVICE_ID_BROADCAST, DEVICE_TYPE_BROADCAST, DEVICE_TYPE_FIRMWARE_UPGRADE, Validate, ManufacturerMessage, Message};
  use grapple_frc_msgs::grapple::*;
  use grapple_frc_msgs::can::{CANId, UnparsedCANMessage, CANMessage};

  use crate::heap_init;

  #[shared]
  struct SharedResources {
    blink_timer: u32,
    status_led: ErasedPin<Output>,
    config: alloc::boxed::Box<dyn grapple_config::GenericConfigurationProvider<LaserCanConfiguration> + Send>,
    sensor: alloc::boxed::Box<dyn crate::Sensor + Send>,
    last_result: Option<MeasureResult>,
    can_tx_queue: VecDeque<bxcan::Frame>,
  }
  
  #[local]
  struct LocalResources {
    watchdog: IWDG,
    led_timer: CounterMs<TIM1>,
    status_timer: CounterHz<TIM2>,
    can_rx: Rx0<Can<CAN1>>,
    can_tx: Tx<Can<CAN1>>,
    flash: flash::Parts,
    reassemble: grapple_frc_msgs::can::FragmentReassembler
  }

  #[init]
  fn init(mut ctx: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {
    let mut flash = ctx.device.FLASH.constrain();
    let clocks = ctx.device.RCC.constrain().cfgr.freeze(&mut flash.acr);

    feed_watchdog(&mut ctx.device.IWDG);

    let mut gpioa = ctx.device.GPIOA.split();
    let mut gpiob = ctx.device.GPIOB.split();
    let mut afio = ctx.device.AFIO.constrain();
    let (pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    /* TIMER INIT */
    let mut systick = ctx.core.SYST.counter_us(&clocks);
    systick.start(100.millis()).unwrap();
    systick.listen(timer::SysEvent::Update);

    /* LED INIT */

    let mut status_led = gpioa.pa8.into_push_pull_output(&mut gpioa.crh);
    status_led.set_low();

    let mut led_timer = ctx.device.TIM1.counter_ms(&clocks);
    led_timer.start(100.millis()).unwrap();
    led_timer.listen(timer::Event::Update);

    heap_init();

    /* I2C INIT */
    let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
    let i2c = BlockingI2c::i2c1(
      ctx.device.I2C1,
      (scl, sda),
      &mut afio.mapr,
      Mode::Fast { frequency: 400.kHz(), duty_cycle: stm32f1xx_hal::i2c::DutyCycle::Ratio16to9 },
      clocks,
      1000,
      5,
      1000,
      1000
    );
    
    // This is ugly, but what can you do?
    let i2c_bus = new_cortexm!(BlockingI2c<I2C1, ( stm32f1xx_hal::gpio::gpiob::PB6<Alternate<OpenDrain>>, stm32f1xx_hal::gpio::gpiob::PB7<Alternate<OpenDrain>> )> = i2c).unwrap();
    
    /* CONFIGURATION INIT */

    let config_delay = ctx.device.TIM3.delay_ms(&clocks);
    let eeprom = M24C64::new(i2c_bus.acquire_i2c(), 0b000);
    let config_marshal = grapple_config::m24c64::M24C64ConfigurationMarshal::new(eeprom, 0, config_delay, PhantomData::<LaserCanConfiguration>);
    let mut config_provider = alloc::boxed::Box::new(grapple_config::ConfigurationProvider::new(config_marshal).ok().unwrap());
    
    if !config_provider.current().validate() {
      *config_provider.current_mut() = LaserCanConfiguration::default();
      config_provider.commit();
    }
    
    /* CAN INIT */
    
    let can = Can::new(ctx.device.CAN1, ctx.device.USB);
    let rx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
    let tx = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);
    
    can.assign_pins((tx, rx), &mut afio.mapr);
    
    let mut can_bus = bxcan::Can::builder(can)
      .set_bit_timing(0x0005_0000)
      .leave_disabled();
    
    unsafe {
      can_bus.modify_filters()
      // Firmware Update Messages
        .enable_bank(0, bxcan::Fifo::Fifo0, Mask32::frames_with_ext_id(
          ExtendedId::new_unchecked(CANId { device_type: DEVICE_TYPE_FIRMWARE_UPGRADE, manufacturer: MANUFACTURER_GRAPPLE, api_class: 0x00, api_index: 0x00, device_id: DEVICE_ID_BROADCAST }.into()),
          ExtendedId::new_unchecked(CANId { device_type: 0xFF, manufacturer: 0xFF, api_class: 0x00, api_index: 0x00, device_id: 0xFF }.into()),
        ))
        // Broadcast Messages
        .enable_bank(1, bxcan::Fifo::Fifo0, Mask32::frames_with_ext_id(
          ExtendedId::new_unchecked(CANId { device_type: DEVICE_TYPE_BROADCAST, manufacturer: MANUFACTURER_GRAPPLE, api_class: 0x00, api_index: 0x00, device_id: DEVICE_ID_BROADCAST }.into()),
          ExtendedId::new_unchecked(CANId { device_type: 0xFF, manufacturer: 0xFF, api_class: 0x00, api_index: 0x00, device_id: 0xFF }.into()),
        ))
        // Specific to this sensor. Note we don't check for Device ID here, since it may change whilst we're still booted. 
        .enable_bank(2, bxcan::Fifo::Fifo0, Mask32::frames_with_ext_id(
          ExtendedId::new_unchecked(CANId { device_type: DEVICE_TYPE_DISTANCE_SENSOR, manufacturer: MANUFACTURER_GRAPPLE, api_class: 0x00, api_index: 0x00, device_id: 0x00 }.into()),
          ExtendedId::new_unchecked(CANId { device_type: 0xFF, manufacturer: 0xFF, api_class: 0x00, api_index: 0x00, device_id: 0x00 }.into()),
        ));
    }
      
    can_bus.enable_interrupts(Interrupts::TRANSMIT_MAILBOX_EMPTY | Interrupts::FIFO0_MESSAGE_PENDING);
    nb::block!(can_bus.enable_non_blocking()).unwrap();
    
    let (can_tx, can_rx, _) = can_bus.split();
    
    let mut status_timer = ctx.device.TIM2.counter_hz(&clocks);
    status_timer.start(50.Hz()).unwrap();
    status_timer.listen(timer::Event::Update);
    
    feed_watchdog(&mut ctx.device.IWDG);
    
    /* SENSOR INIT */
    let mut delay = ctx.device.TIM4.delay_ms(&clocks);

    let mut tof_shut = pb4.into_push_pull_output(&mut gpiob.crl);
    tof_shut.set_high();  // Enable sensor (shutdown is active low)
    
    // Give the sensor some time to start up
    delay.delay_ms(20u16);

    let mut sensor = alloc::boxed::Box::new(VL53L1X::new(i2c_bus.acquire_i2c(), vl53l1x_uld::DEFAULT_ADDRESS));
    if sensor.get_sensor_id().unwrap() != 0xEACC {
      panic!("Wrong Sensor ID!");
    }

    sensor.init(vl53l1x_uld::IOVoltage::Volt2_8).unwrap();
    sensor.start_ranging().unwrap();
    
    apply_configuration(sensor.as_mut(), config_provider.current());

    status_led.set_high();
    (
      SharedResources {
        blink_timer: 20,
        status_led: status_led.erase(),
        config: config_provider,
        sensor,
        last_result: None,
        can_tx_queue: VecDeque::with_capacity(64)
      },
      LocalResources {
        watchdog: ctx.device.IWDG,
        led_timer,
        status_timer,
        can_rx,
        can_tx,
        flash,
        reassemble: grapple_frc_msgs::can::FragmentReassembler::new(1000)
      },
      init::Monotonics()
    )
  }

  fn apply_configuration(sensor: &mut dyn crate::Sensor, config: &LaserCanConfiguration) {
    sensor.stop_ranging().unwrap();
    sensor.set_distance_mode(config.long).unwrap();
    sensor.set_roi(ROI::new(config.roi.w.0 as u16, config.roi.h.0 as u16)).unwrap();
    sensor.set_roi_center(ROICenter::new(config.roi.x.0, config.roi.y.0)).unwrap();
    sensor.set_timing_budget_ms(config.timing_budget as u16).unwrap();
    sensor.start_ranging().unwrap();
  }

  #[task(binds = SysTick, priority=15, local=[])]
  fn systick_tick(_: systick_tick::Context) {
    crate::TIME_MS.fetch_add(100, core::sync::atomic::Ordering::Relaxed);
  }

  #[idle(shared = [], local = [])]
  fn idle(ctx: idle::Context) -> ! {
    loop {
    }
  }

  #[task(binds = TIM1_UP, priority = 1, shared = [blink_timer, status_led], local = [led_timer])]
  fn led_tick(ctx: led_tick::Context) {
    (ctx.shared.blink_timer, ctx.shared.status_led).lock(|timer, led| {
      if *timer > 0 {
        led.toggle();
        *timer -= 1;
      } else {
        led.set_high();
      }
    });

    ctx.local.led_timer.clear_interrupt(timer::Event::Update);
  }

  #[task(binds = TIM2, priority = 15, shared = [sensor, last_result, can_tx_queue, config], local = [status_timer, watchdog])]
  fn status_tick(ctx: status_tick::Context) {
    feed_watchdog(ctx.local.watchdog);

    (ctx.shared.sensor, ctx.shared.can_tx_queue, ctx.shared.last_result, ctx.shared.config).lock(|sensor, q, result, config| {
      if Ok(true) == sensor.data_ready() {
        if let Ok(r) = sensor.get_result() {
          enqueue(CANMessage::Message(
            Message::new(
              config.current().device_id,
              ManufacturerMessage::Grapple(GrappleDeviceMessage::DistanceSensor(
                lasercan::LaserCanMessage::Status(LaserCanStatusFrame { status: r.status as u8, distance_mm: r.distance_mm, ambient: r.ambient, long: config.current().long, budget_ms: config.current().timing_budget, roi: config.current().roi.clone() })
              ))
            )
          ), q);

          *result = Some(r);
        }
      }
    });
    ctx.local.status_timer.clear_interrupt(timer::Event::Update);
  }

  #[task(binds = USB_HP_CAN_TX, priority = 13, local = [can_tx], shared = [can_tx_queue, status_led])]
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

  #[task(binds = USB_LP_CAN_RX0, shared = [can_tx_queue, config, blink_timer, sensor], local = [can_rx, flash, reassemble])]
  fn can_rx(mut ctx: can_rx::Context) {
    let my_serial = lasercan_common::get_serial_hash();

    ctx.shared.config.lock(|cfg| {
      loop {
        match ctx.local.can_rx.receive() {
          Ok(frame) => match (frame.id(), frame.data()) {
            (bxcan::Id::Extended(ext), data) => {
              let d = data.map(|x| x.deref()).unwrap_or(&[]);
              let msg = CANMessage::from(UnparsedCANMessage::new(ext.as_raw(), d));
              let reassembled = ctx.local.reassemble.process(crate::TIME_MS.load(core::sync::atomic::Ordering::Relaxed) as i64, d.len() as u8, msg);
              match reassembled {
                Some((_, msg)) => {
                  match msg {
                    CANMessage::Message(msg) => {
                      if msg.validate().is_ok() {
                        match msg.msg {
                          ManufacturerMessage::Grapple(GrappleDeviceMessage::Broadcast(bmsg)) => match bmsg {
                            GrappleBroadcastMessage::DeviceInfo(di) => match di {
                              device_info::GrappleDeviceInfo::EnumerateRequest => {
                                let new_msg = Message::new(
                                  cfg.current().device_id,
                                  ManufacturerMessage::Grapple(GrappleDeviceMessage::Broadcast(
                                    GrappleBroadcastMessage::DeviceInfo(device_info::GrappleDeviceInfo::EnumerateResponse {
                                      model_id: META_MODEL_ID.clone(),
                                      serial: my_serial,
                                      is_dfu: false,
                                      is_dfu_in_progress: false,
                                      version: META_VERSION.to_owned(),
                                      name: cfg.current().name.clone()
                                    })
                                  ))
                                );

                                ctx.shared.can_tx_queue.lock(|q| enqueue(CANMessage::Message(new_msg), q));
                              },
                              device_info::GrappleDeviceInfo::Blink { serial } if serial == my_serial => {
                                ctx.shared.blink_timer.lock(|timer| { *timer = 30 });
                              },
                              device_info::GrappleDeviceInfo::SetName { serial, name } if serial == my_serial => {
                                cfg.current_mut().name = name;
                                cfg.commit();
                              },
                              device_info::GrappleDeviceInfo::SetId { serial, new_id } if serial == my_serial => {
                                cfg.current_mut().device_id = new_id;
                                cfg.commit();
                              },
                              device_info::GrappleDeviceInfo::CommitConfig { serial } if serial == my_serial => {
                                cfg.commit();
                              },
                              _ => ()
                            },
                          },
                          ManufacturerMessage::Grapple(GrappleDeviceMessage::FirmwareUpdate(fwupdate)) => match fwupdate {
                            firmware::GrappleFirmwareMessage::StartFieldUpgrade { serial } if serial == my_serial => {
                              let mut writer = ctx.local.flash.writer(flash::SectorSize::Sz1K, flash::FlashSize::Sz64K);
                              lasercan_common::bootutil::start_field_upgrade(&mut writer);
                            },
                            _ => ()
                          },
                          ManufacturerMessage::Grapple(GrappleDeviceMessage::DistanceSensor(dist)) if msg.device_id == cfg.current().device_id => match dist {
                            lasercan::LaserCanMessage::SetRange { long } => {
                              cfg.current_mut().long = long;
                              ctx.shared.sensor.lock(|sensor| apply_configuration(sensor.as_mut(), cfg.current()));
                            },
                            lasercan::LaserCanMessage::SetRoi { roi } => {
                              cfg.current_mut().roi = roi;
                              ctx.shared.sensor.lock(|sensor| apply_configuration(sensor.as_mut(), cfg.current()));
                            },
                            lasercan::LaserCanMessage::SetTimingBudget { budget } => {
                              cfg.current_mut().timing_budget = budget.clamp(20, 127);
                              ctx.shared.sensor.lock(|sensor| apply_configuration(sensor.as_mut(), cfg.current()));
                            },
                            _ => ()
                          }
                          _ => ()
                        }
                      }
                    },
                    _ => ()
                  }
                },
                _ => ()
              }
            },
            _ => ()
          },
          Err(nb::Error::WouldBlock) => break,
          Err(nb::Error::Other(_)) => {}    // Ignore overrun errors
        }
      }
    });
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