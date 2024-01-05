#![no_main]
#![no_std]

extern crate alloc;

use core::{mem::MaybeUninit, sync::atomic::AtomicU32};

use alloc::{format, collections::VecDeque};
use bxcan::{filter::Mask32, ExtendedId};
use embedded_alloc::Heap;
use grapple_lasercan::{grapple_frc_msgs::{binmarshal::AsymmetricCow, grapple::{device_info::GrappleModelId, errors::{GrappleResult, GrappleError}, lasercan}}, Sensor, Watchdog, SysTick, DropToBootloader, CanBus, InputOutput};
use lasercan_common::bootutil::{FIRMWARE_MAGIC, feed_watchdog};
use panic_halt as _;
use stm32f1xx_hal::{pac::{IWDG, CAN1, Interrupt}, flash, can::Can, gpio::{ErasedPin, Output}};

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

macro_rules! convert_sensor_err {
  ($ex:expr) => {
    $ex.map_err(|e| GrappleError::Generic(AsymmetricCow(alloc::borrow::Cow::Owned(format!("{:?}", e)))))
  }
}

pub struct VL53Sensor<E: core::fmt::Debug, T: vl53l1x_uld::comm::Write<Error = E> + vl53l1x_uld::comm::Read<Error = E>> {
  pub sensor: vl53l1x_uld::VL53L1X<T>,
  pub mode: lasercan::LaserCanRangingMode,
  pub roi: lasercan::LaserCanRoi,
  pub budget: lasercan::LaserCanTimingBudget,
}

impl <E: core::fmt::Debug, T: vl53l1x_uld::comm::Write<Error = E> + vl53l1x_uld::comm::Read<Error = E>> VL53Sensor<E, T> {
  pub fn new(sensor: vl53l1x_uld::VL53L1X<T>) -> Self {
    Self {
      sensor,
      mode: lasercan::LaserCanRangingMode::Short,
      roi: lasercan::LaserCanRoi { x: lasercan::LaserCanRoiU4(8), y: lasercan::LaserCanRoiU4(8), w: lasercan::LaserCanRoiU4(16), h: lasercan::LaserCanRoiU4(16) },
      budget: lasercan::LaserCanTimingBudget::TB33ms,
    }
  }

  pub fn init(&mut self) -> GrappleResult<'static, ()> {
    convert_sensor_err!(self.sensor.init(vl53l1x_uld::IOVoltage::Volt2_8))?;
    self.set_ranging_mode(self.mode.clone())?;
    self.set_roi(self.roi.clone())?;
    self.set_timing_budget_ms(self.budget.clone())?;
    self.start_ranging()?;
    Ok(())
  }
}

impl <E: core::fmt::Debug, T: vl53l1x_uld::comm::Write<Error = E> + vl53l1x_uld::comm::Read<Error = E>> Sensor for VL53Sensor<E, T> {
  fn data_ready(&mut self) -> GrappleResult<'static, bool> {
    convert_sensor_err!(self.sensor.is_data_ready())
  }

  fn get_result(&mut self) -> GrappleResult<'static, lasercan::LaserCanMeasurement> {
    let result = convert_sensor_err!(self.sensor.get_result())?;
    Ok(lasercan::LaserCanMeasurement {
      status: result.status as u8,
      distance_mm: result.distance_mm,
      ambient: result.ambient,
      mode: self.mode.clone(),
      budget: self.budget.clone(),
      roi: self.roi.clone(),
    })
  }

  fn stop_ranging(&mut self) -> GrappleResult<'static, ()> {
    convert_sensor_err!(self.sensor.stop_ranging())
  }

  fn start_ranging(&mut self) -> GrappleResult<'static, ()> {
    convert_sensor_err!(self.sensor.start_ranging())
  }

  fn set_ranging_mode(&mut self, mode: lasercan::LaserCanRangingMode) -> GrappleResult<'static, ()> {
    convert_sensor_err!(self.sensor.set_distance_mode(match mode {
      lasercan::LaserCanRangingMode::Short => vl53l1x_uld::DistanceMode::Short,
      lasercan::LaserCanRangingMode::Long => vl53l1x_uld::DistanceMode::Long,
    }))?;
    self.mode = mode;
    Ok(())
  }

  fn set_roi(&mut self, roi: lasercan::LaserCanRoi) -> GrappleResult<'static, ()> {
    convert_sensor_err!(self.sensor.set_roi(vl53l1x_uld::roi::ROI::new(roi.w.0 as u16, roi.h.0 as u16)))?;
    convert_sensor_err!(self.sensor.set_roi_center(vl53l1x_uld::roi::ROICenter::new(roi.x.0, roi.y.0)))?;
    self.roi = roi;
    Ok(())
  }

  fn set_timing_budget_ms(&mut self, budget: lasercan::LaserCanTimingBudget) -> GrappleResult<'static, ()> {
    convert_sensor_err!(self.sensor.set_timing_budget_ms(budget.clone() as u8 as u16))?;
    self.budget = budget;
    Ok(())
  }
}

pub struct WatchdogImpl {
  pub watchdog: IWDG
}

impl Watchdog for WatchdogImpl {
  fn feed(&mut self) {
    feed_watchdog(&mut self.watchdog);
  }
}

pub struct SysTickImpl;

impl SysTick for SysTickImpl {
  fn time(&self) -> u32 {
    TIME_MS.load(core::sync::atomic::Ordering::Relaxed)
  }
}

pub struct DropToBootloaderImpl {
  pub flash: flash::Parts
}

impl DropToBootloader for DropToBootloaderImpl {
  fn drop_to_bootloader(&mut self) {
    let mut writer = self.flash.writer(flash::SectorSize::Sz1K, flash::FlashSize::Sz64K);
    lasercan_common::bootutil::start_field_upgrade(&mut writer);
  }
}

pub struct InputOutputImpl {
  pub led: ErasedPin<Output>
}

impl InputOutput for InputOutputImpl {
  fn set_led(&mut self, value: bool) {
    self.led.set_state(match value {
      true => stm32f1xx_hal::gpio::PinState::Low,
      false => stm32f1xx_hal::gpio::PinState::High,
    });
  }

  fn toggle_led(&mut self) {
    self.led.toggle();
  }
}

pub struct CanBusImpl {
  pub can: bxcan::Can<Can<CAN1>>,
  pub queue: VecDeque<bxcan::Frame>,
  pub next_bank: u8
}

impl CanBusImpl {
  pub fn new(can_bus: bxcan::Can<Can<CAN1>>) -> Self {
    Self { can: can_bus, queue: VecDeque::with_capacity(64), next_bank: 0 }
  }
}

impl CanBus for CanBusImpl {
  const MAX_MSG_SIZE: usize = 8;

  fn subscribe(&mut self, id: grapple_lasercan::grapple_frc_msgs::MessageId, mask: grapple_lasercan::grapple_frc_msgs::MessageId) {
    unsafe {
      self.can.modify_filters()
        .enable_bank(self.next_bank, bxcan::Fifo::Fifo0, Mask32::frames_with_ext_id(
          ExtendedId::new_unchecked(id.into()),
          ExtendedId::new_unchecked(mask.into())
        ));
    }
    
    self.next_bank += 1;
  }

  fn enqueue_msg(&mut self, id: grapple_lasercan::grapple_frc_msgs::MessageId, buf: &[u8]) {
    self.queue.push_back(bxcan::Frame::new_data(ExtendedId::new(id.into()).unwrap(), bxcan::Data::new(buf).unwrap()));
    rtic::pend(Interrupt::USB_HP_CAN_TX);
  }
}

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true)]
mod app {
  use core::{marker::PhantomData, arch::asm};

  use bxcan::Interrupts;
  use grapple_lasercan::LaserCANImpl;
  use shared_bus::new_cortexm;
  use grapple_m24c64::M24C64;
  use lasercan_common::bootutil::feed_watchdog;
  use stm32f1xx_hal::{prelude::*, flash::FlashExt, pac::{TIM1, TIM2, I2C1, Interrupt}, timer::{self, CounterMs, CounterHz}, gpio::{Alternate, OpenDrain}, can::Can, i2c::{Mode, BlockingI2c}};

  use crate::{heap_init, META_VERSION, WatchdogImpl, SysTickImpl, DropToBootloaderImpl, CanBusImpl, InputOutputImpl};

  pub type Impl = LaserCANImpl<crate::WatchdogImpl, crate::SysTickImpl, crate::DropToBootloaderImpl, crate::CanBusImpl, crate::InputOutputImpl>;

  #[shared]
  struct SharedResources {
    lasercan_impl: Impl,
  }
  
  #[local]
  struct LocalResources {
    led_timer: CounterMs<TIM1>,
    status_timer: CounterHz<TIM2>,
  }

  #[init]
  fn init(mut ctx: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {
    let mut gpioa = ctx.device.GPIOA.split();
    let mut gpiob = ctx.device.GPIOB.split();
    let mut gpioc = ctx.device.GPIOC.split();
    let mut afio = ctx.device.AFIO.constrain();

    let pc15_board_rev_detect = gpioc.pc15.into_pull_up_input(&mut gpioc.crh);
    
    let mut flash = ctx.device.FLASH.constrain();

    let clocks = match pc15_board_rev_detect.is_low() {
      false => {
        // Rev 3 or Earlier (w/o Crystal)
        ctx.device.RCC.constrain().cfgr.freeze(&mut flash.acr)
      },
      true => {
        // Rev 4 or Later (w/ Crystal)
        ctx.device.RCC.constrain().cfgr.use_hse(8.MHz()).freeze(&mut flash.acr)
      }
    };

    feed_watchdog(&mut ctx.device.IWDG);

    let (_pa15, _pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    /* TIMER INIT */
    let mut systick = ctx.core.SYST.counter_us(&clocks);
    systick.start(100.millis()).unwrap();
    systick.listen(timer::SysEvent::Update);

    /* LED INIT */

    let mut status_led = gpioa.pa8.into_push_pull_output(&mut gpioa.crh);
    status_led.set_low();

    let mut led_timer = ctx.device.TIM1.counter_ms(&clocks);
    led_timer.start(50.millis()).unwrap();
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
    let config_marshal = grapple_config::m24c64::M24C64ConfigurationMarshal::new(eeprom, 0, config_delay, PhantomData::<grapple_lasercan::LaserCanConfiguration>);
    
    /* CAN INIT */
    
    let can = Can::new(ctx.device.CAN1, ctx.device.USB);
    let rx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
    let tx = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);
    
    can.assign_pins((tx, rx), &mut afio.mapr);
    
    let mut can_bus = bxcan::Can::builder(can)
      .set_bit_timing(0x0005_0000)
      .leave_disabled();
      
    can_bus.enable_interrupts(Interrupts::TRANSMIT_MAILBOX_EMPTY | Interrupts::FIFO0_MESSAGE_PENDING);
    nb::block!(can_bus.enable_non_blocking()).unwrap();
    
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

    let mut sensor = alloc::boxed::Box::new(crate::VL53Sensor::new(vl53l1x_uld::VL53L1X::new(i2c_bus.acquire_i2c(), vl53l1x_uld::DEFAULT_ADDRESS)));
    if sensor.sensor.get_sensor_id().unwrap() != 0xEACC {
      panic!("Wrong Sensor ID!");
    }

    sensor.init().unwrap();

    let lasercan_impl = LaserCANImpl::new(
      META_VERSION,
      lasercan_common::get_serial_hash(),
      WatchdogImpl { watchdog: ctx.device.IWDG },
      SysTickImpl,
      DropToBootloaderImpl { flash },
      CanBusImpl::new(can_bus),
      sensor,
      config_marshal,
      InputOutputImpl { led: status_led.erase() }
    ).unwrap();

    (
      SharedResources {
        lasercan_impl
      },
      LocalResources {
        led_timer,
        status_timer
      },
      init::Monotonics()
    )
  }

  #[task(binds = SysTick, priority=15, local=[])]
  fn systick_tick(_: systick_tick::Context) {
    crate::TIME_MS.fetch_add(100, core::sync::atomic::Ordering::Relaxed);
  }

  #[task(binds = TIM1_UP, priority = 1, shared = [lasercan_impl], local = [led_timer])]
  fn led_tick(mut ctx: led_tick::Context) {
    ctx.shared.lasercan_impl.lock(|i| i.on_led_tick());
    ctx.local.led_timer.clear_interrupt(timer::Event::Update);
  }

  #[task(binds = TIM2, priority = 15, shared = [lasercan_impl], local = [status_timer])]
  fn status_tick(mut ctx: status_tick::Context) {
    ctx.shared.lasercan_impl.lock(|i| i.on_status_tick());
    ctx.local.status_timer.clear_interrupt(timer::Event::Update);
  }

  #[task(binds = USB_HP_CAN_TX, priority = 13, shared = [lasercan_impl], local = [])]
  fn can_tx(mut ctx: can_tx::Context) {
    ctx.shared.lasercan_impl.lock(|i| {
      i.can.can.clear_tx_interrupt();

      while let Some(frame) = i.can.queue.front() {
        match i.can.can.transmit(&frame) {
          Ok(status) => match status.dequeued_frame() {
            Some(pending) => {
              i.can.queue.pop_front();
              i.can.queue.push_back(pending.clone());
              rtic::pend(Interrupt::USB_HP_CAN_TX);
            },
            None => {
              i.can.queue.pop_front();
            },
          },
          Err(nb::Error::WouldBlock) => break,
          Err(_) => unreachable!()
        }
      }

      // Don't let the queue grow too much, otherwise we enter a crash loop
      while i.can.queue.len() > 40 {
        i.can.queue.pop_front();
      }
    })
  }

  #[task(binds = USB_LP_CAN_RX0, shared = [lasercan_impl], local = [])]
  fn can_rx(mut ctx: can_rx::Context) {
    unsafe { asm!("nop") }
    ctx.shared.lasercan_impl.lock(|i| {
      loop {
        match i.can.can.receive() {
          Ok(frame) => match (frame.id(), frame.data()) {
            (bxcan::Id::Extended(ext), data) => {
              unsafe { asm!("nop") }
              
              let id = grapple_lasercan::grapple_frc_msgs::MessageId::from(ext.as_raw());
              let buf = data.map(|x| x.as_ref()).unwrap_or(&[]);

              i.on_can_message(id, buf);
            },
            _ => ()
          },
          Err(nb::Error::WouldBlock) => break,
          Err(nb::Error::Other(_)) => {}    // Ignore overrun errors
        }
      }
    });
    unsafe { asm!("nop") }
  }
}