#![no_std]
#![no_main]

use core::{cell::RefCell, panic::PanicInfo};

use cortex_m::interrupt::Mutex;
use cortex_m_semihosting::hprintln;
use led::Leds;
use stm32f0xx_hal::{pwm::{PwmChannels, C1}, pac::{TIM1, FLASH, I2C1}, i2c::I2c, gpio::{gpiob, Alternate, AF1}};
use vl53l1x_uld::{VL53L1X, roi::ROI};

pub mod led;
pub mod frc_can;

// Needs to be here so we can access from panic handler
pub type LedT = Leds<PwmChannels<TIM1, C1>>;
static LEDS: Mutex<RefCell<Option<LedT>>> = Mutex::new(RefCell::new(None));

pub type SensorT = VL53L1X<I2c<I2C1, gpiob::PB6<Alternate<AF1>>, gpiob::PB7<Alternate<AF1>>>>;

#[derive(Debug, Clone)]
pub enum Error {
  FlashUnlockError,
  FlashCorrupted,
  FlashPageError
}

const CONFIGURATION_MAGIC: u16 = 0x1CAB;
const CONFIGURATION_ADDR: u32 = 0x0800_F000;
// const CONFIGURATION_SECTOR: usize = 31;

#[derive(Debug, Clone, PartialEq)]
pub struct Configuration {
  pub magic: u16,
  pub device_id: u16,
  pub ranging_mode: vl53l1x_uld::DistanceMode,
  pub region_of_interest: [u8; 2],
  pub timing_budget: u16
}

impl Default for Configuration {
  fn default() -> Self {
    Self {
      magic: CONFIGURATION_MAGIC,
      device_id: 0x00,
      ranging_mode: vl53l1x_uld::DistanceMode::Short,
      region_of_interest: [16, 16],
      timing_budget: 33
    }
  }
}

impl Configuration {
  pub fn erase(flash: &mut FLASH) -> Result<(), Error> {
    while flash.sr.read().bsy().bit_is_set() {}

    // Unlock Flash
    flash.keyr.write(|w| w.fkeyr().bits(0x45670123));
    flash.keyr.write(|w| w.fkeyr().bits(0xCDEF89AB));

    if !flash.cr.read().lock().is_unlocked() {
      Err(Error::FlashUnlockError)?;
    }

    // Enable Erase Mode
    flash.cr.write(|w| w.lock().unlocked().per().page_erase());
    flash.ar.write(|w| w.far().bits(CONFIGURATION_ADDR));
    flash.cr.write(|w| w.lock().unlocked().per().page_erase().strt().start());

    while flash.sr.read().bsy().bit_is_set() {}

    // Lock the flash
    flash.cr.write(|w| w.lock().locked());

    Ok(())
  }

  pub fn write(&self, flash: &mut FLASH) -> Result<(), Error> {
    Self::erase(flash)?;

    while flash.sr.read().bsy().bit_is_set() {}

    // Unlock Flash
    flash.keyr.write(|w| w.fkeyr().bits(0x45670123));
    flash.keyr.write(|w| w.fkeyr().bits(0xCDEF89AB));

    if !flash.cr.read().lock().is_unlocked() {
      Err(Error::FlashUnlockError)?;
    }

    // Enable Programming Mode
    flash.cr.write(|w| w.lock().unlocked().pg().program());

    // Write the configuration
    unsafe { 
      // *(CONFIGURATION_ADDR as *mut Self) = self.clone
      core::ptr::write_volatile(CONFIGURATION_ADDR as *mut u16, self.magic);
      while flash.sr.read().bsy().bit_is_set() {}
      core::ptr::write_volatile((CONFIGURATION_ADDR + 2) as *mut u16, self.device_id);
      while flash.sr.read().bsy().bit_is_set() {}
      core::ptr::write_volatile((CONFIGURATION_ADDR + 4) as *mut u16, if self.ranging_mode == vl53l1x_uld::DistanceMode::Long { 1 } else { 0 });
      while flash.sr.read().bsy().bit_is_set() {}
      core::ptr::write_volatile((CONFIGURATION_ADDR + 6) as *mut u16, (self.region_of_interest[0] as u16) << 8 | (self.region_of_interest[1] as u16));
      while flash.sr.read().bsy().bit_is_set() {}
      core::ptr::write_volatile((CONFIGURATION_ADDR + 8) as *mut u16, self.timing_budget);
      while flash.sr.read().bsy().bit_is_set() {}
    }

    // Read it back
    if self != &Self::read() {
      // Lock and return error
      flash.cr.write(|w| w.lock().locked());
      Err(Error::FlashCorrupted)?;
    }

    flash.cr.write(|w| w.lock().unlocked().pg().clear_bit());

    let sr = flash.sr.read();
    if sr.pgerr().is_error() {
      // Lock and return error
      flash.cr.write(|w| w.lock().locked());
      Err(Error::FlashPageError)?;
    }

    // Lock the flash
    flash.cr.write(|w| w.lock().locked());

    Ok(())
  }

  pub fn read() -> Self {
    // unsafe { &*(CONFIGURATION_ADDR as *const Self) }
    Self {
      magic: unsafe { core::ptr::read_volatile::<u16>(CONFIGURATION_ADDR as *mut u16) },
      device_id: unsafe { core::ptr::read_volatile::<u16>((CONFIGURATION_ADDR + 2) as *mut u16) },
      ranging_mode: if unsafe { core::ptr::read_volatile::<u16>((CONFIGURATION_ADDR + 4) as *mut u16) } == 0 { vl53l1x_uld::DistanceMode::Short } else { vl53l1x_uld::DistanceMode::Long },
      region_of_interest: [
        unsafe { core::ptr::read_volatile::<u8>((CONFIGURATION_ADDR + 6) as *mut u8) },
        unsafe { core::ptr::read_volatile::<u8>((CONFIGURATION_ADDR + 7) as *mut u8) },
      ],
      timing_budget: unsafe { core::ptr::read_volatile::<u16>((CONFIGURATION_ADDR + 8) as *mut u16) }, 
    }
  }

  pub fn is_configured() -> bool {
    return Self::read().magic == CONFIGURATION_MAGIC;
  }

  pub fn apply(&self, sensor: &mut SensorT) {
    sensor.stop_ranging().unwrap();
    sensor.set_distance_mode(self.ranging_mode).unwrap();
    sensor.set_timing_budget_ms(self.timing_budget).unwrap();
    sensor.set_roi(ROI::new(self.region_of_interest[0] as u16, self.region_of_interest[1] as u16)).unwrap();
    sensor.start_ranging().unwrap();
  }
}

#[rtic::app(device = stm32f0xx_hal::pac, peripherals = true)]
mod app {
  use crate::frc_can::{FrcCanId, DEVICE_ULTRASONIC, MANUFACTURER_GRAPPLE};
  use hal::can::CanInstance;
  use hal::pac::FLASH;
  use hal::timers::Event;
  use stm32f0xx_hal as hal;
  use hal::prelude::*;
  use hal::gpio::Alternate;
  use hal::i2c::I2c;
  use hal::can::bxcan::{Can, ExtendedId, Interrupts, Tx, Frame, Rx};
  use stm32f0xx_hal::can::bxcan::filter::Mask32;
  use vl53l1x_uld::{VL53L1X, MeasureResult, RangeStatus};
  use crate::{LEDS, SensorT, Configuration};
  use crate::led::{Leds, LEDMode};

  const DEVICE_TYPE: u8 = DEVICE_ULTRASONIC;
  const DEVICE_MANUFACTURER: u8 = MANUFACTURER_GRAPPLE;
  const API_STATUS: u16 = 0x01;
  const API_SET_ID: u16 = 0x02;
  const API_SET_RANGE: u16 = 0x10;
  const API_SET_ROI: u16 = 0x11;
  const API_SET_TIMING_BUDGET: u16 = 0x12;

  type CanT = CanInstance<hal::gpio::gpioa::PA12<Alternate<hal::gpio::AF4>>, hal::gpio::gpioa::PA11<Alternate<hal::gpio::AF4>>>;

  #[shared]
  struct SharedResources {
    sensor: SensorT,
    result: Option<MeasureResult>,
    can_tx: Tx<CanT>
  }
  
  #[local]
  struct LocalResources {
    can_send_timer: hal::timers::Timer<hal::pac::TIM2>,
    can_rx: Rx<CanT>,
    flash: FLASH
  }

  #[init]
  fn init(mut ctx: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {
    let mut rcc = ctx.device.RCC.configure().sysclk(8.mhz()).pclk(8.mhz()).freeze(&mut ctx.device.FLASH);

    // Setup Configuration
    if !crate::Configuration::is_configured() {
      crate::Configuration::default().write(&mut ctx.device.FLASH).unwrap();
    }

    let gpioa = ctx.device.GPIOA.split(&mut rcc);
    let gpiob = ctx.device.GPIOB.split(&mut rcc);

    let (
      status_led_pwm,
      mut tof_shut,
      _tof_int,
      i2c_scl,
      i2c_sda,
      can_rx,
      can_tx,
    ) = cortex_m::interrupt::free(|cs| {
      (
        gpioa.pa8.into_alternate_af2(cs),
        gpiob.pb4.into_push_pull_output(cs),
        gpiob.pb5.into_pull_up_input(cs),
        gpiob.pb6.into_alternate_af1(cs),
        gpiob.pb7.into_alternate_af1(cs),
        gpioa.pa11.into_alternate_af4(cs),
        gpioa.pa12.into_alternate_af4(cs)
      )
    });

    // Enable the TOF sensor (shutdown is active low)
    tof_shut.set_high().unwrap();

    // PWM setup
    let pwm = hal::pwm::tim1(ctx.device.TIM1, status_led_pwm, &mut rcc, 200u32.hz());

    // Setup LEDs
    let mut leds = Leds::new(pwm);
    leds.set(LEDMode::Off);
    cortex_m::interrupt::free(|cs| {
      LEDS.borrow(cs).replace(Some(leds));
    });

    // Setup Delay
    let mut delay = hal::delay::Delay::new(ctx.core.SYST, &rcc);

    // Setup I2C
    let i2c = I2c::i2c1(ctx.device.I2C1, (i2c_scl, i2c_sda), 400.khz(), &mut rcc);

    // Give the sensor some time to wake up
    delay.delay_ms(20u16);

    // Initialise sensor to defaults
    let mut sensor = VL53L1X::new(i2c, vl53l1x_uld::DEFAULT_ADDRESS);
    if sensor.get_sensor_id().unwrap() != 0xEACC {
      panic!("Wrong Sensor ID!");
    }
    sensor.init(vl53l1x_uld::IOVoltage::Volt2_8).unwrap();
    sensor.start_ranging().unwrap();
    Configuration::read().apply(&mut sensor);

    // Initialise CAN
    let can_instance = CanInstance::new(ctx.device.CAN, can_tx, can_rx, &mut rcc);
    let mut can = Can::builder(can_instance)
      .set_bit_timing(0x00050000)
      .enable();
  
    can.modify_filters()
      .enable_bank(0, Mask32::frames_with_ext_id(
        // Device Type = Misc, Manufacturer = Grapple. 
        // We check device ID as the messages come in, since we may change ID
        // between power on / power off, and this is easier than redoing the filter bank.
        FrcCanId::new(DEVICE_TYPE, DEVICE_MANUFACTURER, 0x00, 0x00).into(),
        FrcCanId::new(0xFF, 0xFF, 0x00, 0x00).into()
      ))
      // RoboRIO Heartbeat
      .enable_bank(1, Mask32::frames_with_ext_id(
        ExtendedId::new(0x01011840).unwrap(),
        ExtendedId::new(0b11111_11111111_111111_1111_111111).unwrap()
      ));

    can.enable_interrupts(
      Interrupts::FIFO0_MESSAGE_PENDING
    );
    
    let (can_tx, can_rx) = can.split();

    // Setup Timers
    let mut can_send_timer = hal::timers::Timer::tim2(ctx.device.TIM2, 50.hz(), &mut rcc);
    can_send_timer.listen(Event::TimeOut);

    (
      SharedResources {
        sensor,
        result: None,
        can_tx
      },
      LocalResources {
        can_send_timer,
        can_rx,
        flash: ctx.device.FLASH
      },
      init::Monotonics()
    )
  }

  #[idle(shared = [sensor, result, can_tx])]
  fn idle(mut ctx: idle::Context) -> ! {
    loop {
      let result = (&mut ctx.shared.sensor, &mut ctx.shared.result).lock(|sensor, result| {
        if sensor.is_data_ready().unwrap() {
          let r = sensor.get_result().unwrap();
          *result = Some(r.clone());
          Some(r)
        } else {
          None
        }
      });

      if let Some(r) = result {
        cortex_m::interrupt::free(|cs| {
          let mut intensity = 65535 - ((r.distance_mm as u32 * 65535) / 4000u32);
          if r.status != RangeStatus::Valid {
            intensity = 0;
          }
          LEDS.borrow(cs).borrow_mut().as_mut().unwrap().set(LEDMode::Intensity(intensity as u16));
        });
      }
    }
  }

  #[task(binds = TIM2, shared = [can_tx, result], local = [can_send_timer])]
  fn tim2_tick(mut ctx: tim2_tick::Context) {
    // Send out a new status frame
    (&mut ctx.shared.result, &mut ctx.shared.can_tx).lock(|result, can_tx| {
      if let Some(result) = result {
        let cfg = Configuration::read();
        let id: ExtendedId = FrcCanId::new(DEVICE_TYPE, DEVICE_MANUFACTURER, API_STATUS, cfg.device_id as u8).into();
        let distance_bytes = result.distance_mm.to_le_bytes();
        let ambient_bytes = result.ambient.to_le_bytes();
        let msg = [ result.status as u8, distance_bytes[0], distance_bytes[1], ambient_bytes[0], ambient_bytes[1] ];

        can_tx.transmit(&Frame::new_data(id, msg)).ok();
      }
    });

    ctx.local.can_send_timer.wait().ok();
  }

  #[task(binds = CEC_CAN, shared = [sensor], local = [can_rx, flash])]
  fn can(mut ctx: can::Context) {
    loop {
      match ctx.local.can_rx.receive() {
        Ok(frame) => {
          match frame.id() {
            hal::can::bxcan::Id::Standard(_) => (),
            hal::can::bxcan::Id::Extended(id) => {
              let raw_id = id.as_raw();

              if raw_id == 0x01011840 {
                // RoboRIO Heartbeat
              } else if let Some(data) = frame.data() {
                // Is it us?
                let id = FrcCanId::from(id);
                if id.device_type == DEVICE_TYPE && id.manufacturer == DEVICE_MANUFACTURER && id.device_number as u16 == Configuration::read().device_id {
                  // Decode API commands
                  let mut cfg = Configuration::read().clone();

                  match id.api {
                    API_SET_ID if data.len() == 1 => {
                      cfg.device_id = data[0] as u16;
                    },
                    API_SET_RANGE if data.len() == 1 => {
                      cfg.ranging_mode = if data[0] == 0 { vl53l1x_uld::DistanceMode::Short } else { vl53l1x_uld::DistanceMode::Long };
                    },
                    API_SET_ROI if data.len() == 2 => {
                      cfg.region_of_interest[0] = data[0];
                      cfg.region_of_interest[1] = data[1];
                    },
                    API_SET_TIMING_BUDGET if data.len() == 1 => {
                      cfg.timing_budget = data[0] as u16;
                    }
                    _ => ()
                  }

                  cfg.write(ctx.local.flash).unwrap();
                  ctx.shared.sensor.lock(|sensor| cfg.apply(sensor));
                }
              }
            },
          }
        },
        Err(_) => break,
      }
    }
  }
}

#[panic_handler]
fn panic(pinfo: &PanicInfo) -> ! {
  cortex_m::interrupt::free(|cs| {
    let mut l = LEDS.borrow(cs).borrow_mut();
    if let Some(leds) = l.as_mut() {
      leds.set(led::LEDMode::HardFault);
    }
  });
  hprintln!("Error: {:?}", pinfo).ok();
  loop { }
}
