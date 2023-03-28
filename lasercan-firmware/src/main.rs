#![no_std]
#![no_main]

use core::{cell::RefCell, panic::PanicInfo};

use cortex_m::interrupt::Mutex;
use cortex_m_semihosting::hprintln;
use led::Leds;
use stm32f0xx_hal::{pwm::{PwmChannels, C1}, pac::TIM1};

pub mod led;
pub mod frc_can;

// Needs to be here so we can access from panic handler
type LedT = Leds<PwmChannels<TIM1, C1>>;
static LEDS: Mutex<RefCell<Option<LedT>>> = Mutex::new(RefCell::new(None));

#[rtic::app(device = stm32f0xx_hal::pac, peripherals = true)]
mod app {
  use crate::frc_can::{FrcCanId, DEVICE_ULTRASONIC, MANUFACTURER_GRAPPLE};
  use hal::can::CanInstance;
  use hal::timers::Event;
  use stm32f0xx_hal as hal;
  use hal::{prelude::*, pac::I2C1};
  use hal::gpio::{Alternate, AF1, gpiob};
  use hal::i2c::I2c;
  use hal::can::bxcan::{Can, ExtendedId, Interrupts, Tx, Frame, Rx};
  use stm32f0xx_hal::can::bxcan::filter::Mask32;
  use vl53l1x_uld::{VL53L1X, MeasureResult, RangeStatus};
  use crate::LEDS;
  use crate::led::{Leds, LEDMode};

  const DEVICE_TYPE: u8 = DEVICE_ULTRASONIC;
  const DEVICE_MANUFACTURER: u8 = MANUFACTURER_GRAPPLE;
  const API_STATUS: u16 = 0x01;
  const API_SET_RANGE: u16 = 0x10;
  const API_SET_ROI: u16 = 0x11;
  const API_SET_TIMING_BUDGET: u16 = 0x12;

  type CanT = CanInstance<hal::gpio::gpioa::PA12<Alternate<hal::gpio::AF4>>, hal::gpio::gpioa::PA11<Alternate<hal::gpio::AF4>>>;

  #[shared]
  struct SharedResources {
    sensor: VL53L1X<I2c<I2C1, gpiob::PB6<Alternate<AF1>>, gpiob::PB7<Alternate<AF1>>>>,
    result: Option<MeasureResult>,
    can_tx: Tx<CanT>
  }
  
  #[local]
  struct LocalResources {
    can_send_timer: hal::timers::Timer<hal::pac::TIM2>,
    can_rx: Rx<CanT>
  }

  #[init]
  fn init(mut ctx: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {
    let mut rcc = ctx.device.RCC.configure().sysclk(8.mhz()).pclk(8.mhz()).freeze(&mut ctx.device.FLASH);

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
    sensor.set_distance_mode(vl53l1x_uld::DistanceMode::Short).unwrap();
    sensor.set_timing_budget_ms(33).unwrap();
    sensor.start_ranging().unwrap();

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
        can_rx
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
        let id: ExtendedId = FrcCanId::new(DEVICE_TYPE, DEVICE_MANUFACTURER, API_STATUS, 0x00).into();
        let distance_bytes = result.distance_mm.to_le_bytes();
        let ambient_bytes = result.ambient.to_le_bytes();
        let msg = [ result.status as u8, distance_bytes[0], distance_bytes[1], ambient_bytes[0], ambient_bytes[1] ];

        can_tx.transmit(&Frame::new_data(id, msg)).ok();
      }
    });

    ctx.local.can_send_timer.wait().ok();
  }

  #[task(binds = CEC_CAN, shared = [sensor], local = [can_rx])]
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
                if id.device_type == DEVICE_TYPE && id.manufacturer == DEVICE_MANUFACTURER && id.device_number == 0x00 {
                  // Decode API commands
                  ctx.shared.sensor.lock(|sensor| {
                    match id.api {
                      API_SET_RANGE if data.len() == 1 => {
                        sensor.stop_ranging().unwrap();
                        sensor.set_distance_mode(if data[0] == 0 { vl53l1x_uld::DistanceMode::Short } else { vl53l1x_uld::DistanceMode::Long }).unwrap();
                        sensor.start_ranging().unwrap();
                      },
                      API_SET_ROI if data.len() == 2 => {
                        sensor.stop_ranging().unwrap();
                        sensor.set_roi(vl53l1x_uld::roi::ROI::new(data[0] as u16, data[1] as u16)).unwrap();
                        sensor.start_ranging().unwrap();
                      },
                      API_SET_TIMING_BUDGET if data.len() == 1 => {
                        sensor.stop_ranging().unwrap();
                        sensor.set_timing_budget_ms(data[0] as u16).unwrap();
                        sensor.start_ranging().unwrap();
                      }
                      _ => ()
                    }
                  });
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
