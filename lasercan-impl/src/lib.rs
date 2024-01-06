#![no_std]

extern crate alloc;

pub use grapple_frc_msgs;

use alloc::{string::String, borrow::{ToOwned, Cow}, boxed::Box};
use grapple_config::{ConfigurationMarshal, ConfigurationProvider, GenericConfigurationProvider};
use grapple_frc_msgs::{grapple::{lasercan::{LaserCanRoi, LaserCanRangingMode, LaserCanMeasurement, LaserCanRoiU4, LaserCanTimingBudget, self}, errors::{GrappleResult, GrappleError}, TaggedGrappleMessage, fragments::{FragmentReassemblerRx, FragmentReassemblerTx, FragmentReassembler}, GrappleDeviceMessage, firmware::GrappleFirmwareMessage, GrappleBroadcastMessage, device_info::{GrappleDeviceInfo, GrappleModelId}, Request, MANUFACTURER_GRAPPLE, DEVICE_TYPE_DISTANCE_SENSOR}, MessageId, ManufacturerMessage, binmarshal::{Marshal, Demarshal, BitView}, DEVICE_ID_BROADCAST, DEVICE_TYPE_FIRMWARE_UPGRADE, DEVICE_TYPE_BROADCAST};
use grapple_frc_msgs::binmarshal;

// CONFIGURATION

#[derive(Debug, Clone, Marshal, Demarshal)]
#[marshal(magic = b"LCAN//0.2.0")]
pub struct LaserCanConfiguration {
  pub device_id: u8,

  #[marshal(align = "1")]
  pub ranging_mode: LaserCanRangingMode,

  #[marshal(align = "1")]
  pub roi: LaserCanRoi,

  #[marshal(align = "1")]
  pub timing_budget: LaserCanTimingBudget,

  #[marshal(align = "1")]
  pub led_threshold: u16,

  #[marshal(align = "1")]
  pub name: String
}

impl Default for LaserCanConfiguration {
  fn default() -> Self {
    Self {
      device_id: 0,
      ranging_mode: LaserCanRangingMode::Short,
      roi: LaserCanRoi { x: LaserCanRoiU4(8), y: LaserCanRoiU4(8), w: LaserCanRoiU4(16), h: LaserCanRoiU4(16) },
      timing_budget: LaserCanTimingBudget::TB33ms,
      led_threshold: 1000,
      name: "LaserCAN".to_owned()
    }
  }
}

impl LaserCanConfiguration {
  pub fn validate(&self) -> bool {
    if self.device_id > 0x3F { return false }
    if self.roi.w.0 > 16 || self.roi.h.0 > 16 { return false }
    true
  }
}

// TRAITS

pub trait Sensor {
  fn data_ready(&mut self) -> GrappleResult<'static, bool>;
  fn get_result(&mut self) -> GrappleResult<'static, LaserCanMeasurement>;
  fn stop_ranging(&mut self) -> GrappleResult<'static, ()>;
  fn start_ranging(&mut self) -> GrappleResult<'static, ()>;
  fn set_ranging_mode(&mut self, mode: LaserCanRangingMode) -> GrappleResult<'static, ()>;
  fn set_roi(&mut self, roi: LaserCanRoi) -> GrappleResult<'static, ()>;
  fn set_timing_budget_ms(&mut self, budget: LaserCanTimingBudget) -> GrappleResult<'static, ()>;
}

pub trait Watchdog {
  fn feed(&mut self);
}

pub trait SysTick {
  fn time(&self) -> u32;
}

pub trait DropToBootloader {
  fn drop_to_bootloader(&mut self);
}

pub trait CanBus {
  const MAX_MSG_SIZE: usize;

  fn subscribe(&mut self, id: MessageId, mask: MessageId);
  fn enqueue_msg(&mut self, id: MessageId, buf: &[u8]);
}

pub trait InputOutput {
  fn set_led(&mut self, value: bool);    // false = high
  fn toggle_led(&mut self);
}

// IMPL

pub const LED_TICK_PERIOD_MS: usize = 50;
pub const LED_TICK_IRQ_PRIORITY: usize = 1;

pub const STATUS_TICK_PERIOD_MS: usize = 20;
pub const STATUS_TICK_IRQ_PRIORITY: usize = 15;

pub struct LaserCANImpl<WDG, SYST, DTB, CAN, IO> {
  pub firmware_version: &'static str,
  pub serial_number: u32,

  pub watchdog: WDG,
  pub systick: SYST,
  pub to_bootloader: DTB,
  pub can: CAN,
  pub sensor: Box<dyn Sensor + Send + Sync>,
  pub config: Box<dyn GenericConfigurationProvider<LaserCanConfiguration> + Send>,
  pub io: IO,

  reassemble: (FragmentReassemblerRx, FragmentReassemblerTx),
  led_counter: u32,
  blink_counter: u32,
  last_result: Option<LaserCanMeasurement>,
}

impl<
  WDG: Watchdog, SYST: SysTick, DTB: DropToBootloader,
  CAN: CanBus, IO: InputOutput
> LaserCANImpl<WDG, SYST, DTB, CAN, IO> {
  pub fn new<MRSHL: ConfigurationMarshal<LaserCanConfiguration> + 'static + Send>(
    firmware_version: &'static str,
    serial_number: u32,
    watchdog: WDG,
    systick: SYST,
    to_bootloader: DTB,
    can: CAN,
    sensor: Box<dyn Sensor + Send + Sync>,
    marshal: MRSHL,
    io: IO
  ) -> GrappleResult<'static, Self> {
    let provider = Box::new(ConfigurationProvider::new(marshal)
      .map_err(|_| GrappleError::Generic(Cow::Borrowed("Configuration Provider Initialisation Failed").into()))?);

    let mut s = Self {
      firmware_version,
      serial_number,

      watchdog, systick, to_bootloader, can,
      sensor, config: provider, io,

      reassemble: FragmentReassembler::new(1000, CAN::MAX_MSG_SIZE).split(),
      led_counter: 0, blink_counter: 40,
      last_result: None
    };

    s.init()?;

    Ok(s)
  }

  fn init(&mut self) -> GrappleResult<'static, ()> {
    // Try to apply the configuration, but get rid of it if the configuration is invalid.
    let cfg = self.config.current().clone();
    match self.apply_configuration(&cfg) {
      Ok(_) => (),
      Err(_) => {
        *self.config.current_mut() = LaserCanConfiguration::default();
        self.config.commit();
        let cfg = self.config.current().clone();
        self.apply_configuration(&cfg).ok();
      },
    };

    // Firmware Update Messages
    self.can.subscribe(
      MessageId { device_type: DEVICE_TYPE_FIRMWARE_UPGRADE, manufacturer: MANUFACTURER_GRAPPLE, api_class: 0x00, api_index: 0x00, device_id: DEVICE_ID_BROADCAST },
      MessageId { device_type: 0xFF, manufacturer: 0xFF, api_class: 0x00, api_index: 0x00, device_id: 0xFF }
    );

    // Broadcast Messages
    self.can.subscribe(
      MessageId { device_type: DEVICE_TYPE_BROADCAST, manufacturer: MANUFACTURER_GRAPPLE, api_class: 0x00, api_index: 0x00, device_id: DEVICE_ID_BROADCAST },
      MessageId { device_type: 0xFF, manufacturer: 0xFF, api_class: 0x00, api_index: 0x00, device_id: 0xFF }
    );

    // Specific to this sensor. Note we don't check for Device ID here, since it may change whilst we're still booted.
    self.can.subscribe(
      MessageId { device_type: DEVICE_TYPE_DISTANCE_SENSOR, manufacturer: MANUFACTURER_GRAPPLE, api_class: 0x00, api_index: 0x00, device_id: 0x00 },
      MessageId { device_type: 0xFF, manufacturer: 0xFF, api_class: 0x00, api_index: 0x00, device_id: 0x00 }
    );

    Ok(())
  }

  // ============================
  //        INTERRUPTS
  // ============================

  pub fn on_led_tick(&mut self) {
    if self.blink_counter > 0 {
      if self.led_counter % 2 == 0 {
        self.io.toggle_led();
      }
      self.blink_counter -= 1;
    } else if let Some(lr) = &self.last_result {
      let led_thresh = self.config.current().led_threshold;
      if lr.status == 0 && lr.distance_mm < led_thresh && led_thresh > 20 {
        let partial = ((lr.distance_mm as u32) * 20) / (led_thresh as u32) + 1;   // Plus one so we don't modulo by zero and crash the MCU :)
        if self.led_counter % partial == 0 {
          self.io.toggle_led();
        }
      } else {
        self.io.set_led(false);
      }
    } else {
      self.io.set_led(false);
    }

    self.led_counter = self.led_counter.wrapping_add(1);
  }

  // TODO: Move this to be interrupt driven, by TOF_INT
  pub fn on_status_tick(&mut self) {
    self.watchdog.feed();

    if Ok(true) == self.sensor.data_ready() {
      if let Ok(r) = self.sensor.get_result() {
        self.enqueue_can_msg(TaggedGrappleMessage::new(
          self.config.current().device_id,
          GrappleDeviceMessage::DistanceSensor(
            lasercan::LaserCanMessage::Measurement(lasercan::LaserCanMeasurement {
              status: r.status,
              distance_mm: r.distance_mm,
              ambient: r.ambient,
              mode: self.config.current().ranging_mode.clone(),
              budget: self.config.current().timing_budget.clone(),
              roi: self.config.current().roi.clone()
            })
          )
        )).ok();

        self.last_result = Some(r); 
      }
    }
  }

  pub fn on_can_message(&mut self, id: MessageId, buf: &[u8]) -> /* was something processed? */ bool {
    if id.device_id != DEVICE_ID_BROADCAST && id.device_id != self.config.current().device_id {
      return false;
    }

    match ManufacturerMessage::read(&mut BitView::new(buf), id.clone()) {
      Ok(ManufacturerMessage::Grapple(grpl_msg)) => {
        let mut storage: smallvec::SmallVec<[u8; 64]> = smallvec::SmallVec::new();
        
        match self.reassemble.0.defragment(self.systick.time() as i64, &id, grpl_msg, &mut storage) {
          Ok(Some(msg)) => match msg {
            GrappleDeviceMessage::Broadcast(bcast) => match bcast {
              GrappleBroadcastMessage::DeviceInfo(di) => match di {
                GrappleDeviceInfo::EnumerateRequest => {
                  let name = self.config.current().name.clone();

                  let new_msg = TaggedGrappleMessage::new(
                    self.config.current().device_id,
                    GrappleDeviceMessage::Broadcast(
                      GrappleBroadcastMessage::DeviceInfo(GrappleDeviceInfo::EnumerateResponse {
                        model_id: GrappleModelId::LaserCan,
                        serial: self.serial_number,
                        is_dfu: false,
                        is_dfu_in_progress: false,
                        version: Cow::Borrowed(self.firmware_version).into(),
                        name: Cow::Borrowed(name.as_str()).into()
                      })
                    )
                  );

                  self.enqueue_can_msg(new_msg).ok();
                },
                GrappleDeviceInfo::Blink { serial } if serial == self.serial_number => {
                  self.blink_counter = 60;
                },
                GrappleDeviceInfo::SetName { serial, name } if serial == self.serial_number => {
                  self.config.current_mut().name = name.into_owned();
                  self.config.commit();
                },
                GrappleDeviceInfo::SetId { serial, new_id } if serial == self.serial_number => {
                  self.config.current_mut().device_id = new_id;
                  self.config.commit();
                },
                GrappleDeviceInfo::CommitConfig { serial } if serial == self.serial_number => {
                  self.config.commit();
                },
                _ => ()
              }
            },
            GrappleDeviceMessage::DistanceSensor(dist) => match dist {
              lasercan::LaserCanMessage::SetRange(Request::Request(range)) => {
                let response = self.try_configuration_change(|cfg| cfg.ranging_mode = range);

                self.enqueue_can_msg(TaggedGrappleMessage::new(
                  self.config.current().device_id,
                  GrappleDeviceMessage::DistanceSensor(lasercan::LaserCanMessage::SetRange(
                    Request::Ack(response)
                  ))
                )).ok();
              },
              lasercan::LaserCanMessage::SetRoi(Request::Request(roi)) => {
                let response = self.try_configuration_change(|cfg| cfg.roi = roi);

                self.enqueue_can_msg(TaggedGrappleMessage::new(
                  self.config.current().device_id,
                  GrappleDeviceMessage::DistanceSensor(lasercan::LaserCanMessage::SetRoi(
                    Request::Ack(response)
                  ))
                )).ok();
              },
              lasercan::LaserCanMessage::SetTimingBudget(Request::Request(budget)) => {
                let response = self.try_configuration_change(|cfg| cfg.timing_budget = budget);

                self.enqueue_can_msg(TaggedGrappleMessage::new(
                  self.config.current().device_id,
                  GrappleDeviceMessage::DistanceSensor(lasercan::LaserCanMessage::SetTimingBudget(
                    Request::Ack(response)
                  ))
                )).ok();
              },
              lasercan::LaserCanMessage::SetLedThreshold(Request::Request(thresh)) => {
                self.config.current_mut().led_threshold = thresh;
                self.config.commit();
                
                self.enqueue_can_msg(TaggedGrappleMessage::new(
                  self.config.current().device_id,
                  GrappleDeviceMessage::DistanceSensor(lasercan::LaserCanMessage::SetLedThreshold(
                    Request::Ack(Ok(()))
                  ))
                )).ok();
              },
              _ => ()
            },
            GrappleDeviceMessage::FirmwareUpdate(GrappleFirmwareMessage::StartFieldUpgrade { serial }) if serial == self.serial_number => {
              self.to_bootloader.drop_to_bootloader();
            },
            _ => ()
          },
          _ => ()
        }
      },
      Err(_) => ( /* Ignore any malformed messages */ ),
    }

    true
  }

  // ============================
  //          INTERNAL
  // ============================

  fn try_configuration_change<F: FnOnce(&mut LaserCanConfiguration)>(&mut self, f: F) -> GrappleResult<'static, ()> {
    let mut candidate = self.config.current().clone();
    f(&mut candidate);

    match self.apply_configuration(&candidate) {
      Ok(()) => {
        *self.config.current_mut() = candidate;
        self.config.commit();
        Ok(())
      },
      Err(e) => {
        let cfg = self.config.current().clone();
        self.apply_configuration(&cfg).ok();
        Err(e)
      }
    }
  }

  fn apply_configuration(&mut self, cfg: &LaserCanConfiguration) -> GrappleResult<'static, ()> {
    if !cfg.validate() {
      Err(GrappleError::FailedAssertion(Cow::Borrowed("Invalid Configuration!").into()))?;
    }

    self.sensor.stop_ranging()?;
    self.sensor.set_ranging_mode(cfg.ranging_mode.clone())?;
    self.sensor.set_roi(cfg.roi.clone())?;
    self.sensor.set_timing_budget_ms(cfg.timing_budget.clone())?;
    self.sensor.start_ranging()?;
    Ok(())
  }

  fn enqueue_can_msg(&mut self, msg: TaggedGrappleMessage) -> GrappleResult<'static, ()> {
    self.reassemble.1.maybe_fragment(msg.device_id, msg.msg, &mut |id, buf| {
      self.can.enqueue_msg(id, buf)
    }).map_err(|_| GrappleError::Generic(Cow::Borrowed("Failed to encode message").into()))?;
    Ok(())
  }
}
