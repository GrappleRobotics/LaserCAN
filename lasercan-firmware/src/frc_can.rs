use stm32f0xx_hal::can::bxcan::ExtendedId;

pub const DEVICE_BROADCAST: u8 = 0;
pub const DEVICE_ROBOT_CONTROLLER: u8 = 1;
pub const DEVICE_MOTOR_CONTROLLER: u8 = 2;
pub const DEVICE_RELAY_CONTROLLER: u8 = 3;
pub const DEVICE_GYRO_SENSOR: u8 = 4;
pub const DEVICE_ACCELEROMETER: u8 = 5;
pub const DEVICE_ULTRASONIC: u8 = 6;
pub const DEVICE_GEARTOOTH: u8 = 7;
pub const DEVICE_PDP: u8 = 8;
pub const DEVICE_PNEUMATICS: u8 = 9;
pub const DEVICE_MISC: u8 = 10;
pub const DEVICE_IO_BREAKOUT: u8 = 11;
pub const DEVICE_FIRMWARE_UPDATE: u8 = 31;

pub const MANUFACTURER_NI: u8 = 1;
pub const MANUFACTURER_GRAPPLE: u8 = 6;

pub struct FrcCanId {
  pub device_type: u8,
  pub manufacturer: u8,
  pub api: u16,
  pub device_number: u8
}

impl FrcCanId {
  pub fn new(device_type: u8, manufacturer: u8, api: u16, device_number: u8) -> Self {
    Self {
      device_type, manufacturer, api, device_number
    }
  }
}

impl Into<ExtendedId> for FrcCanId {
  fn into(self) -> ExtendedId {
    let id = ((self.device_type as u8 as u32 & 0b11111) << 24) 
      | ((self.manufacturer as u8 as u32 & 0b11111111) << 16)
      | ((self.api as u32 & 0b111111_1111) << 6)
      | (self.device_number as u32 & 0b111111);
    
    ExtendedId::new(id).unwrap()
  }
}

impl From<ExtendedId> for FrcCanId {
  fn from(value: ExtendedId) -> Self {
    let raw = value.as_raw();
    Self {
      device_type: ((raw >> 24) & 0b11111) as u8,
      manufacturer: ((raw >> 16) & 0b11111111) as u8,
      api: ((raw >> 6) & 0b111111_1111) as u16,
      device_number: (raw & 0b111111) as u8,
    }
  }
}