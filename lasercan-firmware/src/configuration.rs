use alloc::string::String;
use grapple_frc_msgs::binmarshal::BinMarshal;
use grapple_frc_msgs::binmarshal;
use alloc::borrow::ToOwned;
use grapple_frc_msgs::grapple::lasercan::{LaserCanRoi, LaserCanRoiU4};

#[derive(Clone, BinMarshal)]
#[marshal(magic = b"LCAN//0.1.2")]
pub struct LaserCanConfiguration {
  #[marshal(bits = "7")]
  pub device_id: u8,
  #[marshal(bits = "1")]
  pub long: bool,
  pub roi: LaserCanRoi,
  pub timing_budget: u8,
  pub led_threshold: u16,
  pub name: String
}

impl Default for LaserCanConfiguration {
  fn default() -> Self {
    Self {
      device_id: 0,
      long: false,
      roi: LaserCanRoi { x: LaserCanRoiU4(8), y: LaserCanRoiU4(8), w: LaserCanRoiU4(16), h: LaserCanRoiU4(16) },
      timing_budget: 33,
      led_threshold: 1000,
      name: "LaserCAN".to_owned()
    }
  }
}

impl LaserCanConfiguration {
  pub fn validate(&self) -> bool {
    if self.device_id > 0x3F { return false }
    if self.timing_budget <= 20 { return false }
    if self.roi.w.0 > 16 || self.roi.h.0 > 16 { return false }
    // TODO: Complex ROI calcs

    true
  }
}