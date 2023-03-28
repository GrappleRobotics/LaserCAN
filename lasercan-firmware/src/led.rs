use embedded_hal;

pub enum LEDMode {
  Off,
  Intensity(u16),
  On,
  HardFault,
}

pub struct Leds<Status> {
  status_led: Status
}

impl<Status>
  Leds<Status>
where
  Status: embedded_hal::PwmPin<Duty = u16>,
{
  pub fn new(mut status_led: Status) -> Self {
    status_led.enable();
    
    Self {
      status_led
    }
  }

  pub fn set(&mut self, mode: LEDMode) {
    match mode {
      LEDMode::Off => {
        self.status_led.set_duty(self.status_led.get_max_duty());
      },
      LEDMode::On => {
        self.status_led.set_duty(0);
      },
      LEDMode::Intensity(intensity) => {
        self.status_led.set_duty(((intensity as u32) * (self.status_led.get_max_duty() as u32) / 65535) as u16)
      },
      LEDMode::HardFault => {
        self.status_led.set_duty(0);
      },
    }
  }
}