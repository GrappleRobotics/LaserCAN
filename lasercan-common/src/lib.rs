#![no_std]

pub mod bootutil;

const UID_PTR: *const u8 = 0x1FFF_F7E8 as _;

pub fn get_serial_hash() -> u32 {
  let uid = unsafe { &*UID_PTR.cast::<[u8; 12]>() };

  let mut hash = 0u32;
  for b in uid {
    hash = (hash.wrapping_mul(31)) ^ (*b as u32);
  }
  hash
}
