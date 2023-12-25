# Bootloader 0.1.0 had an issue where if byte at offset % 1024 == 0 was 0xFF, the bootloader wouldn't accept any new firmware because of a failure to erase the sector.
# This has since been fixed, but as a precaution, this will prevent us accidentally generating a firmware version with this property.

import sys
import os

with open("./target/lasercan-firmware-update.grplfw", 'rb') as f:
  data = f.read()
  for i in range(0, len(data), 1024):
    if data[i] == 0xFF:
      print("Bootloader 0.1.0 incompatible file! (index=0x{:x})".format(i), file=sys.stderr)
      os.remove("target/lasercan-firmware-update.elf")
      os.remove("target/lasercan-firmware-update.grplfw")
      exit(1)