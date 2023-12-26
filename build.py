import json
import os
import shutil
import subprocess
import sys

def run(*cmd):
  return subprocess.check_output(cmd, env=os.environ.copy(), shell=True)

def get_cargo_version(project):
  output = run("cd", project, "&&", "cargo", "metadata", "--no-deps", "--format-version", "1")
  print("META: ", output)
  for item in json.loads(output)["packages"]:
    if item["name"] == project:
      return item["version"]
  return None

def build_bootloader():
  run("cd", "lasercan-bootloader", "&&", "cargo", "build", "--release")
  version = get_cargo_version("lasercan-bootloader")
  shutil.copy("lasercan-bootloader/target/thumbv7m-none-eabi/release/lasercan-bootloader", f"target/lasercan-bootloader-{version}.elf")
  run("arm-none-eabi-objcopy", "-O", "binary", f"target/lasercan-bootloader-{version}.elf", f"target/lasercan-bootloader-{version}.grplbt")

def build_firmware():
  run("cd", "lasercan-firmware", "&&", "cargo", "build", "--release")
  version = get_cargo_version("lasercan-firmware")
  run("arm-none-eabi-objcopy", '--update-section', '.firmware_flag=./firmware_ready_patch.bin', 'lasercan-firmware/target/thumbv7m-none-eabi/release/lasercan-firmware', f"target/lasercan-firmware-update-{version}.elf")
  run("arm-none-eabi-objcopy", "-O", "binary", f"target/lasercan-firmware-update-{version}.elf", f"target/lasercan-firmware-update-{version}.grplfw")

  # Bootloader 0.1.0 had an issue where if byte at offset 0x1400 was 0xFF, the bootloader wouldn't accept any new firmware because of a failure to erase the sector.
  # This has since been fixed, but as a precaution, this will prevent us accidentally generating a firmware version with this property.
  compat = True
  with open(f"./target/lasercan-firmware-update-{version}.grplfw", 'rb') as f:
    data = f.read()
    for i in range(0, len(data), 1024):
      if data[i] == 0xFF:
        print("Bootloader 0.1.0 incompatible file! (index=0x{:x})".format(i), file=sys.stderr)
        compat = False
  
  if not compat:
    os.remove(f"target/lasercan-firmware-update-{version}.elf")
    os.remove(f"target/lasercan-firmware-update-{version}.grplfw")
    exit(1)

def flash_bootloader():
  build_bootloader()
  run("cargo", "flash", "--chip=STM32F103C8", "--elf", "lasercan-bootloader/target/thumbv7m-none-eabi/release/lasercan-bootloader")

def flash_firmware():
  # Can't flash what's in target/ because it contains the firmware flag reset.
  build_firmware()
  run("cargo", "flash", "--chip=STM32F103C8", "--elf", "lasercan-firmware/target/thumbv7m-none-eabi/release/lasercan-firmware")

if len(sys.argv) == 1:
  build_bootloader()
  build_firmware()
elif sys.argv[1] == "bootloader":
  build_bootloader()
elif sys.argv[1] == "firmware":
  build_firmware()
elif sys.argv[1] == "flash":
  if len(sys.argv) == 2:
    flash_bootloader()
    flash_firmware()
  elif sys.argv[2] == "bootloader":
    flash_bootloader()
  elif sys.argv[2] == "firmware":
    flash_firmware()