import json
import os
import shutil
import subprocess
import sys

def run(*cmd, cwd=".", capture = True):
  process = subprocess.Popen(cmd, stdout=subprocess.PIPE if capture else sys.stdout, stderr=sys.stderr, cwd=cwd)
  output, error = process.communicate()

  if process.returncode != 0:
      raise Exception("File handling failed %d %s %s" % (process.returncode, output, error))
  
  return output

def get_cargo_version(project):
  output = run("cargo", "metadata", "--no-deps", "--format-version", "1", cwd = project)
  for item in json.loads(output)["packages"]:
    if item["name"] == project.lower():
      return item["version"]
  return None

def build():
  run("cargo", "build", "--release", cwd = "lasercan-bootloader")
  run("cargo", "build", "--release", cwd = "lasercan-firmware")
  version = get_cargo_version("lasercan-firmware")

  run(
    "grapple-bundle", "build",
    "-b", "lasercan-bootloader/target/thumbv7m-none-eabi/release/lasercan-bootloader",
    "-f", "lasercan-firmware/target/thumbv7m-none-eabi/release/lasercan-firmware",
    "-c", "bundle-config.json",
    "--lasercan-rev1-bootloader-check",
    f"target/LaserCAN-{version}.grplbndl",
    capture = False
  )

def flash():
  build()

  version = get_cargo_version("lasercan-firmware")

  run(
    "grapple-bundle", "flash", "--chip", "STM32F103C8", f"target/LaserCAN-{version}.grplbndl",
    capture = False
  )

if len(sys.argv) == 1 or sys.argv[1] == "build":
  build()
elif sys.argv[1] == "flash":
  flash()