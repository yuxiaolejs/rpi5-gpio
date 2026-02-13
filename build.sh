#!/bin/bash

# Change this to select which test to build:
#   led_test.c  - simple LED blink test (no PCIe)
#   pcie.c      - full PCIe + RP1 init
SOURCE_FILE="pcie.c"

rm -f kernel8.elf kernel8.img

echo "Building $SOURCE_FILE..."

aarch64-none-elf-gcc \
  -ffreestanding -nostdlib -nostartfiles \
  -mcpu=cortex-a76 \
  -Wl,-Ttext=0x80000 \
  start.S \
  $SOURCE_FILE \
  -o kernel8.elf

aarch64-none-elf-objcopy \
  -O binary kernel8.elf kernel8.img

aarch64-none-elf-objdump -D kernel8.elf

# cp kernel8.img /media/${USER}/bootfs/
# cp config.txt /media/${USER}/bootfs/

# sync

# ls -la /media/${USER}/bootfs/kernel8.img