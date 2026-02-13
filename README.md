# Bare Metal PCIe Bring-up on Raspberry Pi 5 (BCM2712 + RP1)

This document describes how to bring up PCIe bare metal on the Raspberry Pi 5 to communicate with the RP1 southbridge chip and control its GPIO pins.

**Warning: This doc is AI generated**

## Overview

The Raspberry Pi 5 uses a BCM2712 SoC connected to an RP1 peripheral controller via PCIe 2.0 x4. The RP1 provides GPIO, UART, SPI, I2C, and other peripherals. To access these from bare metal, we need to:

1. Initialize the BCM2712 PCIe Root Complex
2. Configure the PCIe link to the RP1 endpoint
3. Set up memory windows to access RP1's BAR space
4. Initialize RP1 GPIO registers

## Reference Code

The implementation is based on the Linux kernel drivers:
- `drivers/pci/controller/pcie-brcmstb.c` - BCM2712 PCIe controller driver
- `drivers/mfd/rp1.c` - RP1 MFD driver
- `drivers/pinctrl/pinctrl-rp1.c` - RP1 GPIO/pinctrl driver
- `include/dt-bindings/mfd/rp1.h` - RP1 register definitions

## Hardware Details

### BCM2712 PCIe Controller
- Base address: `0x1000120000` (from device tree `pcie@110000` with `ranges`)
- Uses BRCMSTB PCIe controller IP (BCM7712/BCM2712 variant)

### RP1 Chip
- Vendor ID: `0x1de4` (Raspberry Pi)
- Device ID: `0x0001` (RP1 C0)
- Chip ID: `0x20001927` (read from SYSINFO register)
- BAR1 contains all peripheral registers

## Step-by-Step Initialization

### Step 1: PCIe Controller Base Setup

```c
#define PCIE_BASE                           0x1000120000ULL

// Key register offsets (BCM7712 variant)
#define PCIE_MISC_MISC_CTRL                 0x4008
#define PCIE_MISC_PCIE_STATUS               0x4068
#define PCIE_HARD_DEBUG                     0x4304
#define PCIE_EXT_CFG_INDEX                  0x9000
#define PCIE_EXT_CFG_DATA                   0x8000
```

Reference: `pcie-brcmstb.c` lines 75-180 for register definitions.

### Step 2: Assert Bridge Reset

```c
// Assert bridge software init (reset)
tmp = readl(pcie_base + PCIE_RGR1_SW_INIT_1);
tmp |= (1 << 1);  // PCIE_SW_INIT bit
writel(tmp, pcie_base + PCIE_RGR1_SW_INIT_1);
usleep(100);
```

Reference: `brcm_pcie_bridge_sw_init_set_generic()` in `pcie-brcmstb.c` line 950.

### Step 3: Deassert Bridge Reset

```c
tmp = readl(pcie_base + PCIE_RGR1_SW_INIT_1);
tmp &= ~(1 << 1);
writel(tmp, pcie_base + PCIE_RGR1_SW_INIT_1);
```

### Step 4: Deassert SerDes IDDQ (Power Down)

```c
#define SERDES_IDDQ  (1 << 27)

tmp = readl(pcie_base + PCIE_HARD_DEBUG);
tmp &= ~SERDES_IDDQ;
writel(tmp, pcie_base + PCIE_HARD_DEBUG);
usleep(100);  // Wait for SerDes to stabilize
```

Reference: `brcm_pcie_setup()` in `pcie-brcmstb.c` lines 1399-1406.

### Step 5: Configure MISC_CTRL Register

```c
#define MISC_CTRL_SCB_ACCESS_EN        (1 << 12)
#define MISC_CTRL_CFG_READ_UR_MODE     (1 << 13)
#define MISC_CTRL_MAX_BURST_SIZE_512   (2 << 5)
#define MISC_CTRL_RCB_MPS_MODE         (1 << 10)
#define MISC_CTRL_RCB_64B_MODE         (1 << 11)

tmp = readl(pcie_base + PCIE_MISC_MISC_CTRL);
tmp |= MISC_CTRL_SCB_ACCESS_EN;
tmp |= MISC_CTRL_CFG_READ_UR_MODE;
tmp |= MISC_CTRL_MAX_BURST_SIZE_512;
tmp |= MISC_CTRL_RCB_MPS_MODE;
tmp |= MISC_CTRL_RCB_64B_MODE;
writel(tmp, pcie_base + PCIE_MISC_MISC_CTRL);
```

Reference: `brcm_pcie_setup()` in `pcie-brcmstb.c` lines 1426-1432.

### Step 6: Set Root Complex Class Code

```c
// Set class code to PCI bridge
tmp = readl(pcie_base + 0x043c);
tmp = (tmp & 0xff) | (PCI_CLASS_BRIDGE_PCI << 8);
writel(tmp, pcie_base + 0x043c);
```

### Step 7: Configure PCIe Bridge Bus Numbers

**Critical**: The bridge needs to know which bus numbers to forward to downstream devices.

```c
// Offset 0x18: Primary/Secondary/Subordinate bus numbers
// Primary=0, Secondary=1, Subordinate=1
writel(0x00010100, pcie_base + 0x18);
```

Without this, configuration space accesses to bus 1 will not be forwarded.

### Step 8: Configure Outbound Memory Window

Map CPU addresses to PCIe addresses. The bridge only forwards memory accesses within its Memory Base/Limit window.

**Important**: Check the existing Memory Base/Limit register to see what range the bridge accepts:

```c
uint32_t mem_base_limit = readl(pcie_base + 0x20);
// Format: [31:20]=limit, [15:4]=base (in MB, bits 31:20 of address)
// Example: 0xbff08000 means base=0x80000000, limit=0xbfffffff
```

Configure the outbound window to target an address within this range:

```c
#define PCIE_OUTBOUND_BASE  0x1f00000000ULL  // CPU address
#define PCIE_OUTBOUND_SIZE  0x00800000ULL    // 8MB

// Map CPU 0x1f00000000 -> PCIe 0x80000000
pcie_set_outbound_win(0, PCIE_OUTBOUND_BASE, 0x80000000ULL, PCIE_OUTBOUND_SIZE);
```

The outbound window registers:
- `PCIE_MEM_WIN0_LO` (0x400c): PCIe target address low
- `PCIE_MEM_WIN0_HI` (0x4010): PCIe target address high
- `PCIE_MEM_WIN0_BASE_LIMIT` (0x4070): CPU address base/limit in MB
- `PCIE_MEM_WIN0_BASE_HI` (0x4080): CPU address upper bits
- `PCIE_MEM_WIN0_LIMIT_HI` (0x4084): CPU limit upper bits

Reference: `brcm_pcie_set_outbound_win()` in `pcie-brcmstb.c` lines 620-660.

### Step 9: Configure UBUS/AXI Bridge (BCM2712-specific)

```c
#define PCIE_MISC_UBUS_CTRL                 0x40a4
#define PCIE_MISC_UBUS_TIMEOUT              0x40a8
#define PCIE_MISC_RC_CONFIG_RETRY_TIMEOUT   0x40ac
#define PCIE_MISC_AXI_READ_ERROR_DATA       0x4170

// Suppress AXI error responses
tmp = readl(pcie_base + PCIE_MISC_UBUS_CTRL);
tmp |= (1 << 13);  // REPLY_ERR_DIS
tmp |= (1 << 19);  // REPLY_DECERR_DIS
writel(tmp, pcie_base + PCIE_MISC_UBUS_CTRL);

// Set error data return value
writel(0xFFFFFFFF, pcie_base + PCIE_MISC_AXI_READ_ERROR_DATA);

// Set timeouts
writel(0x0B2D0000, pcie_base + PCIE_MISC_UBUS_TIMEOUT);
writel(0x0ABA0000, pcie_base + PCIE_MISC_RC_CONFIG_RETRY_TIMEOUT);
```

Reference: `brcm_pcie_post_setup_bcm2712()` in `pcie-brcmstb.c` lines 1063-1081.

### Step 10: Enable Memory Space on Root Complex

**Critical**: The Root Complex Command register needs Memory Space enabled.

```c
uint32_t rc_cmd = readl(pcie_base + 0x04);
rc_cmd |= 0x06;  // Memory Space Enable + Bus Master Enable
writel(rc_cmd, pcie_base + 0x04);
```

### Step 11: Deassert PERST#

```c
tmp = readl(pcie_base + PCIE_HARD_DEBUG);
tmp &= ~(1 << 3);  // Clear PERST_ASSERT
writel(tmp, pcie_base + PCIE_HARD_DEBUG);
```

### Step 12: Wait for Link Up

```c
#define PCIE_STATUS_DL_ACTIVE  (1 << 5)
#define PCIE_STATUS_PHYLINKUP  (1 << 4)

// Wait up to 100ms for link
for (int i = 0; i < 20; i++) {
    uint32_t status = readl(pcie_base + PCIE_MISC_PCIE_STATUS);
    if ((status & PCIE_STATUS_DL_ACTIVE) && (status & PCIE_STATUS_PHYLINKUP)) {
        // Link is up!
        break;
    }
    delay_ms(5);
}
```

Reference: `brcm_pcie_link_up()` in `pcie-brcmstb.c` lines 899-905.

## RP1 Initialization

### Step 13: Read RP1 Vendor/Device ID

Use ECAM-style config space access:

```c
// Program index register
uint32_t idx = (1 << 20) | (0 << 15) | (0 << 12);  // bus=1, dev=0, func=0
writel(idx, pcie_base + PCIE_EXT_CFG_INDEX);
dmb();

// Read from data window
uint32_t id = readl(pcie_base + PCIE_EXT_CFG_DATA + 0x00);
// Should be 0x00011de4 (Device ID << 16 | Vendor ID)
```

### Step 14: Program RP1 BAR1

BAR1 must be set to an address within the bridge's memory window:

```c
// Write BAR1 address (must be within 0x80000000-0xbfffffff)
pcie_config_write(1, 0, 0, 0x14, 0x80000000);

// Enable memory space and bus mastering on RP1
uint32_t cmd = pcie_config_read(1, 0, 0, 0x04);
cmd |= 0x06;
pcie_config_write(1, 0, 0, 0x04, cmd);
```

### Step 15: Access RP1 Registers

Now RP1's BAR1 is accessible at CPU address `PCIE_OUTBOUND_BASE`:

```c
volatile uint8_t *rp1_bar1 = (volatile uint8_t *)PCIE_OUTBOUND_BASE;

// Verify by reading chip_id at offset 0
uint32_t chip_id = readl(rp1_bar1 + 0x00);
// Should be 0x20001927
```

## RP1 GPIO Control

### Register Layout

From `include/dt-bindings/mfd/rp1.h`:

```c
#define RP1_IO_BANK0_BASE    0x0d0000  // GPIO control registers
#define RP1_SYS_RIO0_BASE    0x0e0000  // Registered I/O
#define RP1_PADS_BANK0_BASE  0x0f0000  // Pad control
```

### GPIO Control Register (per pin)

Each GPIO has 8 bytes: STATUS (offset 0) and CTRL (offset 4).

```c
#define RP1_GPIO_CTRL              0x04
#define RP1_GPIO_CTRL_FUNCSEL_MASK 0x1f
#define RP1_GPIO_CTRL_OEOVER_MASK  0xc000
#define RP1_GPIO_CTRL_OEOVER_LSB   14

#define RP1_FSEL_GPIO              0x05
#define RP1_OEOVER_ENABLE          3
```

### Pad Control Register (per pin)

```c
// PADS_BANK0 + 0x04 + (gpio * 4)
#define RP1_PAD_OUT_DISABLE_MASK   0x80  // Bit 7: 1=disable output
#define RP1_PAD_IN_ENABLE_MASK     0x40  // Bit 6: 1=enable input
```

### RIO (Registered I/O) Registers

```c
#define RP1_RIO_OUT   0x00  // Output value
#define RP1_RIO_OE    0x04  // Output enable
#define RP1_RIO_IN    0x08  // Input value

// Atomic access offsets
#define RP1_SET_OFFSET  0x2000
#define RP1_CLR_OFFSET  0x3000
```

### Configure GPIO as Output

```c
void gpio_set_output(int gpio, int value) {
    volatile uint8_t *gpio_reg = rp1_bar1 + RP1_IO_BANK0_BASE + (gpio * 8);
    volatile uint8_t *pad_reg = rp1_bar1 + RP1_PADS_BANK0_BASE + 4 + (gpio * 4);
    volatile uint8_t *rio = rp1_bar1 + RP1_SYS_RIO0_BASE;
    uint32_t mask = 1 << gpio;
    
    // 1. Enable pad (clear OUT_DISABLE, set IN_ENABLE)
    uint32_t pad = readl(pad_reg);
    pad |= RP1_PAD_IN_ENABLE_MASK;
    pad &= ~RP1_PAD_OUT_DISABLE_MASK;
    writel(pad, pad_reg);
    
    // 2. Set function select to GPIO, OEOVER to ENABLE
    uint32_t ctrl = readl(gpio_reg + RP1_GPIO_CTRL);
    ctrl &= ~RP1_GPIO_CTRL_FUNCSEL_MASK;
    ctrl |= RP1_FSEL_GPIO;
    ctrl &= ~RP1_GPIO_CTRL_OEOVER_MASK;
    ctrl |= (RP1_OEOVER_ENABLE << RP1_GPIO_CTRL_OEOVER_LSB);
    writel(ctrl, gpio_reg + RP1_GPIO_CTRL);
    
    // 3. Set output enable
    writel(mask, rio + RP1_RIO_OE + RP1_SET_OFFSET);
    
    // 4. Set initial value
    if (value)
        writel(mask, rio + RP1_RIO_OUT + RP1_SET_OFFSET);
    else
        writel(mask, rio + RP1_RIO_OUT + RP1_CLR_OFFSET);
}
```

### Toggle GPIO

```c
void gpio_set_value(int gpio, int value) {
    volatile uint8_t *rio = rp1_bar1 + RP1_SYS_RIO0_BASE;
    uint32_t mask = 1 << gpio;
    
    if (value)
        writel(mask, rio + RP1_RIO_OUT + RP1_SET_OFFSET);
    else
        writel(mask, rio + RP1_RIO_OUT + RP1_CLR_OFFSET);
}
```

### Configure GPIO as Input

```c
void gpio_set_input(int gpio) {
    volatile uint8_t *gpio_reg = rp1_bar1 + RP1_IO_BANK0_BASE + (gpio * 8);
    volatile uint8_t *pad_reg = rp1_bar1 + RP1_PADS_BANK0_BASE + 4 + (gpio * 4);
    volatile uint8_t *rio = rp1_bar1 + RP1_SYS_RIO0_BASE;
    uint32_t mask = 1 << gpio;
    
    // 1. Enable input buffer, disable output driver on pad
    uint32_t pad = readl(pad_reg);
    pad |= RP1_PAD_IN_ENABLE_MASK;      // Enable input
    pad |= RP1_PAD_OUT_DISABLE_MASK;    // Disable output
    writel(pad, pad_reg);
    
    // 2. Set function select to GPIO
    uint32_t ctrl = readl(gpio_reg + RP1_GPIO_CTRL);
    ctrl &= ~RP1_GPIO_CTRL_FUNCSEL_MASK;
    ctrl |= RP1_FSEL_GPIO;
    writel(ctrl, gpio_reg + RP1_GPIO_CTRL);
    
    // 3. Clear output enable (set as input)
    writel(mask, rio + RP1_RIO_OE + RP1_CLR_OFFSET);
}
```

### Read GPIO

```c
int gpio_get_value(int gpio) {
    volatile uint8_t *rio = rp1_bar1 + RP1_SYS_RIO0_BASE;
    uint32_t val = readl(rio + RP1_RIO_IN);
    return (val >> gpio) & 1;
}
```

### Set Pull-up/Pull-down

```c
#define RP1_PULL_NONE  0
#define RP1_PULL_DOWN  1
#define RP1_PULL_UP    2

void gpio_set_pull(int gpio, int pull) {
    volatile uint8_t *pad_reg = rp1_bar1 + RP1_PADS_BANK0_BASE + 4 + (gpio * 4);
    uint32_t pad = readl(pad_reg);
    
    pad &= ~RP1_PAD_PULL_MASK;          // Clear pull bits
    pad |= (pull & 0x3) << RP1_PAD_PULL_LSB;
    
    writel(pad, pad_reg);
}
```

## Common Issues and Solutions

### Issue: Config space reads return 0xFFFFFFFF

**Cause**: PCIe link not up, or bus numbers not configured.

**Solution**: 
1. Check `PCIE_MISC_PCIE_STATUS` for link status
2. Ensure bus numbers are set: `writel(0x00010100, pcie_base + 0x18)`

### Issue: Memory reads return 0xFFFFFFFF or error data

**Cause**: Outbound window not configured correctly, or access outside bridge's memory window.

**Solution**:
1. Read the bridge's Memory Base/Limit register (offset 0x20) to see accepted range
2. Program RP1's BAR1 to an address within that range
3. Configure outbound window to map CPU address to that PCIe address
4. Enable Memory Space on Root Complex: `pcie_base[0x04] |= 0x06`

### Issue: GPIO doesn't toggle

**Cause**: GPIO not configured as output, or pad output disabled.

**Solution**:
1. Set FUNCSEL to GPIO (0x05)
2. Set OEOVER to ENABLE (3)
3. Clear PAD_OUT_DISABLE bit
4. Set RIO_OE bit for the GPIO

## GPIO Pin Mapping

GPIO4 = Physical Pin 7 on the 40-pin header.

Ground pins: 6, 9, 14, 20, 25, 30, 34, 39.

## Build and Run

```bash
aarch64-none-elf-gcc -c -mcpu=cortex-a76 -ffreestanding -nostdlib -o main.o pcie.c
aarch64-none-elf-gcc -c -mcpu=cortex-a76 -o start.o start.S
aarch64-none-elf-ld -T linker.ld -o kernel8.elf start.o main.o
aarch64-none-elf-objcopy -O binary kernel8.elf kernel8.img
```

Copy `kernel8.img` to the boot partition of an SD card with the standard Pi 5 firmware files.

## Debugging Without UART

Since UART is on RP1, it's not available until PCIe is working. Options:

1. **Activity LED**: The Pi 5 has a green activity LED controlled directly by BCM2712 at `gio_aon` base `0x107d517c00`. GPIO 9 (active-low) controls the LED.

2. **Bit-banged UART**: Once you have LED control working, you can bit-bang serial output on the LED pin and capture it with a USB-serial adapter.

```c
#define GIO_DATA_ADDR  0x107d517c04ULL
#define GIO_IODIR_ADDR 0x107d517c08ULL

void act_led_init(void) {
    uint32_t dir = readl(GIO_IODIR_ADDR);
    dir &= ~(1 << 9);  // Set GPIO 9 as output
    writel(dir, GIO_IODIR_ADDR);
}

void act_led_on(void) {
    uint32_t data = readl(GIO_DATA_ADDR);
    data &= ~(1 << 9);  // Active low
    writel(data, GIO_DATA_ADDR);
}
```

## License

This documentation is provided for educational purposes. The Linux kernel code referenced is GPL-licensed.
