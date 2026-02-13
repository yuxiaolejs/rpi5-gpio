/*
 * Bare-metal PCIe + RP1 GPIO driver for Raspberry Pi 5
 * 
 * Based on Linux kernel drivers:
 *   - drivers/pci/controller/pcie-brcmstb.c (BCM2712 PCIe controller)
 *   - drivers/mfd/rp1.c (RP1 MFD driver)
 *   - drivers/pinctrl/pinctrl-rp1.c (RP1 GPIO/pinctrl)
 *   - include/dt-bindings/mfd/rp1.h (RP1 register map)
 */

 #include <stdint.h>

 /* ========================================================================
  * BCM2712 Always-On GPIO (gio_aon) - Controls onboard Activity LED
  * This is DIRECTLY on BCM2712, NOT behind PCIe/RP1!
  * Use this for debugging before PCIe is up.
  * 
  * Reference: https://forums.raspberrypi.com/viewtopic.php?t=378484
  * Confirmed working register addresses:
  *   GIO_DATA  = 0x107d517c04
  *   GIO_IODIR = 0x107d517c08
  * Activity LED is on GPIO 9, active-low (LOW=on, HIGH=off)
  * ======================================================================== */
 
 // Direct register addresses (confirmed working)
 #define GIO_DATA_ADDR   0x107d517c04ULL
 #define GIO_IODIR_ADDR  0x107d517c08ULL
 
 // Activity LED is GPIO 9
 #define ACT_LED_BIT     (1 << 9)
 
 // Pointers to registers
 static volatile uint32_t *gio_data;
 static volatile uint32_t *gio_iodir;
 
 // Memory barrier to ensure writes complete
 static inline void dmb(void) {
     __asm__ volatile("dmb sy" ::: "memory");
 }
 
 // Initialize the onboard activity LED (call this FIRST, before anything else)
 static void act_led_init(void) {
     uint32_t val;
     
     gio_data = (volatile uint32_t *)GIO_DATA_ADDR;
     gio_iodir = (volatile uint32_t *)GIO_IODIR_ADDR;
     
     // Set GPIO 9 as output (clear bit 9 in IODIR)
     // IODIR: 0 = output, 1 = input
     val = *gio_iodir;
     val &= ~ACT_LED_BIT;
     *gio_iodir = val;
     dmb();
 }
 
 // Turn activity LED on (set LOW because it's active-low)
 static void act_led_on(void) {
     uint32_t val = *gio_data;
     val &= ~ACT_LED_BIT;
     *gio_data = val;
     dmb();
 }
 
 // Turn activity LED off (set HIGH because it's active-low)
 static void act_led_off(void) {
     uint32_t val = *gio_data;
     val |= ACT_LED_BIT;
     *gio_data = val;
     dmb();
 }
 
 // Simple delay using busy loop (calibrated from led_test.c)
 static void early_delay(uint32_t count) {
     for (volatile uint32_t i = 0; i < count; i++) {
         __asm__ volatile("nop");
     }
 }
 
 // Timing constants (calibrated - 1000000 ~= 500ms based on testing)
 #define DELAY_SHORT     200000      // ~100ms for quick blinks
 #define DELAY_MEDIUM    1000000     // ~500ms 
 #define DELAY_LONG      2000000     // ~1s pause between stages
 
 /* ========================================================================
  * Bit-banged UART TX on Activity LED GPIO
  * 
  * Connect your USB-UART RX to the activity LED test point/pin.
  * The LED is active-low, so we invert: UART HIGH = LED off, UART LOW = LED on
  * 
  * Settings: 9600 baud, 8N1
  * At 9600 baud, each bit is ~104us
  * 
  * We need to calibrate the delay. From testing: 1000000 loops ~= 500ms
  * So 1 loop ~= 500ns, and for 104us we need ~208 loops
  * But there's overhead, so start with a bit less and adjust.
  * ======================================================================== */
 
 // Bit time for 9600 baud = 104.167us
 // Calibration: 1000000 loops = 500ms, so 1 loop = 0.5us
 // For 104us, we need 104/0.5 = 208 loops
 // Add some margin for function call overhead
 #define UART_BIT_DELAY  243
 
 // Set UART line high (idle state) - LED OFF
 static inline void uart_high(void) {
     uint32_t val = *gio_data;
     val |= ACT_LED_BIT;
     *gio_data = val;
 }
 
 // Set UART line low (start bit, data 0) - LED ON
 static inline void uart_low(void) {
     uint32_t val = *gio_data;
     val &= ~ACT_LED_BIT;
     *gio_data = val;
 }
 
 // Precise bit delay for UART timing
 static void uart_bit_delay(void) {
     for (volatile uint32_t i = 0; i < UART_BIT_DELAY; i++) {
         __asm__ volatile("nop");
     }
 }
 
 // Send one byte via bit-banged UART (8N1)
 static void uart_putc(char c) {
     // Start bit (LOW)
     uart_low();
     uart_bit_delay();
     
     // 8 data bits, LSB first
     for (int i = 0; i < 8; i++) {
         if (c & (1 << i)) {
             uart_high();
         } else {
             uart_low();
         }
         uart_bit_delay();
     }
     
     // Stop bit (HIGH)
     uart_high();
     uart_bit_delay();
     
     // Extra stop bit time for safety
     uart_bit_delay();
 }
 
 // Send a string
 static void uart_puts(const char *s) {
     while (*s) {
         uart_putc(*s++);
     }
 }
 
 // Send a hex digit
 static void uart_puthex_digit(uint8_t d) {
     if (d < 10) {
         uart_putc('0' + d);
     } else {
         uart_putc('a' + d - 10);
     }
 }
 
 // Send a 32-bit value as hex
 static void uart_puthex(uint32_t val) {
     uart_puts("0x");
     for (int i = 7; i >= 0; i--) {
         uart_puthex_digit((val >> (i * 4)) & 0xF);
     }
 }
 
 // Blink the activity LED N times with specified on/off delays
 static void act_led_blink(int count, uint32_t on_delay, uint32_t off_delay) {
     for (int i = 0; i < count; i++) {
         act_led_on();
         early_delay(on_delay);
         act_led_off();
         early_delay(off_delay);
     }
 }
 
 // Debug blink patterns:
 //   1 blink  = Starting
 //   2 blinks = PCIe init starting
 //   3 blinks = PCIe link up!
 //   4 blinks = RP1 found
 //   5 blinks = RP1 chip_id verified
 //   Continuous fast = SUCCESS, GPIO toggling
 //   Continuous slow = ERROR (stuck)
 #define BLINK_START         1
 #define BLINK_PCIE_INIT     2
 #define BLINK_PCIE_LINK_UP  3
 #define BLINK_RP1_FOUND     4
 #define BLINK_RP1_CHIPID_OK 5
 
 static void debug_blink(int stage) {
     // Pause before blinking
     early_delay(DELAY_LONG);
     // Blink 'stage' times
     act_led_blink(stage, DELAY_SHORT, DELAY_SHORT);
     // Pause after
     early_delay(DELAY_LONG);
 }
 
 // Error: blink slowly forever
 static void error_blink_forever(void) {
     while (1) {
         act_led_on();
         early_delay(DELAY_MEDIUM);
         act_led_off();
         early_delay(DELAY_MEDIUM);
     }
 }
 
 /* ========================================================================
  * BCM2712 PCIe Root Complex Controller
  * From: drivers/pci/controller/pcie-brcmstb.c
  * ======================================================================== */
 
 // BCM2712 PCIe controller base address (from device tree / serial log)
 #define PCIE_BASE               0x1000120000ULL
 
 // Key register offsets for BCM2712 (uses BCM7712 offsets)
 #define PCIE_MISC_MISC_CTRL                 0x4008
 #define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LO    0x400c
 #define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_HI    0x4010
 #define PCIE_MISC_RC_BAR1_CONFIG_LO         0x402c
 #define PCIE_MISC_PCIE_CTRL                 0x4064
 #define PCIE_MISC_PCIE_STATUS               0x4068
 #define PCIE_MISC_REVISION                  0x406c
 #define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT    0x4070
 #define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI       0x4080
 #define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI      0x4084
 #define PCIE_HARD_DEBUG                     0x4304  // BCM7712 offset
 #define PCIE_EXT_CFG_DATA                   0x8000
 #define PCIE_EXT_CFG_INDEX                  0x9000
 #define PCIE_RGR1_SW_INIT_1                 0x9210
 #define PCIE_RC_CFG_PRIV1_ID_VAL3           0x043c
 
 // Status bits
 #define PCIE_STATUS_DL_ACTIVE               (1 << 5)
 #define PCIE_STATUS_PHYLINKUP               (1 << 4)
 
 // MISC_CTRL bits
 #define MISC_CTRL_RCB_64B_MODE              (1 << 7)
 #define MISC_CTRL_RCB_MPS_MODE              (1 << 10)
 #define MISC_CTRL_SCB_ACCESS_EN             (1 << 12)
 #define MISC_CTRL_CFG_READ_UR_MODE          (1 << 13)
 #define MISC_CTRL_MAX_BURST_SIZE_MASK       (0x3 << 20)
 #define MISC_CTRL_MAX_BURST_SIZE_512        (0x2 << 20)
 
 // HARD_DEBUG bits
 #define SERDES_IDDQ                         (1 << 27)
 
 // BCM2712-specific UBUS/AXI registers
 #define PCIE_MISC_UBUS_CTRL                 0x40a4
 #define PCIE_MISC_UBUS_CTRL_REPLY_ERR_DIS   (1 << 13)
 #define PCIE_MISC_UBUS_CTRL_REPLY_DECERR_DIS (1 << 19)
 #define PCIE_MISC_UBUS_TIMEOUT              0x40a8
 #define PCIE_MISC_RC_CONFIG_RETRY_TIMEOUT   0x40ac
 #define PCIE_MISC_AXI_READ_ERROR_DATA       0x4170
 
 // PCIE_CTRL bits (for 7278-style PERST)
 #define PCIE_CTRL_PERSTB_MASK               (1 << 2)
 
 // RGR1_SW_INIT_1 bits
 #define RGR1_SW_INIT_1_INIT_MASK            (1 << 1)
 
 /* ========================================================================
  * RP1 Chip Constants
  * From: include/dt-bindings/mfd/rp1.h, include/linux/rp1_platform.h
  * ======================================================================== */
 
 // RP1 PCI IDs
 #define PCI_VENDOR_ID_RPI                   0x1de4
 #define PCI_DEVICE_ID_RP1_C0                0x0001
 
 // RP1 Chip ID (read from SYSINFO)
 #define RP1_C0_CHIP_ID                      0x20001927
 
 // RP1 peripheral base offsets within BAR1
 #define RP1_SYSINFO_BASE                    0x000000
 #define RP1_IO_BANK0_BASE                   0x0d0000
 #define RP1_IO_BANK1_BASE                   0x0d4000
 #define RP1_IO_BANK2_BASE                   0x0d8000
 #define RP1_SYS_RIO0_BASE                   0x0e0000
 #define RP1_SYS_RIO1_BASE                   0x0e4000
 #define RP1_SYS_RIO2_BASE                   0x0e8000
 #define RP1_PADS_BANK0_BASE                 0x0f0000
 #define RP1_PADS_BANK1_BASE                 0x0f4000
 #define RP1_PADS_BANK2_BASE                 0x0f8000
 
 // SYSINFO register offsets
 #define SYSINFO_CHIP_ID_OFFSET              0x00000000
 #define SYSINFO_PLATFORM_OFFSET             0x00000004
 
 /* ========================================================================
  * RP1 GPIO Constants
  * From: drivers/pinctrl/pinctrl-rp1.c
  * ======================================================================== */
 
 #define RP1_NUM_GPIOS                       54
 
 // Register block offsets (for atomic set/clear operations)
 #define RP1_RW_OFFSET                       0x0000
 #define RP1_XOR_OFFSET                      0x1000
 #define RP1_SET_OFFSET                      0x2000
 #define RP1_CLR_OFFSET                      0x3000
 
 // GPIO register offsets (per-pin, 8 bytes apart)
 #define RP1_GPIO_STATUS                     0x0000
 #define RP1_GPIO_CTRL                       0x0004
 
 // GPIO_CTRL field masks
 #define RP1_GPIO_CTRL_FUNCSEL_MASK          0x0000001f
 #define RP1_GPIO_CTRL_OUTOVER_MASK          0x00003000
 #define RP1_GPIO_CTRL_OUTOVER_LSB           12
 #define RP1_GPIO_CTRL_OEOVER_MASK           0x0000c000
 #define RP1_GPIO_CTRL_OEOVER_LSB            14
 
 // Function select values
 #define RP1_FSEL_GPIO                       0x05
 #define RP1_FSEL_NONE                       0x09
 #define RP1_FSEL_NONE_HW                    0x1f
 
 // Output override values
 #define RP1_OUTOVER_PERI                    0
 #define RP1_OUTOVER_LOW                     2
 #define RP1_OUTOVER_HIGH                    3
 
 // OE override values
 #define RP1_OEOVER_PERI                     0
 #define RP1_OEOVER_DISABLE                  2
 #define RP1_OEOVER_ENABLE                   3
 
 // RIO (Register I/O) offsets
 #define RP1_RIO_OUT                         0x00
 #define RP1_RIO_OE                          0x04
 #define RP1_RIO_IN                          0x08
 
 // PAD control bits
 #define RP1_PAD_SLEWFAST_MASK               0x00000001
 #define RP1_PAD_SCHMITT_MASK                0x00000002
 #define RP1_PAD_PULL_MASK                   0x0000000c
 #define RP1_PAD_PULL_LSB                    2
 #define RP1_PAD_DRIVE_MASK                  0x00000030
 #define RP1_PAD_IN_ENABLE_MASK              0x00000040
 #define RP1_PAD_OUT_DISABLE_MASK            0x00000080
 
 #define RP1_PAD_DRIVE_2MA                   0x00000000
 #define RP1_PAD_DRIVE_4MA                   0x00000010
 #define RP1_PAD_DRIVE_8MA                   0x00000020
 #define RP1_PAD_DRIVE_12MA                  0x00000030
 
 // Pull-up/down values
 #define RP1_PUD_OFF                         0
 #define RP1_PUD_DOWN                        1
 #define RP1_PUD_UP                          2
 
 /* ========================================================================
  * Outbound window configuration
  * This maps CPU addresses to PCIe addresses for accessing RP1 BARs
  * ======================================================================== */
 
 // CPU address where we'll map PCIe space (must match outbound window setup)
 // From serial log: Linux uses 0x1f00000000
 #define PCIE_OUTBOUND_BASE                  0x1f00000000ULL
 #define PCIE_OUTBOUND_SIZE                  0x00800000ULL   // 8MB
 
 /* ========================================================================
  * Helper macros
  * ======================================================================== */
 
 #define readl(addr)         (*(volatile uint32_t *)(addr))
 #define writel(val, addr)   (*(volatile uint32_t *)(addr) = (val))
 #define readw(addr)         (*(volatile uint16_t *)(addr))
 #define writew(val, addr)   (*(volatile uint16_t *)(addr) = (val))
 
 /* ========================================================================
  * Global state
  * ======================================================================== */
 
 static volatile uint8_t *pcie_base;
 static volatile uint8_t *rp1_bar1;      // RP1's BAR1 (peripheral registers)
 static volatile uint8_t *rp1_gpio;      // GPIO bank 0 base
 static volatile uint8_t *rp1_rio;       // RIO bank 0 base  
 static volatile uint8_t *rp1_pads;      // PADS bank 0 base
 
 /* ========================================================================
  * Delay functions
  * ======================================================================== */
 
 static void delay_us(uint32_t us) {
     // Simple busy loop - calibrate for your clock speed
     // At ~1GHz, 100 iterations ~= 1us
     for (volatile uint32_t i = 0; i < us * 10; i++);
 }
 
 static void delay_ms(uint32_t ms) {
     delay_us(ms * 1000);
 }
 
 /* ========================================================================
  * PCIe controller functions (from pcie-brcmstb.c)
  * ======================================================================== */
 
 static int pcie_link_up(void) {
     uint32_t val = readl(pcie_base + PCIE_MISC_PCIE_STATUS);
     uint32_t dla = val & PCIE_STATUS_DL_ACTIVE;
     uint32_t plu = val & PCIE_STATUS_PHYLINKUP;
     return dla && plu;
 }
 
 // BCM2712 uses 7278-style PERST control via PCIE_CTRL register
 static void perst_set(int assert) {
     uint32_t tmp = readl(pcie_base + PCIE_MISC_PCIE_CTRL);
     if (assert)
         tmp &= ~PCIE_CTRL_PERSTB_MASK;  // PERSTB is active-low
     else
         tmp |= PCIE_CTRL_PERSTB_MASK;
     writel(tmp, pcie_base + PCIE_MISC_PCIE_CTRL);
 }
 
 static void bridge_sw_init_set(int assert) {
     uint32_t tmp = readl(pcie_base + PCIE_RGR1_SW_INIT_1);
     if (assert)
         tmp |= RGR1_SW_INIT_1_INIT_MASK;
     else
         tmp &= ~RGR1_SW_INIT_1_INIT_MASK;
     writel(tmp, pcie_base + PCIE_RGR1_SW_INIT_1);
 }
 
 // Configure outbound window to map CPU addresses to PCIe addresses
 static void pcie_set_outbound_win(uint8_t win, uint64_t cpu_addr, 
                                   uint64_t pcie_addr, uint64_t size) {
     uint64_t cpu_addr_mb = cpu_addr / (1024 * 1024);
     uint64_t limit_addr_mb = (cpu_addr + size - 1) / (1024 * 1024);
     uint32_t tmp;
     
     uart_puts("  Outbound win ");
     uart_puthex(win);
     uart_puts(": CPU ");
     uart_puthex((uint32_t)(cpu_addr >> 32));
     uart_puthex((uint32_t)cpu_addr);
     uart_puts(" -> PCIe ");
     uart_puthex((uint32_t)(pcie_addr >> 32));
     uart_puthex((uint32_t)pcie_addr);
     uart_puts(" size ");
     uart_puthex((uint32_t)size);
     uart_puts("\r\n");
     
     // Set the base of the pcie_addr window (where CPU accesses go in PCIe space)
     writel((uint32_t)pcie_addr, pcie_base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LO + (win * 8));
     writel((uint32_t)(pcie_addr >> 32), pcie_base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_HI + (win * 8));
     
     // Write the addr base & limit (in MBs)
     // BASE_LIMIT register format: [31:20]=limit_mb, [15:4]=base_mb
     tmp = ((uint32_t)(limit_addr_mb & 0xFFF) << 20) | ((uint32_t)(cpu_addr_mb & 0xFFF) << 4);
     writel(tmp, pcie_base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT + (win * 4));
     
     uart_puts("  BASE_LIMIT reg: ");
     uart_puthex(tmp);
     uart_puts(" (base_mb=");
     uart_puthex((uint32_t)cpu_addr_mb);
     uart_puts(", limit_mb=");
     uart_puthex((uint32_t)limit_addr_mb);
     uart_puts(")\r\n");
     
     // Write upper bits (bits above the 12 bits in BASE_LIMIT)
     writel((uint32_t)(cpu_addr_mb >> 12), pcie_base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI + (win * 8));
     writel((uint32_t)(limit_addr_mb >> 12), pcie_base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI + (win * 8));
     
     uart_puts("  BASE_HI=");
     uart_puthex((uint32_t)(cpu_addr_mb >> 12));
     uart_puts(", LIMIT_HI=");
     uart_puthex((uint32_t)(limit_addr_mb >> 12));
     uart_puts("\r\n");
 }
 
 // Config space access for downstream devices
 // Uses ECAM-style addressing: (bus << 20) | (devfn << 12) | (reg & 0xfff)
 // where devfn = (dev << 3) | func
 static uint32_t pcie_config_read(uint8_t bus, uint8_t dev, uint8_t func, uint16_t reg) {
     uint32_t idx;
     uint32_t val;
     
     if (bus == 0 && dev == 0 && func == 0) {
         // Root complex access - direct register access
         return readl(pcie_base + (reg & 0xffc));
     }
     
     if (!pcie_link_up())
         return 0xffffffff;
 
     // Program index register with ECAM-style offset
     // Note: we write bus/devfn, NOT the register offset here
     idx = ((uint32_t)bus << 20) | ((uint32_t)dev << 15) | ((uint32_t)func << 12);
     writel(idx, pcie_base + PCIE_EXT_CFG_INDEX);
     dmb();  // Ensure index write completes before data read
 
     // Read from data window at register offset
     val = readl(pcie_base + PCIE_EXT_CFG_DATA + (reg & 0xffc));
     return val;
 }
 
 static void pcie_config_write(uint8_t bus, uint8_t dev, uint8_t func, uint16_t reg, uint32_t val) {
     uint32_t idx;
     
     if (bus == 0 && dev == 0 && func == 0) {
         // Root complex access
         writel(val, pcie_base + (reg & 0xffc));
         dmb();
         return;
     }
     
     if (!pcie_link_up())
         return;
 
     idx = ((uint32_t)bus << 20) | ((uint32_t)dev << 15) | ((uint32_t)func << 12);
     writel(idx, pcie_base + PCIE_EXT_CFG_INDEX);
     dmb();
     writel(val, pcie_base + PCIE_EXT_CFG_DATA + (reg & 0xffc));
     dmb();
 }
 
 int pcie_init(void) {
     uint32_t tmp;
     int i;
 
     pcie_base = (volatile uint8_t *)PCIE_BASE;
 
     // 1. Assert bridge reset
     bridge_sw_init_set(1);
 
     // 2. Assert PERST#
     perst_set(1);
 
     delay_us(200);
 
     // 3. Deassert bridge reset
     bridge_sw_init_set(0);
 
     // 4. Take SerDes out of IDDQ (power-down)
     tmp = readl(pcie_base + PCIE_HARD_DEBUG);
     tmp &= ~SERDES_IDDQ;
     writel(tmp, pcie_base + PCIE_HARD_DEBUG);
 
     delay_us(200);
 
     // 5. Configure MISC_CTRL (matching Linux driver settings)
     tmp = readl(pcie_base + PCIE_MISC_MISC_CTRL);
     tmp |= MISC_CTRL_SCB_ACCESS_EN;
     tmp |= MISC_CTRL_CFG_READ_UR_MODE;
     tmp |= MISC_CTRL_RCB_MPS_MODE;
     tmp |= MISC_CTRL_RCB_64B_MODE;
     tmp &= ~MISC_CTRL_MAX_BURST_SIZE_MASK;
     tmp |= MISC_CTRL_MAX_BURST_SIZE_512;
     writel(tmp, pcie_base + PCIE_MISC_MISC_CTRL);
     uart_puts("  MISC_CTRL: ");
     uart_puthex(tmp);
     uart_puts("\r\n");
 
     // 6. Set class code to PCI-PCI bridge (0x060400)
     tmp = readl(pcie_base + PCIE_RC_CFG_PRIV1_ID_VAL3);
     tmp &= ~0xffffff;
     tmp |= 0x060400;
     writel(tmp, pcie_base + PCIE_RC_CFG_PRIV1_ID_VAL3);
 
     // 7. Configure bridge bus numbers (Type 1 header offset 0x18)
     // Primary bus = 0, Secondary bus = 1, Subordinate bus = 1
     // This tells the bridge that downstream devices appear on bus 1
     // Format: [subordinate:secondary:primary:0] = 0x00010100
     writel(0x00010100, pcie_base + 0x18);
     dmb();
 
     // 8. Configure outbound window - map CPU 0x1f00000000 -> PCIe 0x0
     // Map to PCIe address 0x80000000 (within bridge's memory window 0x80000000-0xbfffffff)
     pcie_set_outbound_win(0, PCIE_OUTBOUND_BASE, 0x80000000ULL, PCIE_OUTBOUND_SIZE);
 
     // 9. BCM2712-specific: Configure UBUS/AXI bridge
     // Suppress AXI error responses and return 0xFFFFFFFF for read failures
     uart_puts("  Configuring UBUS/AXI bridge...\r\n");
     tmp = readl(pcie_base + PCIE_MISC_UBUS_CTRL);
     uart_puts("  UBUS_CTRL before: ");
     uart_puthex(tmp);
     tmp |= PCIE_MISC_UBUS_CTRL_REPLY_ERR_DIS;
     tmp |= PCIE_MISC_UBUS_CTRL_REPLY_DECERR_DIS;
     writel(tmp, pcie_base + PCIE_MISC_UBUS_CTRL);
     uart_puts(" -> after: ");
     uart_puthex(readl(pcie_base + PCIE_MISC_UBUS_CTRL));
     uart_puts("\r\n");
     
     // Set the value returned on read errors (use 0xFFFFFFFF like Linux)
     uart_puts("  AXI_READ_ERROR_DATA before: ");
     uart_puthex(readl(pcie_base + PCIE_MISC_AXI_READ_ERROR_DATA));
     writel(0xFFFFFFFF, pcie_base + PCIE_MISC_AXI_READ_ERROR_DATA);
     uart_puts(" -> after: ");
     uart_puthex(readl(pcie_base + PCIE_MISC_AXI_READ_ERROR_DATA));
     uart_puts("\r\n");
     
     // Set timeouts (from Linux driver)
     writel(0x0B2D0000, pcie_base + PCIE_MISC_UBUS_TIMEOUT);  // 250ms
     writel(0x0ABA0000, pcie_base + PCIE_MISC_RC_CONFIG_RETRY_TIMEOUT);  // 240ms
     
     uart_puts("  HARD_DEBUG: ");
     uart_puthex(readl(pcie_base + PCIE_HARD_DEBUG));
     uart_puts("\r\n");
     
     uart_puts("  PCIE_STATUS: ");
     uart_puthex(readl(pcie_base + PCIE_MISC_PCIE_STATUS));
     uart_puts("\r\n");
 
     // 10. Deassert PERST# to release the endpoint
     perst_set(0);
 
     // 9. Wait 100ms per PCIe spec
     delay_ms(100);
 
     // 10. Poll for link up (up to 100ms more)
     for (i = 0; i < 20; i++) {
         if (pcie_link_up()) {
             break;
         }
         delay_ms(5);
     }
 
     if (!pcie_link_up()) {
         return -1;  // Link failed
     }
     
     // Enable Memory Space on Root Complex
     uint32_t rc_cmd = readl(pcie_base + 0x04);
     uart_puts("  RC Cmd before: ");
     uart_puthex(rc_cmd);
     rc_cmd |= 0x06;  // Memory Space Enable + Bus Master Enable
     writel(rc_cmd, pcie_base + 0x04);
     dmb();
     uart_puts(" -> after: ");
     uart_puthex(readl(pcie_base + 0x04));
     uart_puts("\r\n");
     
     // Debug: Read Root Port PCI config space memory window registers
     uart_puts("  RC Config space:\r\n");
     uart_puts("    Cmd/Status (0x04): ");
     uart_puthex(readl(pcie_base + 0x04));
     uart_puts("\r\n");
     uart_puts("    Primary/Secondary/Subordinate (0x18): ");
     uart_puthex(readl(pcie_base + 0x18));
     uart_puts("\r\n");
     uart_puts("    Memory Base/Limit (0x20): ");
     uart_puthex(readl(pcie_base + 0x20));
     uart_puts("\r\n");
     uart_puts("    Prefetch Base/Limit (0x24): ");
     uart_puthex(readl(pcie_base + 0x24));
     uart_puts("\r\n");
     uart_puts("    Prefetch Base Upper (0x28): ");
     uart_puthex(readl(pcie_base + 0x28));
     uart_puts("\r\n");
     uart_puts("    Prefetch Limit Upper (0x2C): ");
     uart_puthex(readl(pcie_base + 0x2C));
     uart_puts("\r\n");
 
     return 0;
 }
 
 /* ========================================================================
  * RP1 initialization (from rp1.c)
  * ======================================================================== */
 
 int rp1_init(void) {
     uint32_t id;
     uint32_t bar1_addr;
     uint32_t cmd;
     uint32_t chip_id;
     
     // Read Vendor/Device ID at bus 1, dev 0, func 0
     uart_puts("  Config read bus=1 dev=0 func=0 reg=0...\r\n");
     id = pcie_config_read(1, 0, 0, 0x00);
     uart_puts("  Vendor/Device ID: ");
     uart_puthex(id);
     uart_puts("\r\n");
     
     if (id == 0xFFFFFFFF) {
         uart_puts("  ERROR: 0xFFFFFFFF = no device\r\n");
         return -1;
     }
     
     if (id == 0x00000000) {
         uart_puts("  ERROR: 0x00000000 = config broken\r\n");
         return -1;
     }
     
     // Check for RP1: Vendor ID 0x1de4, Device ID 0x0001
     if ((id & 0xffff) != PCI_VENDOR_ID_RPI || 
         ((id >> 16) & 0xffff) != PCI_DEVICE_ID_RP1_C0) {
         uart_puts("  ERROR: wrong vendor/device ID\r\n");
         return -1;
     }
     uart_puts("  RP1 vendor/device ID OK\r\n");
     
     // BAR1 needs to be programmed with an address within the bridge's memory window
     // The bridge forwards addresses 0x80000000-0xbfffffff (from Memory Base/Limit register)
     // Our outbound window maps CPU 0x1f00000000 -> PCIe 0x80000000
     // So we program BAR1 to PCIe address 0x80000000
     
     // First read BAR1 to see its current state
     uint32_t bar1_raw = pcie_config_read(1, 0, 0, 0x14);
     uart_puts("  BAR1 before: ");
     uart_puthex(bar1_raw);
     uart_puts("\r\n");
     
     // Program BAR1 to PCIe address 0x80000000 (within bridge window)
     uart_puts("  Programming BAR1 to 0x80000000...\r\n");
     pcie_config_write(1, 0, 0, 0x14, 0x80000000);
     
     // Read back to verify
     bar1_raw = pcie_config_read(1, 0, 0, 0x14);
     uart_puts("  BAR1 after: ");
     uart_puthex(bar1_raw);
     uart_puts("\r\n");
     
     // BAR1 is now at PCIe address 0
     
     // Enable memory space access and bus mastering
     cmd = pcie_config_read(1, 0, 0, 0x04);
     uart_puts("  CMD before: ");
     uart_puthex(cmd);
     cmd |= 0x06;  // Memory Space Enable + Bus Master Enable
     pcie_config_write(1, 0, 0, 0x04, cmd);
     uart_puts(" -> after: ");
     uart_puthex(cmd);
     uart_puts("\r\n");
     
     // Give some time for config to settle
     dmb();
     delay_us(100);
     
     // CPU address: outbound window maps PCIE_OUTBOUND_BASE (0x1f00000000) -> PCIe 0x80000000
     // BAR1 is now at PCIe address 0x80000000, so CPU address is PCIE_OUTBOUND_BASE
     rp1_bar1 = (volatile uint8_t *)PCIE_OUTBOUND_BASE;
     uart_puts("  rp1_bar1 CPU addr: ");
     uart_puthex((uint32_t)((uint64_t)rp1_bar1 >> 32));
     uart_puthex((uint32_t)(uint64_t)rp1_bar1);
     uart_puts("\r\n");
     
     // Read back outbound window registers to verify
     uart_puts("  Verifying outbound window regs:\r\n");
     uart_puts("    WIN0_LO: ");
     uart_puthex(readl(pcie_base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LO));
     uart_puts("\r\n    WIN0_HI: ");
     uart_puthex(readl(pcie_base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_HI));
     uart_puts("\r\n    BASE_LIMIT: ");
     uart_puthex(readl(pcie_base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT));
     uart_puts("\r\n    BASE_HI: ");
     uart_puthex(readl(pcie_base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI));
     uart_puts("\r\n    LIMIT_HI: ");
     uart_puthex(readl(pcie_base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI));
     uart_puts("\r\n");
     
     // Debug: Try reading first few words from BAR1 region
     uart_puts("  Attempting memory reads:\r\n");
     for (int i = 0; i < 4; i++) {
         uart_puts("    [");
         uart_puthex(i * 4);
         uart_puts("]: ");
         uint32_t val = readl(rp1_bar1 + i * 4);
         uart_puthex(val);
         uart_puts("\r\n");
     }
     
     // Try to read chip_id
     uart_puts("  Reading chip_id from offset ");
     uart_puthex(RP1_SYSINFO_BASE + SYSINFO_CHIP_ID_OFFSET);
     uart_puts("...\r\n");
     
     chip_id = readl(rp1_bar1 + RP1_SYSINFO_BASE + SYSINFO_CHIP_ID_OFFSET);
     uart_puts("  chip_id: ");
     uart_puthex(chip_id);
     uart_puts(" (expected: 0x20001927)\r\n");
     
     // Set up convenience pointers for GPIO bank 0 (GPIO 0-27)
     rp1_gpio = rp1_bar1 + RP1_IO_BANK0_BASE;
     rp1_rio = rp1_bar1 + RP1_SYS_RIO0_BASE;
     rp1_pads = rp1_bar1 + RP1_PADS_BANK0_BASE;
     
     return 0;
 }
 
 /* ========================================================================
  * RP1 GPIO functions (from pinctrl-rp1.c)
  * ======================================================================== */
 
 // Get pointer to GPIO registers for a specific pin
 static volatile uint8_t *gpio_get_pin_reg(int gpio) {
     // Each GPIO has 8 bytes (STATUS + CTRL)
     // Bank 0: GPIO 0-27, Bank 1: GPIO 28-33, Bank 2: GPIO 34-53
     if (gpio < 28) {
         return rp1_bar1 + RP1_IO_BANK0_BASE + (gpio * 8);
     } else if (gpio < 34) {
         return rp1_bar1 + RP1_IO_BANK1_BASE + ((gpio - 28) * 8);
     } else {
         return rp1_bar1 + RP1_IO_BANK2_BASE + ((gpio - 34) * 8);
     }
 }
 
 // Get pointer to RIO registers for a specific pin
 static volatile uint8_t *gpio_get_rio_reg(int gpio) {
     if (gpio < 28) {
         return rp1_bar1 + RP1_SYS_RIO0_BASE;
     } else if (gpio < 34) {
         return rp1_bar1 + RP1_SYS_RIO1_BASE;
     } else {
         return rp1_bar1 + RP1_SYS_RIO2_BASE;
     }
 }
 
 // Get bit offset within the bank
 static int gpio_get_offset(int gpio) {
     if (gpio < 28) {
         return gpio;
     } else if (gpio < 34) {
         return gpio - 28;
     } else {
         return gpio - 34;
     }
 }
 
 // Get pointer to PAD register for a specific pin
 static volatile uint8_t *gpio_get_pad_reg(int gpio) {
     // PAD registers start at offset 0x4 within the bank, 4 bytes per GPIO
     if (gpio < 28) {
         return rp1_bar1 + RP1_PADS_BANK0_BASE + 0x4 + (gpio * 4);
     } else if (gpio < 34) {
         return rp1_bar1 + RP1_PADS_BANK1_BASE + 0x4 + ((gpio - 28) * 4);
     } else {
         return rp1_bar1 + RP1_PADS_BANK2_BASE + 0x4 + ((gpio - 34) * 4);
     }
 }
 
 // Set GPIO function select
 void gpio_set_fsel(int gpio, uint32_t fsel) {
     volatile uint8_t *pin_reg = gpio_get_pin_reg(gpio);
     volatile uint8_t *pad_reg = gpio_get_pad_reg(gpio);
     uint32_t ctrl, pad;
     
     if (gpio >= RP1_NUM_GPIOS) return;
     
     // Enable input and output on pad
     pad = readl(pad_reg);
     pad |= RP1_PAD_IN_ENABLE_MASK;      // Enable input
     pad &= ~RP1_PAD_OUT_DISABLE_MASK;   // Enable output
     writel(pad, pad_reg);
     
     // Set function select
     ctrl = readl(pin_reg + RP1_GPIO_CTRL);
     ctrl &= ~RP1_GPIO_CTRL_FUNCSEL_MASK;
     ctrl |= (fsel & RP1_GPIO_CTRL_FUNCSEL_MASK);
     
     if (fsel == RP1_FSEL_GPIO || fsel < RP1_FSEL_NONE) {
         // Set OEOVER to ENABLE (force output enable on)
         ctrl &= ~RP1_GPIO_CTRL_OEOVER_MASK;
         ctrl |= (RP1_OEOVER_ENABLE << RP1_GPIO_CTRL_OEOVER_LSB);
         // Set OUTOVER to PERI (let RIO control output value)
         ctrl &= ~RP1_GPIO_CTRL_OUTOVER_MASK;
         ctrl |= (RP1_OUTOVER_PERI << RP1_GPIO_CTRL_OUTOVER_LSB);
     }
     
     writel(ctrl, pin_reg + RP1_GPIO_CTRL);
 }
 
 // Set GPIO direction (1 = input, 0 = output)
 void gpio_set_dir(int gpio, int is_input) {
     volatile uint8_t *rio = gpio_get_rio_reg(gpio);
     int offset = gpio_get_offset(gpio);
     uint32_t mask = 1 << offset;
     
     if (is_input) {
         writel(mask, rio + RP1_RIO_OE + RP1_CLR_OFFSET);
     } else {
         writel(mask, rio + RP1_RIO_OE + RP1_SET_OFFSET);
     }
 }
 
 // Set GPIO output value
 void gpio_set_value(int gpio, int value) {
     volatile uint8_t *rio = gpio_get_rio_reg(gpio);
     int offset = gpio_get_offset(gpio);
     uint32_t mask = 1 << offset;
     
     if (value) {
         writel(mask, rio + RP1_RIO_OUT + RP1_SET_OFFSET);
     } else {
         writel(mask, rio + RP1_RIO_OUT + RP1_CLR_OFFSET);
     }
 }
 
 // Get GPIO input value
 int gpio_get_value(int gpio) {
     volatile uint8_t *rio = gpio_get_rio_reg(gpio);
     int offset = gpio_get_offset(gpio);
     uint32_t val = readl(rio + RP1_RIO_IN);
     return (val >> offset) & 1;
 }
 
 // Configure GPIO as output with initial value
 void gpio_set_output(int gpio, int value) {
     gpio_set_value(gpio, value);
     gpio_set_dir(gpio, 0);  // 0 = output
     gpio_set_fsel(gpio, RP1_FSEL_GPIO);
 }
 
// Configure GPIO as input
void gpio_set_input(int gpio) {
    volatile uint8_t *pad_reg = gpio_get_pad_reg(gpio);
    uint32_t pad;
    
    // Enable input buffer on pad
    pad = readl(pad_reg);
    pad |= RP1_PAD_IN_ENABLE_MASK;      // Enable input
    pad |= RP1_PAD_OUT_DISABLE_MASK;    // Disable output driver
    writel(pad, pad_reg);
    
    gpio_set_dir(gpio, 1);  // 1 = input (clear OE)
    gpio_set_fsel(gpio, RP1_FSEL_GPIO);
}
 
 // Set pull-up/pull-down
 void gpio_set_pull(int gpio, int pull) {
     volatile uint8_t *pad_reg = gpio_get_pad_reg(gpio);
     uint32_t pad = readl(pad_reg);
     
     pad &= ~RP1_PAD_PULL_MASK;
     pad |= (pull & 0x3) << RP1_PAD_PULL_LSB;
     
     writel(pad, pad_reg);
 }
 
 /* ========================================================================
  * Main - Toggle a GPIO pin with debug LED feedback
  * 
  * DEBUG BLINK PATTERNS (on the Pi 5's green activity LED):
  *   1 blink  = Starting up
  *   2 blinks = PCIe init starting  
  *   3 blinks = PCIe link is up!
  *   4 blinks = RP1 found on PCIe bus
  *   5 blinks = RP1 chip_id verified, success!
  *   Continuous fast blink = SUCCESS, also toggling external GPIO
  *   Continuous slow blink = ERROR (check which stage you reached)
  * ======================================================================== */
 
 // Define which GPIO to toggle (0-27 for header pins)
 // GPIO 4 = Pin 7 on the 40-pin header (commonly used)
 #define TEST_GPIO   4
 
 void main(void) {
     int ret;
     
     // ===== Initialize activity LED / UART =====
     act_led_init();
     
     // UART idle is HIGH (LED off)
     uart_high();
     early_delay(DELAY_MEDIUM);  // Let line settle
     
     uart_puts("\r\n\r\n=== Pi5 Bare Metal PCIe ===\r\n");
     
     // ===== STAGE 1: PCIe init =====
     uart_puts("PCIe init...\r\n");
     
     ret = pcie_init();
     if (ret < 0) {
         uart_puts("FAIL: PCIe link failed\r\n");
         while (1) {}
     }
     uart_puts("OK: PCIe link up\r\n");
     
     // ===== STAGE 2: RP1 init =====
     uart_puts("RP1 init...\r\n");
     
     ret = rp1_init();
     if (ret == -1) {
         uart_puts("FAIL: RP1 not found\r\n");
         while (1) {}
     }
     uart_puts("OK: RP1 found\r\n");
     
     // ===== STAGE 3: Test GPIO =====
     uart_puts("GPIO test on pin ");
     uart_puthex(TEST_GPIO);
     uart_puts("\r\n");
     
     gpio_set_output(TEST_GPIO, 0);
    // Simple GPIO blink loop
    uart_puts("Blinking GPIO");
    uart_puthex(TEST_GPIO);
    uart_puts("...\r\n");
    
    while (1) {
        gpio_set_value(TEST_GPIO, 1);
        early_delay(DELAY_LONG);
        
        gpio_set_value(TEST_GPIO, 0);
        early_delay(DELAY_LONG);
    }
}
 