# Wiring

There are three ways to communicate with the Roadrunner sensor:
* I2C or UART connected to any MCU, or
* Serial over USB connected to the host.

Each has a respective firmware variant, and depending on which variant is flashed on the rp2040 board, you'll need to connect the Roadrunner to the printer correctly.

### UART

Any 2 GPIOs can be used for UART communication, which makes it very convenient on some boards. UART communication can be less reliable and requires more overhead, so I2C is preferred if possible. Find the schematics for the mcu your Roadrunner sensor will be connected to, and find any 4-pin (or more) connector which has `5V`, `GND` and two GPIOs available.

### I2C

Native I2C communication requires special purpose pins to be available on a microcontroller. Some boards have a dedicated connector available for I2C, but many do not.

To find out which buses are compiled into your version of klipper, download `klipper.log` and search for `BUS_PINS_i2c`. There will be one line for each MCU with all i2c buses listed by name with the corresponding pins.

Below are some of the common boards you might want to connect this sensor to.

### Serial over USB

Simply connect over USB, you can leave out the bottom wire cover.

#### BTT SB2209/SB2240 (stm32 mcu)

Schematics: [SB2209](https://github.com/bigtreetech/EBB/blob/master/EBB%20SB2240_2209%20CAN/SB2209/Hardware/SB2209.png), [SB2240](https://github.com/bigtreetech/EBB/blob/master/EBB%20SB2240_2209%20CAN/SB2240/Hardware/SB2240.png)

| Klipper bus name | Connector | SCL | SDA | Notes |
|---|---|---|---|---|
| i2c1_PB6_PB7 | Endstop | Stop1 (PB6) | Stop3 (PB7) | Less convenient since each pin is located on a different connector |
| i2c1_PB8_PB9 | BLTouch | PROBE (PB8) | SERVOS (PB9) | |
| i2c1_PA9_PA10 | SPI_OUT | IO (PA9) | NSS (PA10) | |
| i2c2_PB10_PB11 | SPI_OUT | CLK (PB10) | MOSI (PB11) | |

#### BTT SB2209 (rp2040 mcu)

Schematics: [SB2209](https://github.com/bigtreetech/EBB/blob/master/EBB%20SB2209%20CAN%20(RP2040)/Hardware/EBB%20SB2209%20CAN%20V1.0%EF%BC%88RP2040%EF%BC%89-Pin.png)

There are no good options for I2C on this board, the least bad option is to use USB-C for communication with the klipper host, which frees up the CAN-L/CAN-H pins to connect the sensor over I2C. Unless you only need one hotend fan or can relocate one fan to another pin.

| Klipper bus name | Connector | SCL | SDA | Notes |
|---|---|---|---|---|
| i2c0b | CAN port | CAN-L (gpio5) | CAN-H (gpio4) | Only usable when the board is connected via USB-C |
| i2c0d | SB0000 | Parts cooling fan (gpio13) | TACH (gpio12) | Only usable when PWM fan and parts cooling fan are not used on SB0000 |
| i2c1d | SB0000 | PWM (gpio15) | Hotend Fan (gpio14) | Only usable when PWM fan and hotend fans are not used on SB0000 |

#### BTT EBB36/42 V1.0 (stm32 mcu)

Schematics: [EBB36](https://github.com/bigtreetech/EBB/blob/master/EBB%20CAN%20V1.0%20(STM32F072)/EBB36%20CAN%20V1.0/Hardware/EBB36%20CAN%20V1.0-PIN.png), [EBB42](https://github.com/bigtreetech/EBB/blob/master/EBB%20CAN%20V1.0%20(STM32F072)/EBB42%20CAN%20V1.0/Hardware/EBB42%20CAN%20V1.0-PIN.png)

| Klipper bus name | Connector | SCL | SDA | Notes |
|---|---|---|---|---|
| i2c1_PB6_PB7 | I2C | PB6 | PB7 | Best option |
| i2c1_PB8_PB9 | CAN | RX (PB8) | TX (PB9) | Only usable when the board is connected via USB-C |

#### BTT EBB36/42 V1.1 / V1.2 (stm32 mcu)

Schematics: [EBB36](https://github.com/bigtreetech/EBB/blob/master/EBB%20CAN%20V1.1%20(STM32G0B1)/EBB36%20CAN%20V1.1/Hardware/EBB36%20CAN%20V1.1%26V1.2-PIN.png), [EBB42](https://github.com/bigtreetech/EBB/blob/master/EBB%20CAN%20V1.1%20(STM32G0B1)/EBB42%20CAN%20V1.1/Hardware/EBB42%20CAN%20V1.1%26V1.2-PIN.png)

| Klipper bus name | Connector | SCL | SDA | Notes |
|---|---|---|---|---|
| i2c3_PB3_PB4 | I2C | PB3 | PB4 | Best option |
| i2c1_PB6_PB7 | Endstop | PB6 | PB7 | |
| i2c1_PB8_PB9 | Probe | PB8 | PB9 | |

#### Mellow Fly SB2040 V2 (rp2040 mcu)

Schematics: [Fly SB2040 v2](https://www.fabreeko.com/cdn/shop/products/Mellow-Fly-SB2040-V1-Board-For-Voron-2-4-R2-Trident-Stealthburner-CW2-Extruder-Klipper-Hotend.jpg_Q90.jpg__4_550x.webp?v=1675264365)

| Klipper bus name | Connector | SCL | SDA | Notes |
|---|---|---|---|---|
| i2c0d | EXP | FAN0 (gpio13) | RGB (gpio12) | |
| i2c0e | EXP | PWM1 (gpio17) | PWM0 (gpio16) | |
| i2c0h | Endstop | gpio29 | gpio28 | |

#### Fystec SB CAN Toolhead Board (stm32 mcu)

Schematics: [SB CAN TH](https://wiki.fysetc.com/SB%20CAN%20ToolHead/)

| Klipper bus name | Connector | SCL | SDA | Notes |
|---|---|---|---|---|
| i2c1 | IN.0 / IO.2 | IN.0 (PB6) | IO.2 (PB7) | |

#### BTT SKR v1.1 (lpc176x mcu)

Schematics: [SKR v1.1](https://github.com/bigtreetech/BIGTREETECH-SKR-V1.1/blob/master/hardware/SKR%20V1.1%20PINOUT.pdf)

| Klipper bus name | Connector | SCL | SDA | Notes |
|---|---|---|---|---|
| i2c1 | E1 motor | STEP (P0.1) | DIR (P0.0) | |
| i2c1a | Z motor | STEP (P0.20) | EN (P0.19) | |

#### BTT SKR v1.3 (lpc176x mcu)

Schematics: [SKR v1.3](https://github.com/bigtreetech/BIGTREETECH-SKR-V1.3/blob/master/BTT%20SKR%20V1.3/hardware/SKR-V1.3-pinout.jpg)

| Klipper bus name | Connector | SCL | SDA | Notes |
|---|---|---|---|---|
| i2c1 | E1 motor | STEP (P0.1) | DIR (P0.0) | |
| i2c1a | Y motor | DIR (P0.20) | STEP (P0.19) | |

#### BTT SKR v1.4 (lpc176x mcu)

Schematics: [SKR v1.4](https://github.com/bigtreetech/BIGTREETECH-SKR-V1.3/blob/master/BTT%20SKR%20V1.4/Hardware/SKR-V1.4-pinout.jpg)

| Klipper bus name | Connector | SCL | SDA | Notes |
|---|---|---|---|---|
| i2c1 | I2C | SCL (P0.1) | SDA (P0.0) | Best option for this board |
| i2c1a | Y motor | DIR (P0.20) | STEP (P0.19) | |

#### BTT SKR 2 (stm32 mcu)

Schematics: [SKR 2](https://github.com/bigtreetech/SKR-2/blob/master/Hardware/BIGTREETECH%20SKR%202-Pin.pdf)

| Klipper bus name | Connector | SCL | SDA | Notes |
|---|---|---|---|---|
| i2c1 | Fans | FAN1 (PB6) | FAN2 (PB7) | |
| i2c1a | I2C | PB8 | PB9 | Best option for this board |
| i2c2 | Wifi | PB10 | PB11 | |

#### BTT SKR 3 / SKR 3 EZ (stm32 mcu)

Schematics: [SKR 3](https://github.com/bigtreetech/SKR-3/blob/master/Hardware%20(SKR%203)/BIGTREETECH%20SKR%203-PIN.pdf), [SKR 3 EZ](https://github.com/bigtreetech/SKR-3/blob/master/Hardware%20(SKR%203%20EZ)/BIGTREETECH%20SKR%203%20EZ%20V1.0-PIN.pdf)

| Klipper bus name | Connector | SCL | SDA | Notes |
|---|---|---|---|---|
| i2c1 | Fans | FAN1 (PB6) | FAN0 (PB7) | |
| i2c2 | Wifi | PB10 | PB11 | |

#### BTT Octopus v1.0 / Octopus Pro (stm32 mcu)

Schematics: [Octopus v1.0](https://github.com/bigtreetech/BIGTREETECH-OCTOPUS-V1.0/blob/master/Hardware/BIGTREETECH%20Octopus%20-%20PIN.pdf), [Octopus Pro](https://github.com/bigtreetech/BIGTREETECH-OCTOPUS-Pro/blob/master/Hardware/BIGTREETECH%20Octopus%20Pro%20V1.1-Pin.jpg)

| Klipper bus name | Connector | SCL | SDA | Notes |
|---|---|---|---|---|
| i2c1 | Probe | PB6 | PB7 | |
| i2c1a | I2C | PB8 | PB9 | Best option for this board |

#### Others

Finding a suitable connector on any board is relatively easy, you can follow these steps for your specific board:

1. Find the type of processor on your board, for example STM32 for the BTT SB2209, lpc1769 for the BTT SKR 1.4, etc.
2. In the [klipper repository](https://github.com/Klipper3d/klipper/tree/master/src), locate a file named `i2c.c` within the sub-folder for your processor type, for example [`src/lpc176/i2c.c`](https://github.com/Klipper3d/klipper/blob/master/src/lpc176x/i2c.c).
3. The bus name with corresponding pin names are listed in the file, for example the line `DECL_CONSTANT_STR("BUS_PINS_i2c1", "P0.1,P0.0");` means the bus named `i2c1` has its SCL and SDA pins on `P0.1` and `P0.0` respectively.
4. Find schematics for your board which has all the connectors labeled with pins.
5. With the schematics, check which of the available connectors on your board map to an I2C bus.
