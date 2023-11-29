# Roadrunner assembly manual

Gather all parts from the [bill of materials](BOM.md) before getting started.

### 1. Heat-set inserts

Add the heat-set inserts into `body_part_a.stl` and `body_part_b.stl`.

<img src="../images/manual/001_assembled.jpg" height="200">

There is one last insert on the top not pictured below:

<img src="../images/manual/002_assembled.jpg" height="200">

<img src="../images/manual/003_assembled.jpg" height="200">

### 2. Idler assembly

<img src="../images/manual/008_parts.jpg" height="200">

Assemble `idler.stl` together with the idler gear, needle bearings, and the 3x20mm shaft.

<img src="../images/manual/008_assembled.jpg" height="200">

### 3. Main body assembly

Add the 5mm bearings into `body_part_a.stl` and `body_part_b.stl`.

<img src="../images/manual/004_assembled.jpg" height="200">

Assemble the 5mm shaft, 3x2 magnet and magnet holder parts.

<img src="../images/manual/005_assembled.jpg" height="200">

Fit the 5mm shaft into the bearing before adding the AS5600 board.

<img src="../images/manual/006_parts.jpg" height="200">

Tighten the AS5600 board using M3 x 6mm BHCS screws, the 4-pin header should be aligned with the opening on the left. Make sure the 5mm shaft can spin freely once the screws are tight.

<img src="../images/manual/006_assembled.jpg" height="200">

Add the drive gear to the 5mm shaft, take care to tighten the setscrew (with loctite). Add the 3x30mm shaft as well.

<img src="../images/manual/007_assembled.jpg" height="200">

Complete the main body assembly by sliding the idler onto the 3mm shaft, screw the body A and B parts together using two M3 x 16mm SHCS.

<img src="../images/manual/010_assembled.jpg" height="200">

### 4. Electronics

The next step is to solder the electronics as shown here:

<img src="../images/schematics.jpg" height="350">

Prepare the IR sensor by removing the 3-pin 0.1" header from the board.

<img src="../images/manual/011_parts.jpg" height="200">

Add the 2mm self-tapping screw and make sure the IR sensor lines up with the idler arm.

<img src="../images/manual/011_assembled.jpg" height="200">

Prepare the RP2040-Zero and a 4-pin segment of 90-degree 0.1" header. The middle 2 pins should be removed since they are not connected straight between the two boards (we will solder them in reverse later).

<img src="../images/manual/012_parts.jpg" height="200">

Line up the 90-degree header with both boards and solder in place. Pay attention to solder on pins 9 and 12 (it's easy to be off-by-one and solder on the wrong pins).

<img src="../images/manual/012_soldered_2.jpg" height="200">

You should be able to remove both circuit boards together at this point. The middle two pins on the connector are still unsoldered, this is normal.

<img src="../images/manual/012_soldered_3.jpg" height="200">

With the RP2040 set aside, solder some thin wires to the IR sensor board.

<img src="../images/manual/013_soldered_2.jpg" height="200">

Add the RP2040 and AS5600 boards back in place, and cut the IR sensor wires to length.

<img src="../images/manual/014_wires.jpg" height="200">

The red (5V) and black (GND) wires should reach the bottom edge of the PCB, and the brown (middle) wire should reach the GP0 pin. There is a pocket between the two boards for the extra wire length to fit in.

<img src="../images/manual/014_soldered_2.jpg" height="200">

Add two wires between the RP2040 pins GP10/GP11 and the AS5600 pins SDA/SCL. Note that the I2C pin order on the two boards are reversed so the wires should cross each other!

<img src="../images/manual/015_soldered.jpg" height="200">

Solder the 5V and GND pads on the AS5600 board to the 5V and GND pads on the RP2040.

<img src="../images/manual/015_soldered_2.jpg" height="200">

Add the female JST-XH header to `wire_cover.stl`, and solder wires to the connector. The red (5V) and black (GND) wires are soldered to the RP2040 5V and GND pads, the green and blue wires are soldered to the RP2040 GP4/GP5 pads.

<img src="../images/manual/016_soldered_1.jpg" height="200">

Make sure you solder on the correct pads as pictured. The numbering starts at 0 and the silkscreen alignment can be misleading.

<img src="../images/manual/016_soldered_2.jpg" height="200">

Fit the wires into the channel built into the wire cover, make sure the wires aren't getting pinched.

<img src="../images/manual/016_assembled_2.jpg" height="200">

Add the M3 x 6mm BHCS and M3 x 10mm BHCS to the secure wire cover.

<img src="../images/manual/016_assembled.jpg" height="200">

Push the wires neatly snug together, without covering the RP2040 buttons or the LED. 

<img src="../images/manual/017_wires.jpg" height="200">

### Flashing and testing

Now is time to flash the RP2040 and test your work!

1. Head over to the [releases](https://github.com/EiNSTeiN-/roadrunner-filament-sensor/releases) page.
2. Download the firmware for your configuration, for example `roadrunner_v1_uart_rgb.uf2`.
3. Connect RP2040 to your computer with a USB cable.
4. Press the `BOOT` and `RESET` buttons together, release the `RESET` button, and 1 second later release the `BOOT` button.
5. A drive named `RPI-RP2` will appear on your computer
6. Copy the `.uf2` file to the `RPI-RP2` drive, the RP2040 will reboot and the LED will light up.

##### Communication over UART or I2C?

Both firmwares are available to provide options for all MCUs and toolhead boards. There's pros and cons to both, the main ones are listed below:

**UART:**
* Pro: Good option for MCUs with limited number of pins or without native I2C support. I2C SDA/SCL pins are often special-purpose pins whereas any GPIOs can be used for UART TX/RX.
* Con: The protocol is implemented using bitbang, which is all-around less efficient compared to MCUs that have native I2C support.
* Con: It's only possible to read 4 bytes at a time over UART, so multiple reads are necessary for every update from the sensor, leading to increased communication overhead.
* Con: The implementation is more prone to errors during communication with the RP2040, leading to even greater communication overhead due to retries.

**I2C:**
* Pro: Can be daisy-chained to attach multiple sensors using only 2 pins (requires re-compiling the firmware to get unique I2C IDs).
* Pro: Can read all data from sensor in one go, which lowers the overhead between the MCU and the host.
* Pro: During testing, I2C was less prone to communication failures than UART.
* Con: I2C errors are unrecoverable. In case the RP2040 stops responding or there are delays on the i2c bus for any reason, the printer will perform an emergency stop with an `i2c timeout` error.

If you have available pins on your MCU, I2C is recommended. Otherwise use UART.

##### Flashing red LED (wiring issues)

The LED will flash red when there is an error with the AS5600 board:

<img src="../images/manual/018_firmware_error.gif" height="200">

Possible reasons for this state include:
1. The AS5600 is not responding over the I2C bus.
2. The magnet is too weak, too strong or not detected at all.

If you assembled the sensor correctly, the magnet should be in the correct place, so the most likely reason for this error is an issue with the wiring for the AS5600 board.

##### Solid red, green or blue LED

If everything is soldered correctly, moving the idler into and out of the IR sensor should toggle the LED between blue and green.

Troubleshooting:
1. If the LED stays the same color when the IR sensor is obstructed, the IR sensor may be soldered incorrectly.
2. If the LED switches between red and blue instead of green and blue, you may have a RP2040-Zero clone with an GRB neopixel. Simply flash the GRB firmware variant and that should fix the issue.

Once the lever arm can be actioned by hand, test with a piece of filament. Trim the lever arm slightly so inserting the filament will push the arm out of the way of the IR sensor, and removing the filament will obstruct the sensor.

<img src="../images/manual/019_IR_arm.jpg" height="200">

Once everything is done, your sensor should work!

<img src="../images/manual/019_filament_demo.gif" height="200">

### Final touches

Add the cover, printed in your favorite accent color.

<img src="../images/manual/020_assembled_1.jpg" height="200">

Add the PC4-M5 fittings on top and bottom.

<img src="../images/manual/021_assembled.jpg" height="200">

Lastly, add the spring assembly

<img src="../images/manual/022_assembled.jpg" height="200">

Your sensor is completed!
