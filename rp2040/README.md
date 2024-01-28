### Dependencies

```
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
```

### Configuring

Some neopixels have RGB color ordering while others have GRB. When the filament is inserted the LED should ligth green, if it is RED then you should use the GRB ordering. Uncomment the following line in `neopixel.h` and re-run `make` to build the firmware.

```
#define GRB_LED_ORDER 1
```

### Building

```
git clone https://github.com/EiNSTeiN-/roadrunner-filament-sensor.git
cd roadrunner-filament-sensor
git submodule update --init --recursive
cd rp2040
mkdir build
cd build
cmake ..
make
```

You'll find multiple `.uf2` files in the `roadrunner-filament-sensor/rp2040/build` directory. The name contains `uart`, `i2c` or `usbserial` for the communication mode between mcu and sensor, and `rgb` vs `grb` for the neopixel type.

### Flashing

1. Connect RP2040-Zero to computer via USB
2. Press `BOOT` and `RESET` at the same time
3. Release `RESET` and 1 second later release `BOOT`
4. A drive named `RPI-RP2` will appear
5. Copy newly built `.uf2` file to the drive to flash the new firmware.