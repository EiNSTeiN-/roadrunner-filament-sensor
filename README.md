# Roadrunner: a high resolution filament sensor for 3D printers

![Preview](images/assembly.png)

### Features

* **High resolution** sub-millimeter motion detection
* **Low cost** at approx. ~12$ USD for parts on Aliexpress
* **Small and light** at half the size of some comercial products
* Helps you **determine runout reason** between filament runout, filament jams, or partial under-extrusion
* **Calibrates max flow** with helpful gcode command.

Most 3D printer motion sensors are bulky, slow to trigger, prone to false positives, and have a high detection distance meaning a large amount material is extruded before actually detecting a runout, leading to poor layer adhesion and failed prints after a runout.

The Roadrunner is based on a magnetic rotary encoder which can detect sub-millimeter movement in the filament for accurate extrusion length measurement, in addition to an IR sensor which can instantly detect filament runout. With the combination of both sensors and a dedicated RP2040-Zero board for collecting data, the Roadrunner can detect which type of issue is affecting a print, between simply reaching the end of a spool, the filament no longer moving, or the filament moving but at a lower than expected rate of extrusion.

By leveraging the higher precision of the sensor, it's possible to determine the length, speed and volumetric flow that are expected and compare them to the actual measured values in real time which gives us a reasonable approximation of _how much_ the filament is being under-extruded by. A simple to use `CALIBRATE_MAX_FLOW` command is provided to perform free-air extrusion tests and automatically benchmark the maximum volumetric flow of any temperature/nozzle/material combination.

### Build guide

This is an open-source project, so you can build your own!

See the [bill of material](manual/BOM.md) and the [assembly manual](manual/ASSEMBLY.md).

### Klipper installation

Clone the repository and create a symlink to the sensor code:

```
git clone https://github.com/EiNSTeiN-/roadrunner-filament-sensor.git
ln -s ~/roadrunner-filament-sensor/klippy/extras/high_resolution_filament_sensor.py ~/klipper/klippy/extras/high_resolution_filament_sensor.py
```

Restart klipper for the new sensor type to be recognized:
```
sudo service klipper restart
```

To get automatic updates within moonraker, add the following block at the end of `moonraker.conf`:
```ini
[update_manager roadrunner-filament-sensor]
type: git_repo
path: ~/roadrunner-filament-sensor
origin: https://github.com/EiNSTeiN-/roadrunner-filament-sensor.git
primary_branch: main
managed_services: klipper
```

### Configuration

Add the following configuration to your `printer.cfg`:

```
[high_resolution_filament_sensor roadrunner]
extruder: extruder
#  The extruder to which this sensor is attached
i2c_address: 64
#  The address of the sensor on the I2C bus, the default is 64 (0x40). Only
#  change this if you attach more than one sensor on the same I2C bus.
#i2c_mcu: mcu
#  The mcu on which the sensor is connected.
i2c_bus: i2c1
#  The I2C bus to which the sensor is connected on the mcu.
i2c_timeout: 30000
#  The time after which an I2C timeout error is thrown.
pause_on_runout: False
#  Whether or not to pause the print automatically when a runout
#  condition is triggered.
runout_gcode: RESPOND TYPE=command MSG='Filament runout detected'
#  The gcode to execute when a runout condition is detected.
invert_direction: False
# If the measured distance/speed values are negative, change this to True.
rotation_distance: 23.4
# The length of filament which corresponds to a full 360 degree rotation
# of the magnet over the hall rotary encoder. A good starting point is the
# circumference of the BMG gears in the sensor.
underextrusion_max_rate: 0.5
# What rate of underextrusion can be tolerated, as a value between 0.0 and 1.0.
# The rate is calculated by comparing the extruder position to the measured 
# filament length over a certain period. A rate of 0.0 means the measured 
# filament motion matches the expected extrusion distance over the period. 
# A rate of 1.0 means no filament motion was detected at all when some motion 
# was expected for the extruder.
underextrusion_period: 5
# How long should the underextrusion rate stay above the max rate before
# a runout is triggered.
```

### Usage

##### Calibrate the sensor's rotation distance:

The first thing to do is to calibrate the `rotation_distance` for the sensor itself. This is done
with `CALIBRATE_FILAMENT_SENSOR_ROTATION_DISTANCE`:

```
CALIBRATE_FILAMENT_SENSOR_ROTATION_DISTANCE SENSOR=roadrunner TEMP=210 LENGTH=10 SPEED=120 COUNT=10
```

* `TEMP`: The temperature to heat up the extruder heater at. Default is 200.
* `LENGTH`: The length of filament to extrude. Default is 25.
* `SPEED`: The extrusion speed, in rotation per minute (e.g. 300 means 5mm/s). Default is 100.
* `COUNT`: How many times to perform the test.

The sensor will heat up the extruder heater to the specified temperature, extrude some filament, 
and print some stats. Make sure to specify a speed setting low enough to avoid overrunning your hotend, otherwise the rotation_distance will be wrong!

##### Max flow calibration

Before starting, make sure your configured `filament_diameter` is correctly configured.

You can determine the maximum volumetric flow for an extruder/nozzle/filament combination using the following command:

```
CALIBRATE_MAX_FLOW SENSOR=roadrunner TEMP=210 DURATION=5 START=2 STOP=25 STEP=1
```

* `TEMP`: The temperature to heat up the extruder heater at. Default is 200.
* `DURATION`: How long to extrude for during each test, used to determine the extrusion length (length = speed * duration). Default is 5 seconds.
* `START`: The minimum volumetric flow to test. Default is 5.
* `STOP`: The maximum volumetric flow to test. Default is 25.
* `STEP`: The amount by which to increment the volumetric flow between each test. Default is 1.
* `COUNT`: How many times to perform the test for each volumetric flow value. Default is 3.
* `MIN_EXTRUSION_RATE`: The minimum extrusion rate that will be considered acceptable during the test. Used for recommending a maximum volumetric flow. Default is 99.
