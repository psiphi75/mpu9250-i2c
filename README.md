# mpu9250-i2c

MPU9250 driver for embedded devices and Linux written in Rust.

A platform agnostic driver to interface with the MPU9250 over i2c.

This driver was built using [`embedded-hal`] traits.

[`embedded-hal`]: https://docs.rs/embedded-hal/

## Features

- [x] 3-axis gyroscope
- [x] 3-axis compass (magenetometer)
- [x] 3-axis accelerometer
- [x] Temperature device
- [x] AHRS (Attitude and heading reference system) - has Magwick filter example
- [ ] Tests

## Examples

Example code can be found in the /src/bin folder, this samples work
in Linux devices with an i2c bus, like the Raspberry Pi and BeagleBone.
The sample includes:

- calibrate.rs - code to calibrate the MPU9250 device.
- mpu9250.rs - basic example that reads all data from the device.
- ahrs.rs - Fully functional AHRS algorithm, uses the Magwick filter.

## Linux Example

```Rust
extern crate linux_embedded_hal as hal;
extern crate mpu9250;
use hal::{Delay, I2cdev};
use mpu9250::{calibration::Calibration, Mpu9250};

fn main() {

  // Linux device
  let dev = I2cdev::new("/dev/i2c-2").unwrap();

  // Set the calibration to the default setting.  This can
  // be set to a custom value specific for the device.
  let cal = Calibration {
    ..Default::default()
  };

  let mpu9250 = &mut Mpu9250::new(dev, Delay, cal).unwrap();

  // Initialise with default settings
  mpu9250.init().unwrap();

  // Probe the temperature
  let temp = mpu9250.get_temperature_celcius().unwrap();
}
```

## Calibration

The technology used in these devices is very noisy. Each component
requires a different calibration method. Compile the `calibrate.rs`
file which will produce a executable called `calibrate`.
When run, `calibrate` will give you instructions on how to calibrate.
Then after the calibration step for each component the calibration
settings unique for that device are printed to the console. These
values can be used in your code.
The calibration is temperature sensitive. Hence if you want to be
very precise you should repeat the calibration at different temperatures.
At a high-level the calibration does the following:

- Gyroscope: the average value is taken, this is called the bias.
- Accelerometer: the scale of the accelerometer at rest should
  range from -1.0g to 1.0g, where g is the 9.81 m/s. The
  calibration will take this into account. As well as
  the noise on orthogonal axes.
- Magnetometer: this component can be very different for each device.
  This calibration will ensure that it's extremeties are
  discovered. You will need to rotate the device around
  in all directions.

## MPU9250 documentation

https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/

https://www.invensense.com/wp-content/uploads/2015/02/MPU-9250-Datasheet.pdf

https://www.invensense.com/wp-content/uploads/2015/02/MPU-9250-Register-Map.pdf

## License

Copyright 2018 Simon M. Werner

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.  
You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0. Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.
