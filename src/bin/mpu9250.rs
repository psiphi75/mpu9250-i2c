/*****************************************************************************
 *                                                                           *
 *  Copyright 2018 Simon M. Werner                                           *
 *                                                                           *
 *  Licensed under the Apache License, Version 2.0 (the "License");          *
 *  you may not use this file except in compliance with the License.         *
 *  You may obtain a copy of the License at                                  *
 *                                                                           *
 *      http://www.apache.org/licenses/LICENSE-2.0                           *
 *                                                                           *
 *  Unless required by applicable law or agreed to in writing, software      *
 *  distributed under the License is distributed on an "AS IS" BASIS,        *
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
 *  See the License for the specific language governing permissions and      *
 *  limitations under the License.                                           *
 *                                                                           *
 *****************************************************************************/

// BeagleBone Demo
//
// Works on the BeagleBone Blue, it should work on the Rasberry Pi too.  As
// well as other Linux devices with an i2c bus.
//

extern crate linux_embedded_hal as hal;

extern crate mpu9250_i2c;
mod print;

use hal::{Delay, I2cdev};
use mpu9250_i2c::{calibration::Calibration, vector::Vector, Mpu9250};
use print::Print;
use std::thread::sleep;
use std::time::Duration;
use std::time::Instant;

pub fn to_ms(duration: Duration) -> u64 {
  duration.as_secs() * 1_000 + (duration.subsec_millis() as u64)
}

fn main() {
  // The device, this is quite possible to change for your device.
  let dev = I2cdev::new("/dev/i2c-2").unwrap();

  // Sample calibration settings.  You will need to derive your own.
  // Each device will have different calibration settings.
  let cal = Calibration {
    mag_offset: Vector {
      x: 9.5,
      y: 20.78125,
      z: -28.04101,
    },
    mag_scale: Vector {
      x: 1.49696,
      y: 1.44312,
      z: 1.56484,
    },

    // Gryoscope
    gyro_bias_offset: Vector {
      x: 0.57159,
      y: -0.5399,
      z: 0.10633,
    },

    // Accelerometer
    accel_offset: Vector {
      x: 0.00913,
      y: 0.02747,
      z: -0.10344,
    },
    accel_scale_lo: Vector {
      x: 0.99700,
      y: 0.97594,
      z: 0.94592,
    },
    accel_scale_hi: Vector {
      x: -1.0045,
      y: -0.9867,
      z: -1.0648,
    },
  };

  // Let's start
  let mut mpu9250 = Mpu9250::new(dev, Delay, cal).unwrap();
  mpu9250.init().unwrap();

  // Print out some useful information
  Print::mpu9250_settings(&mut mpu9250);
  Print::ak8963_settings(&mut mpu9250);

  // Set up some stuff
  let mut last_mag_read = Instant::now();
  let mut last_read;
  let rate = mpu9250.get_accel_gyro_rate_ms();
  let mag_rate = 10; // 10 milliseconds per read
  let va = &mut Vector {
    x: 0.0,
    y: 0.0,
    z: 0.0,
  };
  let vg = &mut Vector {
    x: 0.0,
    y: 0.0,
    z: 0.0,
  };

  // Let's boogie!
  loop {
    // Get the accelerometer and gyro data
    mpu9250.get_accel_gyro(va, vg).unwrap();
    last_read = Instant::now();
    print!(
      "Accel=>  {{ x: {:.5}, y: {:.5}, z: {:.5} }}",
      va.x, va.y, va.z
    );
    print!(
      "\tGyro=> {{ x: {:.5}, y: {:.5}, z: {:.5} }}",
      vg.x, vg.y, vg.z
    );

    // The mag refresh rate is around 10 milliseconds
    let mag_elapsed = to_ms(last_mag_read.elapsed());
    if mag_elapsed >= mag_rate {
      let vm = mpu9250.get_mag().unwrap();
      print!(
        "\tmag=> {{ x: {:.5}, y: {:.5}, z: {:.5} }}, Temp=> {:0.5}",
        vm.x,
        vm.y,
        vm.z,
        mpu9250.get_temperature_celsius().unwrap()
      );
      last_mag_read = Instant::now();
    }
    println!();

    // Wait until the next read - normally around 4 ms refresh rate
    let elapsed = to_ms(last_read.elapsed());
    if elapsed < rate {
      sleep(Duration::from_millis(rate - elapsed));
    }
  }
}
