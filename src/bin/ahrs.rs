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

extern crate linux_embedded_hal as hal;
extern crate madgwick;
extern crate mpu9250_i2c;

use hal::{Delay, I2cdev};
use madgwick::{F32x3, Marg};
use mpu9250_i2c::{calibration::Calibration, vector::Vector, Mpu9250};
use std::thread::sleep;
use std::time::Duration;
use std::time::Instant;

fn to_ms(duration: Duration) -> u64 {
  duration.as_secs() * 1_000 + (duration.subsec_millis() as u64)
}

fn vector_to_f32x3(v: &Vector<f32>) -> F32x3 {
  F32x3 {
    x: v.x,
    y: v.y,
    z: v.z,
  }
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

  // Set up some stuff
  let mut last_read;
  let rate = 10;

  // Set up Magwick
  let mut ahrs = Marg::new(0.3, 0.01);
  let mut i: i32 = 0;

  // Let's boogie!
  loop {
    // Get the accelerometer and gyro data
    let (va, vg) = mpu9250.get_accel_gyro().unwrap();
    last_read = Instant::now();
    let vm = &mpu9250.get_mag().unwrap();

    // Update the ahrs
    {
      let mut ar = vector_to_f32x3(&vg);
      ar *= std::f32::consts::PI / 180.0;
      let q = ahrs.update(vector_to_f32x3(vm), ar, vector_to_f32x3(&va));
      i += 1;
      if i >= 10 {
        i = 0;
        let ang = 2.0 * q.0.acos();
        let sin_angle = (ang / 2.0).sin();
        println!(
          "{:.2} {:.5} {:.5} {:.5}",
          ang * 180.0 / std::f32::consts::PI,
          q.1 / sin_angle,
          q.2 / sin_angle,
          q.3 / sin_angle
        );
      }
    }

    // Wait until the next read - normally around 10 ms refresh rate
    let elapsed = to_ms(last_read.elapsed());
    if elapsed < rate {
      sleep(Duration::from_millis(rate - elapsed));
    }
  }
}
