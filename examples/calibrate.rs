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
extern crate mpu9250_i2c;

use hal::{Delay, I2cdev};
use mpu9250_i2c::{calibration::Calibration, vector::Vector, Mpu9250};
use std::f32;
use std::io;
use std::io::prelude::*;
use std::thread::sleep;
use std::time::Duration;

const NUM_GYRO_READS: i32 = 5000;
const NUM_ACCEL_READS: i32 = 2000;
const NUM_MAG_READS: i32 = 2000;

fn main() {
  let dev = I2cdev::new("/dev/i2c-2").unwrap();
  let cal = Calibration {
    ..Default::default()
  };
  let mpu9250 = &mut Mpu9250::new(dev, Delay, cal).unwrap();
  mpu9250.init().unwrap();

  calibrate_gyro(mpu9250);
  calibrate_accel(mpu9250);
  calibrate_mag(mpu9250);
}

fn wait() {
  let num_secs = 12;
  for i in 1..num_secs {
    print!("\rStarting in {} seconds  ", num_secs - i);
    io::stdout().flush().expect("Could not flush stdout");

    sleep(Duration::from_secs(1));
  }
  println!("");
}

//
//
//   GYROSCOPE
//
//
//   Calibrate the gyro.  The device needs to remain still during calibration.  The calibration will
//   be applied to the gyro.  This is only simple calibration for Gyro bias for when the Gyro is still.
//   More sophisticated calibration tools can be applied.
//
//   NOTE: The Gyro must not be moved during this process.
//
//

fn calibrate_gyro(mpu9250: &mut Mpu9250<I2cdev, Delay>) {
  println!("--- GYRO CALIBRATION ---");
  println!("Keep the MPU very still.  Calculating gyroscope bias");
  mpu9250.wait(100);

  let mut vg_sum = Vector {
    x: 0.0,
    y: 0.0,
    z: 0.0,
  };

  println!("   x         y         z");
  for _ in 0..NUM_GYRO_READS {
    let vg = mpu9250.get_gyro().unwrap();

    vg_sum.x += vg.x;
    vg_sum.y += vg.y;
    vg_sum.z += vg.z;

    mpu9250.wait(4);
    print!("\r{:0.5}  {:0.5}  {:0.5}   ", vg.x, vg.y, vg.z);
  }

  vg_sum.x /= -(NUM_GYRO_READS as f32);
  vg_sum.y /= -(NUM_GYRO_READS as f32);
  vg_sum.z /= -(NUM_GYRO_READS as f32);

  println!("\t\t\t\t\t\t\t\t\n");
  println!(
    "gyro_bias_offset: Vector {{ x: {:0.5},  y: {:0.5},  z: {:0.5} }},",
    vg_sum.x, vg_sum.y, vg_sum.z
  );
}

//
//   ACCELEROMETER
//
//
//   Calibrate the Accelerometer.  This device will need to be rotated with the X, Y and Z axes up and down.  The axis
//   you point up/down will be calibrated against gravity (so you must have it vertical).  You may want to hold it against
//   a wall or similar.  While the one axis is being calibrated against gravity, the other two axes will be perpendicular
//   to gravity, so will read near zero, this value will be used as the offset.
//
//   The scaling is simple linear scaling, based on the common formular for a line, y = m * x + c, where y is our scaled
//   and offset result, while x is the raw value.  This formular is actually applied in the main mpu9250.js file.  But
//   this calibration process outputs those values.
//

#[derive(Clone, Copy, Debug, PartialEq)]
enum Axis {
  X = 0,
  Y = 1,
  Z = 2,
}
#[derive(Clone, Copy, Debug, PartialEq)]
enum Direction {
  Up = 0,
  Down = 1,
}

/**
 * This will syncronuously read the accel data from MPU9250.  It will gather the offset and scalar values.
 */
fn calibrate_accel_axis(
  mpu9250: &mut Mpu9250<I2cdev, Delay>,
  offset: &mut Vector<f32>,
  scale_lo: &mut Vector<f32>,
  scale_hi: &mut Vector<f32>,
  axis: Axis,
  dir: Direction,
) {
  for _ in 0..NUM_ACCEL_READS {
    let va = mpu9250.get_accel().unwrap();

    if axis == Axis::X {
      if dir == Direction::Up {
        scale_hi.x += va.x;
      } else {
        scale_lo.x += va.x;
      }
    } else {
      offset.y += va.y;
      offset.z += va.z;
    }

    if axis == Axis::Y {
      if dir == Direction::Up {
        scale_hi.y += va.y;
      } else {
        scale_lo.y += va.y;
      }
    } else {
      offset.x += va.x;
      offset.z += va.z;
    }

    if axis == Axis::Z {
      if dir == Direction::Up {
        scale_hi.z += va.z;
      } else {
        scale_lo.z += va.z;
      }
    } else {
      offset.x += va.x;
      offset.y += va.y;
    }

    mpu9250.wait(4);
  }
}

/**
 * Set up the next capture for an axis and a direction (up / down).
 */
fn run_next_capture(
  mpu9250: &mut Mpu9250<I2cdev, Delay>,
  offset: &mut Vector<f32>,
  scale_lo: &mut Vector<f32>,
  scale_hi: &mut Vector<f32>,
  axis: Axis,
  dir: Direction,
) {
  const AXES_STR: &[&str] = &["X", "Y", "Z"];
  const DIRS_STR: &[&str] = &["Up", "Down"];

  println!(
    "Point the {} axis arrow {}.",
    AXES_STR[axis as usize], DIRS_STR[dir as usize]
  );

  wait();
  println!("Reading values - hold still");

  calibrate_accel_axis(mpu9250, offset, scale_lo, scale_hi, axis, dir);
}

fn calibrate_accel(mpu9250: &mut Mpu9250<I2cdev, Delay>) {
  println!("--- ACCEL CALIBRATION ---");

  let offset = &mut Vector {
    x: 0.0,
    y: 0.0,
    z: 0.0,
  };
  let scale_lo = &mut Vector {
    x: 0.0,
    y: 0.0,
    z: 0.0,
  };
  let scale_hi = &mut Vector {
    x: 0.0,
    y: 0.0,
    z: 0.0,
  };

  run_next_capture(mpu9250, offset, scale_lo, scale_hi, Axis::X, Direction::Up);
  run_next_capture(
    mpu9250,
    offset,
    scale_lo,
    scale_hi,
    Axis::X,
    Direction::Down,
  );
  run_next_capture(mpu9250, offset, scale_lo, scale_hi, Axis::Y, Direction::Up);
  run_next_capture(
    mpu9250,
    offset,
    scale_lo,
    scale_hi,
    Axis::Y,
    Direction::Down,
  );
  run_next_capture(mpu9250, offset, scale_lo, scale_hi, Axis::Z, Direction::Up);
  run_next_capture(
    mpu9250,
    offset,
    scale_lo,
    scale_hi,
    Axis::Z,
    Direction::Down,
  );

  offset.x /= NUM_ACCEL_READS as f32 * 4.0;
  offset.y /= NUM_ACCEL_READS as f32 * 4.0;
  offset.z /= NUM_ACCEL_READS as f32 * 4.0;
  scale_lo.x /= NUM_ACCEL_READS as f32;
  scale_lo.y /= NUM_ACCEL_READS as f32;
  scale_lo.z /= NUM_ACCEL_READS as f32;
  scale_hi.x /= NUM_ACCEL_READS as f32;
  scale_hi.y /= NUM_ACCEL_READS as f32;
  scale_hi.z /= NUM_ACCEL_READS as f32;

  println!(
    "    accel_offset: Vector {{ x: {:.5}, y: {:.5}, z: {:.5} }},",
    offset.x, offset.y, offset.z
  );
  println!(
    "    accel_scale_lo: Vector {{ x: {:.5}, y: {:.5}, z: {:.5} }},",
    scale_lo.x, scale_lo.y, scale_lo.z
  );
  println!(
    "    accel_scale_hi: Vector {{ x: {:.5}, y: {:.5}, z: {:.5} }},",
    scale_hi.x, scale_hi.y, scale_hi.z
  );
}

//
//  MAGNETOMETER
//
//
//  Once the calibration is started you will want to move the sensor around all axes.  What we want is to find the
//  extremes (min/max) of the x, y, z values such that we can find the offset and scale values.
//
//  These calibration calculations are based on this page:
//  http://www.camelsoftware.com/2016/03/13/imu-maths-calculate-orientation-pt3/
//

fn calibrate_mag(mpu9250: &mut Mpu9250<I2cdev, Delay>) {
  let v_min = &mut Vector {
    x: f32::MAX,
    y: f32::MAX,
    z: f32::MAX,
  };
  let v_max = &mut Vector {
    x: f32::MIN,
    y: f32::MIN,
    z: f32::MIN,
  };

  println!(
    "Rotate the magnometer around all 3 axes, until the min and max values don't change anymore."
  );

  println!("    x        y        z      min x     min y     min z     max x     max y     max z");
  for _ in 0..NUM_MAG_READS {
    let vm = mpu9250.get_mag().unwrap();
    v_min.x = v_min.x.min(vm.x);
    v_min.y = v_min.y.min(vm.y);
    v_min.z = v_min.z.min(vm.z);
    v_max.x = v_max.x.max(vm.x);
    v_max.y = v_max.y.max(vm.y);
    v_max.z = v_max.z.max(vm.z);

    print!(
      " {:.3}    {:.3}    {:.3}    {:.3}   {:.3}   {:.3}   {:.3}   {:.3}   {:.3}       \r",
      vm.x, vm.y, vm.z, v_min.x, v_min.y, v_min.z, v_max.x, v_max.y, v_max.z
    );

    mpu9250.wait(10);
  }

  let v_avg = Vector {
    x: (v_max.x - v_min.x) / 2.0,
    y: (v_max.y - v_min.y) / 2.0,
    z: (v_max.z - v_min.z) / 2.0,
  };
  let avg_radius = (v_avg.x + v_avg.y + v_avg.z) / 3.0;

  println!("");
  println!(
    "    mag_offset: Vector {{ x: {:.5}, y: {:.5}, z: {:.5} }},",
    (v_min.x + v_max.x) / 2.0,
    (v_min.y + v_max.y) / 2.0,
    (v_min.z + v_max.z) / 2.0
  );
  println!(
    "    mag_scale: Vector {{ x: {:.5}, y: {:.5}, z: {:.5} }},",
    avg_radius / v_avg.x,
    avg_radius / v_avg.y,
    avg_radius / v_avg.z,
  );
}
