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

#[allow(unused_imports)]
use hal::{Delay, I2cdev};

use mpu9250_i2c::mpu9250 as device;
use mpu9250_i2c::Mpu9250;

pub struct Print;

impl Print {
  pub fn mpu9250_settings(mpu9250: &mut Mpu9250<hal::I2cdev, hal::Delay>) {
    const CLK_RNG: &[&str] = &[
      "0 (Internal 20MHz oscillator)",
      "1 (Auto selects the best available clock source)",
      "2 (Auto selects the best available clock source)",
      "3 (Auto selects the best available clock source)",
      "4 (Auto selects the best available clock source)",
      "5 (Auto selects the best available clock source)",
      "6 (Internal 20MHz oscillator)",
      "7 (Stops the clock and keeps timing generator in reset)",
    ];
    let cal = mpu9250.get_calibration();

    println!("MPU9250:");
    println!("--> Device address: {:#x}", device::ADDRESS);
    println!("--> Device ID: {:#x}", mpu9250.get_device_id().unwrap());
    println!(
      "--> BYPASS enabled: {}",
      if mpu9250.get_bypass_enabled().unwrap() {
        "Yes"
      } else {
        "No"
      }
    );
    println!(
      "--> SleepEnabled Mode: {}",
      if mpu9250.get_sleep_enabled().unwrap() {
        "On"
      } else {
        "Off"
      }
    );
    println!(
      "--> i2c Master Mode: {}",
      if mpu9250.get_i2c_master_mode().unwrap() {
        "Enabled"
      } else {
        "Disabled"
      }
    );
    println!("--> Power Management (0x6B, 0x6C):");
    println!(
      "  --> Clock Source: {}",
      CLK_RNG[mpu9250.get_clock_source().unwrap() as usize]
    );

    fn yn(b: bool) -> &'static str {
      if b {
        "Yes"
      } else {
        "No"
      }
    }

    let accel_power_settings = mpu9250.get_accel_power_settings().unwrap();
    println!(
      "  --> Accel enabled (x, y, z): ({}, {}, {})",
      yn(accel_power_settings.x),
      yn(accel_power_settings.y),
      yn(accel_power_settings.z)
    );

    let gyro_power_settings = mpu9250.get_gyro_power_settings().unwrap();
    println!(
      "  --> Gyro enabled (x, y, z): ({}, {}, {})",
      yn(gyro_power_settings.x),
      yn(gyro_power_settings.y),
      yn(gyro_power_settings.z)
    );

    //
    // Print Accelerometer settings
    //
    const A_FS_RANGE: &[&str] = &["±2g (0)", "±4g (1)", "±8g (2)", "±16g (3)"];

    println!("Accelerometer:");
    println!(
      "--> Full Scale Range (0x1C): {}",
      A_FS_RANGE[mpu9250.get_full_scale_accel_range().unwrap() as usize]
    );
    println!("--> Scalar: 1/{}", 1.0 / mpu9250.get_accel_inv_scale());
    println!("--> Calibration:");
    println!("  --> Offset: ");
    println!("    --> x: {}", cal.accel_offset.x);
    println!("    --> y: {}", cal.accel_offset.y);
    println!("    --> z: {}", cal.accel_offset.z);
    println!("  --> Scale: ");
    println!(
      "    --> x: ({}, {})",
      cal.accel_scale_lo.x, cal.accel_scale_hi.x
    );
    println!(
      "    --> y: ({}, {})",
      cal.accel_scale_lo.y, cal.accel_scale_hi.y
    );
    println!(
      "    --> z: ({}, {})",
      cal.accel_scale_lo.z, cal.accel_scale_hi.z
    );

    //
    // Print Gyroscope settings
    //
    const G_FS_RANGE: &[&str] = &[
      "+250 dps (0)",
      "+500 dps (1)",
      "+1000 dps (2)",
      "+2000 dps (3)",
    ];

    println!("Gyroscope:");
    println!(
      "--> Full Scale Range (0x1B): {}",
      G_FS_RANGE[mpu9250.get_full_scale_gyro_range().unwrap() as usize]
    );
    println!("--> Scalar: 1/{}", 1.0 / mpu9250.get_gyro_inv_scale());
    println!("--> Bias Offset:");
    println!("  --> x: {}", cal.gyro_bias_offset.x);
    println!("  --> y: {}", cal.gyro_bias_offset.y);
    println!("  --> z: {}", cal.gyro_bias_offset.z);
  }

  pub fn ak8963_settings(mpu9250: &mut Mpu9250<hal::I2cdev, hal::Delay>) {
    const CNTL_MODES: &[&str] = &[
      "0x00 (Power-down mode)",
      "0x01 (Single measurement mode)",
      "0x02 (Continuous measurement mode 1: 8Hz)",
      "0x03 Invalid mode",
      "0x04 (External trigger measurement mode)",
      "0x05 Invalid mode",
      "0x06 (Continuous measurement mode 2: 100Hz)",
      "0x07 Invalid mode",
      "0x08 (Self-test mode)",
      "0x09 Invalid mode",
      "0x0A Invalid mode",
      "0x0B Invalid mode",
      "0x0C Invalid mode",
      "0x0D Invalid mode",
      "0x0E Invalid mode",
      "0x0F Invalid mode",
      "0x0F (Fuse ROM access mode)",
    ];

    let cal = mpu9250.get_calibration();
    let asa = mpu9250.ak8963_get_asa();

    let mode = mpu9250.ak8963_get_cntl().unwrap() as u8;
    println!("Magnetometer (Compass):");
    println!(
      "--> Device ID: {:#x}",
      mpu9250.ak8963_get_device_id().unwrap()
    );
    println!("--> Mode: {:#x}: {}", mode, CNTL_MODES[mode as usize]);
    println!("--> ASA Scalars:");
    println!("  --> x: {}", asa.x);
    println!("  --> y: {}", asa.y);
    println!("  --> z: {}", asa.z);
    println!("--> Offset:");
    println!("  --> x: {}", cal.mag_offset.x);
    println!("  --> y: {}", cal.mag_offset.y);
    println!("  --> z: {}", cal.mag_offset.z);
    println!("--> Scale:");
    println!("  --> x: {}", cal.mag_scale.x);
    println!("  --> y: {}", cal.mag_scale.y);
    println!("  --> z: {}", cal.mag_scale.z);
  }
}
