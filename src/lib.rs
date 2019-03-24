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

//! # mpu9250-i2c
//!
//! MPU9250 driver for embedded devices and Linux written in Rust.
//!
//! A platform agnostic driver to interface with the MPU9250 over i2c.
//! This driver can access the following components of the MPU9250:
//!
//! - 3-axis gyroscope
//! - 3-axis compass (magenetometer)
//! - 3-axis accelerometer
//! - Temperature device
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/
//!
//! ## Examples
//!
//! Example code can be found in the /src/bin folder, this samples work
//! in Linux devices with an i2c bus, like the Raspberry Pi and BeagleBone.
//! The sample includes:
//!
//! - calibrate.rs - code to calibrate the MPU9250 device.
//! - mpu9250.rs - basic example that reads all data from the device.
//! - ahrs.rs - Fully functional AHRS algorithm, uses the Magwick filter.
//!
//! ## Linux Example
//!
//! ```
//! extern crate linux_embedded_hal as hal;
//! extern crate mpu9250_i2c as mpu9250_i2c;
//! use hal::{Delay, I2cdev};
//! use mpu9250::{calibration::Calibration, Mpu9250};
//!
//! fn main() {
//!
//!   // Linux device
//!   let dev = I2cdev::new("/dev/i2c-2").unwrap();
//!
//!   // Set the calibration to the default setting.  This can
//!   // be set to a custom value specific for the device.
//!   let cal = Calibration {
//!     ..Default::default()
//!   };
//!
//!   let mpu9250 = &mut Mpu9250::new(dev, Delay, cal).unwrap();
//!
//!   // Initialise with default settings
//!   mpu9250.init().unwrap();
//!
//!   // Probe the temperature
//!   let temp = mpu9250.get_temperature_celcius().unwrap();
//! }
//! ```
//!
//! ## Calibration
//!
//! The technology used in these devices is very noisy. Each component
//! requires a different calibration method. Compile the `calibrate.rs`
//! file which will produce a executable called `calibrate`.
//! When run, `calibrate` will give you instructions on how to calibrate.
//! Then after the calibration step for each component the calibration
//! settings unique for that device are printed to the console. These
//! values can be used in your code.
//! The calibration is temperature sensitive. Hence if you want to be
//! very precise you should repeat the calibration at different temperatures.
//! At a high-level the calibration does the following:
//!
//! - Gyroscope: the average value is taken, this is called the bias.
//! - Accelerometer: the scale of the accelerometer at rest should
//!   range from -1.0g to 1.0g, where g is the 9.81 m/s. The
//!   calibration will take this into account. As well as
//!   the noise on orthogonal axes.
//! - Magnetometer: this component can be very different for each device.
//!   This calibration will ensure that it's extremeties are
//!   discovered. You will need to rotate the device around
//!   in all directions.
//!
//! ## MPU9250 documentation
//!
//! https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/
//!
//! https://www.invensense.com/wp-content/uploads/2015/02/MPU-9250-Datasheet.pdf
//!
//! https://www.invensense.com/wp-content/uploads/2015/02/MPU-9250-Register-Map.pdf
//!
//! ## License
//!
//! Copyright 2018 Simon M. Werner
//!
//! Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
//! except in compliance with the License.  
//! You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0. Unless
//! required by applicable law or agreed to in writing, software distributed under the License
//! is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
//! express or implied. See the License for the specific language governing permissions and
//! limitations under the License.

#![no_std]
#![deny(missing_docs)]

extern crate embedded_hal;

mod i2c_tools;
mod mag;

/// Calibration module required to calibrate devices
pub mod calibration;

/// The MPU9250 accelerometer and gyroscope register values
pub mod mpu9250;

/// Simple Vector
pub mod vector;

use self::embedded_hal::blocking::delay::DelayMs;
use self::embedded_hal::blocking::i2c::{Read, Write, WriteRead};

use calibration::Calibration;
use i2c_tools::I2CTools;
use vector::Vector;

/// The MPU9250, this is where all the work is done.
pub struct Mpu9250<I, D> {
  /// The i2c driver, depends on platform
  i2c: I2CTools<I>,

  /// The delay component, depends on platform.
  delay: D,

  /// The Accerometer scale factor
  accel_inv_scale: f32,

  /// The Gyroscope scale factor
  gyro_inv_scale: f32,

  /// The calibration settings
  cal: Calibration,

  /// The Magenetomer sensitivity adjustment values
  asa: Vector<f32>,
}

#[allow(dead_code)]
impl<I, D, E> Mpu9250<I, D>
where
  I: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
  D: DelayMs<u8>,
{
  /// Creates a new driver for the MPU 9250.
  pub fn new(dev: I, delay: D, cal: Calibration) -> Result<Self, E> {
    Ok(Self {
      i2c: I2CTools::new(dev)?,
      delay,
      accel_inv_scale: 1.0,
      gyro_inv_scale: 1.0,
      cal,
      asa: Vector {
        x: 0.0,
        y: 0.0,
        z: 0.0,
      },
    })
  }

  /// Initialise the device with by performing the following
  ///   - a soft reset
  ///   - setting the clock source
  ///   - setting the Accelerometer and Gyroscope Full Scale ranges
  ///   - Enabling the magenetometer
  pub fn init(&mut self) -> Result<(), E> {
    // soft reset the device
    self.soft_reset()?;

    // Set to 400 kHz
    self.i2c.write_byte(
      mpu9250::ADDRESS,
      mpu9250::Register::I2C_MST_CTRL.addr(),
      0x0d,
    )?;

    // define clock source
    self.set_clock_source(mpu9250::RegPwrMgmt1::CLKSEL_1.addr())?;

    // define gyro range
    self.set_full_scale_gyro_range(mpu9250::GyroConfig::GYRO_FS_SEL_250)?;

    // define accel range
    self.set_full_scale_accel_range(mpu9250::AccelConfig::ACCEL_FS_SEL_2g)?;

    self.enable_magnetometer()?;

    Ok(())
  }

  /// Wait for the given amount of time in milliseconds.
  pub fn wait(&mut self, ms: u8) {
    self.delay.delay_ms(ms);
  }

  /// Get the update rate in milliseconds
  pub fn get_accel_gyro_rate_ms(&mut self) -> u64 {
    4 // FIXME: time in milliseconds between reads, may be dependant on clock settings
  }

  /// Perform a soft reset
  fn soft_reset(&mut self) -> Result<(), E> {
    self.i2c.write_byte(
      mpu9250::ADDRESS,
      mpu9250::Register::PWR_MGMT_1.addr(),
      mpu9250::RegPwrMgmt1::H_RESET.addr(),
    )?;
    self.delay.delay_ms(10);
    Ok(())
  }

  /// Set the clock source
  pub fn set_clock_source(&mut self, src: u8) -> Result<(), E> {
    self
      .i2c
      .write_byte(mpu9250::ADDRESS, mpu9250::Register::PWR_MGMT_1.addr(), src)
  }

  /// Get the clock source
  pub fn get_clock_source(&mut self) -> Result<u8, E> {
    let byte = self
      .i2c
      .read_byte(mpu9250::ADDRESS, mpu9250::Register::PWR_MGMT_1.addr())?;
    Ok(byte & 0x07)
  }

  /// Set the inverse scale factor.  We set it once as an inverse such that we don't
  /// need to calcualte it all the time.
  fn set_gyro_inv_scale(&mut self, scale_factor: mpu9250::GyroConfig) {
    self.gyro_inv_scale = match scale_factor {
      mpu9250::GyroConfig::GYRO_FS_SEL_250 => 1.0 / 131.0,
      mpu9250::GyroConfig::GYRO_FS_SEL_500 => 1.0 / 65.5,
      mpu9250::GyroConfig::GYRO_FS_SEL_1000 => 1.0 / 32.8,
      mpu9250::GyroConfig::GYRO_FS_SEL_2000 => 1.0 / 16.4,
      _ => 1.0,
    };
  }

  /// Set the Full Scale range and calculate the scale factor
  pub fn set_full_scale_gyro_range(&mut self, scale_factor: mpu9250::GyroConfig) -> Result<(), E> {
    self.set_gyro_inv_scale(scale_factor);
    self.i2c.write_byte(
      mpu9250::ADDRESS,
      mpu9250::Register::GYRO_CONFIG.addr(),
      scale_factor.addr(),
    )
  }

  /// Read the Full Scale range for the gyro from the device
  pub fn get_full_scale_gyro_range(&mut self) -> Result<(u8), E> {
    let mut byte = self
      .i2c
      .read_byte(mpu9250::ADDRESS, mpu9250::Register::GYRO_CONFIG.addr())?;

    byte &= 0b0001_1000;
    byte >>= 3;

    Ok(byte)
  }

  /// Set the inverse scale factor.  We set it once as an inverse such that we don't
  /// need to calcualte it all the time.
  fn set_accel_inv_scale(&mut self, scale_factor: mpu9250::AccelConfig) {
    self.accel_inv_scale = match scale_factor {
      mpu9250::AccelConfig::ACCEL_FS_SEL_2g => 1.0 / 16384.0,
      mpu9250::AccelConfig::ACCEL_FS_SEL_4g => 1.0 / 8192.0,
      mpu9250::AccelConfig::ACCEL_FS_SEL_8g => 1.0 / 4096.0,
      mpu9250::AccelConfig::ACCEL_FS_SEL_16g => 1.0 / 2048.0,
      _ => 1.0,
    }
  }

  /// Set the Full Scale range and calculate the scale factor
  pub fn set_full_scale_accel_range(
    &mut self,
    scale_factor: mpu9250::AccelConfig,
  ) -> Result<(), E> {
    self.set_accel_inv_scale(scale_factor);
    self.i2c.write_byte(
      mpu9250::ADDRESS,
      mpu9250::Register::ACCEL_CONFIG_1.addr(),
      scale_factor.addr(),
    )
  }

  /// Read the Full Scale range for the accelerometer from the device
  pub fn get_full_scale_accel_range(&mut self) -> Result<(u8), E> {
    let mut byte = self
      .i2c
      .read_byte(mpu9250::ADDRESS, mpu9250::Register::ACCEL_CONFIG_1.addr())?;

    byte &= 0b0001_1000;
    byte >>= 3;

    Ok(byte)
  }

  /// Set the sleep enabled byte, read MPU9250 hardware docs for more details
  pub fn set_sleep_enabled(&mut self) -> Result<(), E> {
    self.i2c.write_byte(
      mpu9250::ADDRESS,
      mpu9250::Register::PWR_MGMT_1.addr(),
      mpu9250::RegPwrMgmt1::SLEEP.addr(),
    )
  }

  /// Get the sleep enabled byte.
  pub fn get_sleep_enabled(&mut self) -> Result<bool, E> {
    let byte = self
      .i2c
      .read_byte(mpu9250::ADDRESS, mpu9250::Register::PWR_MGMT_1.addr())?;
    Ok(byte & mpu9250::RegPwrMgmt1::SLEEP.addr() != 0)
  }

  /// Put the device into i2c master mode.
  pub fn set_i2c_master_mode(&mut self) -> Result<(), E> {
    self.i2c.write_byte(
      mpu9250::ADDRESS,
      mpu9250::Register::USER_CTRL.addr(),
      mpu9250::RegUserCtrl::I2C_MST_EN.addr(),
    )
  }

  /// Returns `true` if the device is in i2c master mode
  pub fn get_i2c_master_mode(&mut self) -> Result<bool, E> {
    let byte = self
      .i2c
      .read_byte(mpu9250::ADDRESS, mpu9250::Register::USER_CTRL.addr())?;
    Ok(byte & mpu9250::RegUserCtrl::I2C_MST_EN.addr() != 0)
  }

  /// Get the power management settings for the gyroscope
  pub fn get_gyro_power_settings(&mut self) -> Result<Vector<bool>, E> {
    let mut byte = self
      .i2c
      .read_byte(mpu9250::ADDRESS, mpu9250::Register::PWR_MGMT_2.addr())?;

    byte &= 0x07;

    Ok(Vector {
      x: (byte >> 2) & 0x01 == 0x00,
      y: (byte >> 1) & 0x01 == 0x00,
      z: byte & 0x01 == 0x00,
    })
  }

  /// Get the power management settings for the accelerometer
  pub fn get_accel_power_settings(&mut self) -> Result<Vector<bool>, E> {
    let mut byte = self
      .i2c
      .read_byte(mpu9250::ADDRESS, mpu9250::Register::PWR_MGMT_2.addr())?;

    byte &= 0x38;
    Ok(Vector {
      x: (byte >> 5) & 0x01 == 0x00,
      y: (byte >> 4) & 0x01 == 0x00,
      z: (byte >> 3) & 0x01 == 0x00,
    })
  }

  /// Apply the calibration to the accelerometer
  fn accel_apply_calibration(&self, value: f32, offset: f32, scale_lo: f32, scale_hi: f32) -> f32 {
    if value < 0.0 {
      -(value * self.accel_inv_scale - offset) / (scale_lo - offset)
    } else {
      (value * self.accel_inv_scale - offset) / (scale_hi - offset)
    }
  }

  /// Read the raw accel data and convert it to a Vector
  fn align_accel(&self, bytes: &mut [u8]) -> Vector<f32> {
    let xi: i16 = self.i2c.u8_to_i16_be(bytes, 0);
    let yi: i16 = self.i2c.u8_to_i16_be(bytes, 2);
    let zi: i16 = self.i2c.u8_to_i16_be(bytes, 4);

    Vector {
      x: self.accel_apply_calibration(
        f32::from(xi),
        self.cal.accel_offset.x,
        self.cal.accel_scale_lo.x,
        self.cal.accel_scale_hi.x,
      ),
      y: self.accel_apply_calibration(
        f32::from(yi),
        self.cal.accel_offset.y,
        self.cal.accel_scale_lo.y,
        self.cal.accel_scale_hi.y,
      ),
      z: self.accel_apply_calibration(
        f32::from(zi),
        self.cal.accel_offset.z,
        self.cal.accel_scale_lo.z,
        self.cal.accel_scale_hi.z,
      ),
    }
  }

  /// Get the accelerometer data.
  ///
  /// Returns a vector and the units are in g.
  pub fn get_accel(&mut self) -> Result<Vector<f32>, E> {
    let bytes: &mut [u8] = &mut [0; 6];
    self.i2c.read_bytes(
      mpu9250::ADDRESS,
      mpu9250::Register::ACCEL_XOUT_H.addr(),
      bytes,
    )?;
    Ok(self.align_accel(bytes))
  }

  fn align_gyro(&self, bytes: &mut [u8], offset: usize) -> Vector<f32> {
    let xi: i16 = self.i2c.u8_to_i16_be(bytes, offset);
    let yi: i16 = self.i2c.u8_to_i16_be(bytes, 2 + offset);
    let zi: i16 = self.i2c.u8_to_i16_be(bytes, 4 + offset);

    Vector {
      x: f32::from(xi) * self.gyro_inv_scale + self.cal.gyro_bias_offset.x,
      y: f32::from(yi) * self.gyro_inv_scale + self.cal.gyro_bias_offset.y,
      z: f32::from(zi) * self.gyro_inv_scale + self.cal.gyro_bias_offset.z,
    }
  }

  /// Get the gyroscope data.
  ///
  /// Returns a vector and the units are in degrees / second.
  pub fn get_gyro(&mut self) -> Result<Vector<f32>, E> {
    let bytes: &mut [u8] = &mut [0; 6];
    self.i2c.read_bytes(
      mpu9250::ADDRESS,
      mpu9250::Register::GYRO_XOUT_H.addr(),
      bytes,
    )?;
    Ok(self.align_gyro(bytes, 0))
  }

  /// Get the accelerometer and gyroscope data.  This is more efficient
  /// than reading the accelerometer and gyroscope individually.
  ///
  /// The results are written into the two vectors proviced `va` and `vg`.
  pub fn get_accel_gyro(&mut self) -> Result<(Vector<f32>, Vector<f32>), E> {
    let bytes: &mut [u8] = &mut [0; 14];
    self.i2c.read_bytes(
      mpu9250::ADDRESS,
      mpu9250::Register::ACCEL_XOUT_H.addr(),
      bytes,
    )?;

    // Accelerometer - bytes 0:5
    let mut v = self.align_accel(bytes);
    let va = Vector {
      x: v.x,
      y: v.y,
      z: v.z,
    };

    // Skip Temperature - bytes 6:7

    // Gyroscope - bytes 8:13
    v = self.align_gyro(bytes, 8);
    let vg = Vector {
      x: v.x,
      y: v.y,
      z: v.z,
    };

    Ok((va, vg))
  }

  /// Get the raw temperature data
  pub fn get_temperature_raw(&mut self) -> Result<i16, E> {
    let bytes: &mut [u8] = &mut [0; 2];
    self.i2c.read_bytes(
      mpu9250::ADDRESS,
      mpu9250::Register::TEMP_OUT_H.addr(),
      bytes,
    )?;
    Ok(self.i2c.u8_to_i16_be(bytes, 0))
  }

  /// Get the temperature in degrees Celcius
  pub fn get_temperature_celsius(&mut self) -> Result<f32, E> {
    let raw_temp = self.get_temperature_raw()?;
    Ok(f32::from(raw_temp) / 333.87 + 21.0)
  }

  /// Get the device id
  pub fn get_device_id(&mut self) -> Result<u8, E> {
    self
      .i2c
      .read_byte(mpu9250::ADDRESS, mpu9250::Register::WHO_AM_I.addr())
  }

  /// enable bypass, read the MPU9250 docs.
  pub fn set_bypass_enabled(&mut self, state: bool) -> Result<(), E> {
    self.i2c.write_byte(
      mpu9250::ADDRESS,
      mpu9250::Register::INT_PIN_CFG.addr(),
      if state {
        mpu9250::IntCfg::BYPASS_EN.addr()
      } else {
        0x00
      },
    )
  }

  /// Check if bypass is enabled
  pub fn get_bypass_enabled(&mut self) -> Result<bool, E> {
    let byte = self
      .i2c
      .read_byte(mpu9250::ADDRESS, mpu9250::Register::INT_PIN_CFG.addr())?;
    Ok(byte & mpu9250::IntCfg::BYPASS_EN.addr() != 0)
  }

  /// Enable the magnetometer.  This will set the bypass, then run `ak8963_init()`.
  pub fn enable_magnetometer(&mut self) -> Result<(bool), E> {
    self.set_bypass_enabled(true)?;
    self.delay.delay_ms(10);

    self.ak8963_init()?;
    if self.get_bypass_enabled()? {
      self.ak8963_init()?;
      Ok(true)
    } else {
      Ok(false)
    }
  }

  /// Initialise the magnetometer
  pub fn ak8963_init(&mut self) -> Result<(), E> {
    self.ak8963_update_sensitivity_adjustment_values()?;
    self.delay.delay_ms(10);

    // 100 Hz continuous measurement, 16-bit
    self.ak8963_set_cntl(mag::Ctnl1::MODE_CONTINUE_MEASURE_2)
  }

  /// Get the magnetomter device Id
  pub fn ak8963_get_device_id(&mut self) -> Result<(u8), E> {
    self
      .i2c
      .read_byte(mag::ADDRESS, mag::Register::WHO_AM_I.addr())
  }

  /// Get and update the magnetometer sensitivity adjustment values
  pub fn ak8963_update_sensitivity_adjustment_values(&mut self) -> Result<(), E> {
    // Need to set to Fuse mode to get valid values from this.
    let current_mode = self.ak8963_get_cntl()?;
    self.ak8963_set_cntl(mag::Ctnl1::MODE_FUSE_ROM_ACCESS)?;
    self.delay.delay_ms(20);

    let x = self
      .i2c
      .read_byte(mag::ADDRESS, mag::Register::ASA_X.addr())?;
    let y = self
      .i2c
      .read_byte(mag::ADDRESS, mag::Register::ASA_Y.addr())?;
    let z = self
      .i2c
      .read_byte(mag::ADDRESS, mag::Register::ASA_Z.addr())?;

    // Get the ASA* values
    self.asa.x = ((f32::from(x) - 128.0) * 0.5) / 128.0 + 1.0;
    self.asa.y = ((f32::from(y) - 128.0) * 0.5) / 128.0 + 1.0;
    self.asa.z = ((f32::from(z) - 128.0) * 0.5) / 128.0 + 1.0;

    self.ak8963_set_cntl(current_mode)
  }

  /// Get the magnetometer data.  Returns values in degrees per second.
  ///
  /// Note, this will align the orientation of the magnetometer's reference
  /// frame to the same as the accelerometer and gyroscope.  Read the "Orientation of Axes"
  /// section of the Mpu9250 vendor documentation.
  pub fn get_mag(&mut self) -> Result<Vector<f32>, E> {
    let bytes: &mut [u8] = &mut [0; 6];

    self.ak8963_get_mag_raw(bytes)?;

    let xi = f32::from(self.i2c.u8_to_i16_le(bytes, 0));
    let yi = f32::from(self.i2c.u8_to_i16_le(bytes, 2));
    let zi = f32::from(self.i2c.u8_to_i16_le(bytes, 4));

    // Orientate the magnetometer to the same reference frame as the accelerometer
    // and gyroscope.

    let (xi, yi, zi) = (yi, xi, -zi);

    Ok(Vector {
      x: (xi * self.asa.x - self.cal.mag_offset.x) * self.cal.mag_scale.x,
      y: (yi * self.asa.y - self.cal.mag_offset.y) * self.cal.mag_scale.y,
      z: (zi * self.asa.z - self.cal.mag_offset.z) * self.cal.mag_scale.z,
    })
  }

  /// Get the raw magnetometer data
  /// This function has an intentional delay of 1 millisecond.
  pub fn ak8963_get_mag_raw(&mut self, bytes: &mut [u8]) -> Result<(), E> {
    self
      .i2c
      .read_bytes(mag::ADDRESS, mag::Register::XOUT_L.addr(), bytes)?;

    // For some reason when we read ST2 (Status 2) just after reading byte, this ensures the
    // next reading is fresh.  If we do it before without a pause, only 1 in 15 readings will
    // be fresh.  The setTimeout ensures this read goes to the back of the queue, once all other
    // computation is done.
    self.delay.delay_ms(1);
    self
      .i2c
      .read_byte(mag::ADDRESS, mag::Register::ST2.addr())?;

    Ok(())
  }

  /// Get the magnetometer control details
  pub fn ak8963_get_cntl(&mut self) -> Result<(mag::Ctnl1), E> {
    Ok(mag::Ctnl1::from(
      self
        .i2c
        .read_byte(mag::ADDRESS, mag::Register::CNTL.addr())?,
    ))
  }

  /// Set the magnetometer control details
  pub fn ak8963_set_cntl(&mut self, mode: mag::Ctnl1) -> Result<(), E> {
    self
      .i2c
      .write_byte(mag::ADDRESS, mag::Register::CNTL.addr(), mode.addr())
  }

  /// Set the currently used calibration values
  pub fn get_calibration(&mut self) -> Calibration {
    Calibration::copy(&self.cal)
  }

  /// Get the sensitivity adjustment values
  pub fn ak8963_get_asa(&mut self) -> Vector<f32> {
    Vector::copy(&self.asa)
  }

  /// Get the accelometer scale factor
  pub fn get_accel_inv_scale(&mut self) -> f32 {
    self.accel_inv_scale
  }

  /// Get the gyrscope scale factor
  pub fn get_gyro_inv_scale(&mut self) -> f32 {
    self.gyro_inv_scale
  }
}
