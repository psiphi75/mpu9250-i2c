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

/// The magnetometer address on the i2c bus.  This may not be initially visible.
pub const ADDRESS: u8 = 0x0c;

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[allow(missing_docs)]
#[derive(Copy, Clone)]
pub enum Register {
  WHO_AM_I = 0x00, // should return 0x48
  WHO_AM_I_RESPONSE = 0x48,
  INFO = 0x01,
  ST1 = 0x02,    // data ready status bit 0
  XOUT_L = 0x03, // data
  XOUT_H = 0x04,
  YOUT_L = 0x05,
  YOUT_H = 0x06,
  ZOUT_L = 0x07,
  ZOUT_H = 0x08,
  ST2 = 0x09,    // Data overflow bit 3 and data read error status bit 2
  CNTL = 0x0a, // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
  ASTC = 0x0c, // Self test control
  I2CDIS = 0x0f, // I2C disable
  ASA_X = 0x10, // Fuse ROM x-axis sensitivity adjustment value
  ASA_Y = 0x11, // Fuse ROM y-axis sensitivity adjustment value
  ASA_Z = 0x12,
}

impl Register {
  /// Convert to a usable value
  pub fn addr(&self) -> u8 {
    *self as u8
  }
}

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[allow(missing_docs)]
#[derive(Copy, Clone)]
pub enum Ctnl1 {
  MODE_POWER_DOWN = 0b0000_0000,         // Power-down mode
  MODE_SINGLE_MEASURE = 0b0000_0001,     // Single measurement mode
  MODE_CONTINUE_MEASURE_1 = 0b0000_0010, // Continuous measurement mode 1
  MODE_CONTINUE_MEASURE_2 = 0b0000_0110, // Continuous measurement mode 2
  MODE_EXT_TRIG_MEASURE = 0b0000_0100,   // External trigger measurement mode
  MODE_SELF_TEST_MODE = 0b0000_1000,     // Self-test mode
  MODE_FUSE_ROM_ACCESS = 0b0000_1111,    // Fuse ROM access mode
  BIT_16 = 0b0001_0000,                  // 16 bit output, otherwise 14 bit output
  NONE = 0xff,
}

impl From<u8> for Ctnl1 {
  fn from(n: u8) -> Ctnl1 {
    match n {
      0b0000_0000 => Ctnl1::MODE_POWER_DOWN,
      0b0000_0001 => Ctnl1::MODE_SINGLE_MEASURE,
      0b0000_0010 => Ctnl1::MODE_CONTINUE_MEASURE_1,
      0b0000_0110 => Ctnl1::MODE_CONTINUE_MEASURE_2,
      0b0000_0100 => Ctnl1::MODE_EXT_TRIG_MEASURE,
      0b0000_1000 => Ctnl1::MODE_SELF_TEST_MODE,
      0b0000_1111 => Ctnl1::MODE_FUSE_ROM_ACCESS,
      0b0001_0000 => Ctnl1::BIT_16,
      _ => Ctnl1::NONE,
    }
  }
}

impl Ctnl1 {
  /// Convert to a usable value
  pub fn addr(&self) -> u8 {
    *self as u8
  }
}
