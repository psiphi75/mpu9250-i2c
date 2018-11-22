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

/// The accelerometer address on the i2c bus.
pub const ADDRESS: u8 = 0x68;

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[allow(missing_docs)]
#[derive(Copy, Clone)]
pub enum Register {
  SMPLRT_DIV = 0x19,
  CONFIG = 0x1a,
  EXT_SENS_DATA_00 = 0x49,
  EXT_SENS_DATA_01 = 0x4a,
  EXT_SENS_DATA_02 = 0x4b,
  EXT_SENS_DATA_03 = 0x4c,
  EXT_SENS_DATA_04 = 0x4d,
  INT_PIN_CFG = 0x37,
  PWR_MGMT_1 = 0x6b,
  PWR_MGMT_2 = 0x6c,
  USER_CTRL = 0x6a,
  WHO_AM_I = 0x75,

  // I2C stuff
  I2C_MST_CTRL = 0x24,
  I2C_MST_STATUS = 0x36,
  I2C_SLV0_ADDR = 0x25,
  I2C_SLV0_CTRL = 0x27,
  I2C_SLV0_DO = 0x63,
  I2C_SLV0_REG = 0x26,
  I2C_SLV1_DO = 0x64,
  I2C_SLV2_DO = 0x65,
  I2C_SLV4_ADDR = 0x31,
  I2C_SLV4_CTRL = 0x34,
  I2C_SLV4_DI = 0x35,
  I2C_SLV4_DO = 0x33,
  I2C_SLV4_REG = 0x32,

  // Accel
  ACCEL_CONFIG_1 = 0x1c,
  ACCEL_CONFIG_2 = 0x1d,
  ACCEL_XOUT_L = 0x3c,
  ACCEL_XOUT_H = 0x3b,
  ACCEL_YOUT_L = 0x3e,
  ACCEL_YOUT_H = 0x3d,
  ACCEL_ZOUT_L = 0x40,
  ACCEL_ZOUT_H = 0x3f,

  // Temp
  TEMP_OUT_H = 0x41,
  TEMP_OUT_L = 0x42,

  // Gyro
  GYRO_CONFIG = 0x1b,
  GYRO_XOUT_H = 0x43,
  GYRO_XOUT_L = 0x44,
  GYRO_YOUT_H = 0x45,
  GYRO_YOUT_L = 0x46,
  GYRO_ZOUT_H = 0x47,
  GYRO_ZOUT_L = 0x48,
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
pub enum RegPwrMgmt1 {
  H_RESET = 0b1000_0000, // Reset the internal registers and restores the default settings. Write a 1 to set the reset, the bit will auto clear.
  SLEEP = 0b0100_0000, // Set to sleep mode (After OTP loads, the PU_SLEEP_MODE bit will be written here)
  CYCLE = 0b0010_0000, // When set, and SLEEP and STANDBY are not set, the chip will cycle between sleep and taking a single sample at a rate determined by LP_ACCEL_ODR register
  GYRO_STANDBY = 0b0001_0000, // When set, the gyro drive and pll circuitry are enabled, but the sense paths are disabled. This is a low power mode that allows quick enabling of the gyros.
  PD_PTAT = 0b0000_1000,      // Power down internal PTAT voltage generator and PTAT ADC
  CLKSEL_0 = 0b0000_0000,     // 0 Internal 20MHz oscillator
  CLKSEL_1 = 0b0000_0001, // 1 Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  CLKSEL_2 = 0b0000_0010, // 2 Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  CLKSEL_3 = 0b0000_0011, // 3 Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  CLKSEL_4 = 0b0000_0100, // 4 Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  CLKSEL_5 = 0b0000_0101, // 5 Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  CLKSEL_6 = 0b0000_0110, // 6 Internal 20MHz oscillator
  CLKSEL_7 = 0b0000_0111, // 7 Stops the clock and keeps timing generator in reset
}

impl RegPwrMgmt1 {
  /// Convert to a usable value
  pub fn addr(&self) -> u8 {
    *self as u8
  }
}

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[allow(missing_docs)]
#[derive(Copy, Clone)]
pub enum RegUserCtrl {
  DMP_EN = 0b1000_0000,         // 7 Reserved
  FIFO_EN = 0b0100_0000,        // 6 Enable FIFO operation mode
  I2C_MST_EN = 0b0010_0000,     // 5 Enable the I2C Master I/F module
  I2C_IF_DIS = 0b0001_0000, // 4 Disable I2C Slave module and put the serial interface in SPI mode only
  DMP_RESET = 0b0000_1000,  // 3 Reserved
  FIFO_RESET = 0b0000_0100, // 2 Reset FIFO module. Reset is asynchronous.
  I2C_MST_RESET = 0b0000_0010, // 1 Reset I2C Master module. Reset is asynchronous.
  SIG_COND_RESET = 0b0000_0001, // 0 Reset all gyro digital signal path, accel digital signal path, and temp digital signal path.
}

impl RegUserCtrl {
  /// Convert to a usable value
  pub fn addr(&self) -> u8 {
    *self as u8
  }
}

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[allow(missing_docs)]
#[derive(Copy, Clone)]
pub enum IntCfg {
  ACTL = 0b1000_0000,              // The logic level for INT pin is active low.
  OPEN = 0b0100_0000,              // INT pin is configured as open drain.
  LATCH_INT_EN = 0b0010_0000,      // INT pin level held until interrupt status is cleared.
  INT_ANYRD_2CLEAR = 0b0001_0000, // Interrupt status is cleared if any read operation is performed.
  ACTL_FSYNC = 0b0000_1000, // The logic level for the FSYNC pin as an interrupt is active low.
  FSYNC_INT_MODE_EN = 0b0000_0100, // This enables the FSYNC pin to be used as an interrupt.
  BYPASS_EN = 0b0000_0010, // When enabled, i2c_master interface pins  will go into ‘bypass mode’
  NONE = 0b0000_0001,      // Reserved
}

impl IntCfg {
  /// Convert to a usable value
  pub fn addr(&self) -> u8 {
    *self as u8
  }
}

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[allow(missing_docs)]
#[derive(Copy, Clone)]
pub enum AccelConfig {
  AX_ST_EN = 0b1000_0000,         // X Accel self-test
  AY_ST_EN = 0b0100_0000,         // Y Accel self-test
  AZ_ST_EN = 0b0010_0000,         // Z Accel self-test
  ACCEL_FS_SEL_2g = 0b0000_0000,  // Accel Full Scale Select: ±2g
  ACCEL_FS_SEL_4g = 0b0000_1000,  // Accel Full Scale Select: ±4g
  ACCEL_FS_SEL_8g = 0b0001_0000,  // Accel Full Scale Select: ±8g
  ACCEL_FS_SEL_16g = 0b0001_1000, // Accel Full Scale Select: ±16g
  NONE_0 = 0b0000_0100,           // Reserved
  NONE_1 = 0b0000_0010,           // Reserved
  NONE_2 = 0b0000_0001,           // Reserved
}

impl AccelConfig {
  /// Convert to a usable value
  pub fn addr(&self) -> u8 {
    *self as u8
  }
}

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[allow(missing_docs)]
#[derive(Copy, Clone)]
pub enum GyroConfig {
  XGYRO_Cten = 0b1000_0000,       // X Gyro self-test
  YGYRO_Cten = 0b0100_0000,       // Y Gyro self-test
  ZGYRO_Cten = 0b0010_0000,       // Z Gyro self-test
  GYRO_FS_SEL_250 = 0b0000_0000,  // Gyro Full Scale Select: +250 dps
  GYRO_FS_SEL_500 = 0b0000_1000,  // Gyro Full Scale Select: +500 dps
  GYRO_FS_SEL_1000 = 0b0001_0000, // Gyro Full Scale Select: +1000 dps
  GYRO_FS_SEL_2000 = 0b0001_1000, // Gyro Full Scale Select: +2000 dps
  NONE = 0b0000_0100,             // Reserved
  Fchoice_b1 = 0b0000_0010,       // Used to bypass DLPF
  Fchoice_b0 = 0b0000_0001,       // Used to bypass DLPF
}

impl GyroConfig {
  /// Convert to a usable value
  pub fn addr(&self) -> u8 {
    *self as u8
  }
}
