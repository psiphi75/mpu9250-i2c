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

extern crate embedded_hal;

use self::embedded_hal::blocking::i2c::{Read, Write, WriteRead};

#[derive(Clone, Copy)]
pub struct I2CTools<I2C> {
  i2c: I2C,
}

impl<I2C, E> I2CTools<I2C>
where
  I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
{
  /// Creates a new driver
  pub fn new(i2c: I2C) -> Result<Self, E> {
    Ok(Self { i2c })
  }

  pub fn write_byte(&mut self, address: u8, reg: u8, val: u8) -> Result<(), E> {
    let bytes = [reg, val];
    self.i2c.write(address, &bytes)
  }

  pub fn read_byte(&mut self, address: u8, reg: u8) -> Result<u8, E> {
    let bytes: &mut [u8] = &mut [0; 1];
    self.read_bytes(address, reg, bytes)?;
    Ok(bytes[0])
  }

  pub fn read_bytes(&mut self, address: u8, reg: u8, bytes: &mut [u8]) -> Result<(), E> {
    let cmd_bytes = [reg];
    self.i2c.write_read(address, &cmd_bytes, bytes)
  }

  pub fn u8_to_i16_be(&self, byte: &mut [u8], offset: usize) -> i16 {
    (i16::from(byte[offset]) << 8) + i16::from(byte[offset + 1])
  }
  pub fn u8_to_i16_le(&self, byte: &mut [u8], offset: usize) -> i16 {
    (i16::from(byte[offset + 1]) << 8) + i16::from(byte[offset])
  }
}
