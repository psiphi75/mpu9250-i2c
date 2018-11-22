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

use vector::Vector;

/// Calibration struct for the mag (magnetometer/compass), acceleromter and gyroscope.
/// Need to run the `calibrate` executable to obtain these values.
pub struct Calibration {
  // Magnetometer
  /// The mag offset/bias values
  pub mag_offset: Vector<f32>,
  /// Scale the g value
  pub mag_scale: Vector<f32>,

  // Gryoscope
  /// The average value at rest
  pub gyro_bias_offset: Vector<f32>,

  // Accelerometer
  /// Offset
  pub accel_offset: Vector<f32>,

  /// Lower scale factor value
  pub accel_scale_lo: Vector<f32>,

  /// Higher scale factor value
  pub accel_scale_hi: Vector<f32>,
}

impl Calibration {
  /// Make a copy of the calibration
  pub fn copy(cal: &Self) -> Self {
    Calibration {
      mag_offset: Vector::copy(&cal.mag_offset),
      mag_scale: Vector::copy(&cal.mag_scale),

      // Gryoscope
      gyro_bias_offset: Vector::copy(&cal.gyro_bias_offset),

      // Accelerometer
      accel_offset: Vector::copy(&cal.accel_offset),
      accel_scale_lo: Vector::copy(&cal.accel_scale_lo),
      accel_scale_hi: Vector::copy(&cal.accel_scale_hi),
    }
  }
}

impl Default for Calibration {
  /// The default calibration.  This is like having no calibration in place.
  fn default() -> Calibration {
    Calibration {
      mag_offset: Vector {
        x: 0.0,
        y: 0.0,
        z: 0.0,
      },
      mag_scale: Vector {
        x: 1.0,
        y: 1.0,
        z: 1.0,
      },

      // Gryoscope
      gyro_bias_offset: Vector {
        x: 0.0,
        y: 0.0,
        z: 0.0,
      },

      // Accelerometer
      accel_offset: Vector {
        x: 0.0,
        y: 0.0,
        z: 0.0,
      },
      accel_scale_lo: Vector {
        x: -1.0,
        y: -1.0,
        z: -1.0,
      },
      accel_scale_hi: Vector {
        x: 1.0,
        y: 1.0,
        z: 1.0,
      },
    }
  }
}
