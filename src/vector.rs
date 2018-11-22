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

/// Simple vector structure
pub struct Vector<T: Copy> {
  /// x-axis
  pub x: T,
  /// y-axis
  pub y: T,
  /// z-axis
  pub z: T,
}

impl<T: Copy> Vector<T> {
  /// Make a copy of the vector.
  pub fn copy(v: &Self) -> Vector<T> {
    Vector {
      x: v.x,
      y: v.y,
      z: v.z,
    }
  }
}
