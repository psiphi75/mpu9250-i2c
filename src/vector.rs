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
