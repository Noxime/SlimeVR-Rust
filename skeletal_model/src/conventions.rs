//! This module describes the mathematical conventions we adopt throughout the skeletal
//! model.
//!
//! # Coordinate system
//! We adopt a right hand rule coordinate system:
//! - Your pointer finger is `+X`, which points "right"
//! - Your middle finger is `+Y`, which points "up"
//! - Your thumb is `+Z`, which points "backward". `-Z` points "forward".
//!
//! This convention is the same as other libraries like SteamVR and OpenGL.
//!
//! # Rotation representation
//! We use quaternions to represent rotations whenever possible. We try not to use
//! euler angles in our implementation to avoid possible gimbal lock issues.

#[allow(unused)]
use crate::prelude::*;

use nalgebra::{Unit, Vector3};
use num_traits::Zero;

/// A vector in the "up" or `+Y` direction
#[inline]
pub fn up_vec() -> Unit<Vector3<f32>> {
	Vector3::y_axis()
}

/// A vector in the "forward" or `-Z` direction
#[inline]
pub fn forward_vec() -> Unit<Vector3<f32>> {
	-Vector3::z_axis()
}

/// A vector in the "right" or `+X` direction
#[inline]
pub fn right_vec() -> Unit<Vector3<f32>> {
	Vector3::x_axis()
}

/// Creates a [`UnitQuat`] that corresponds to the local frame of an observer standing
/// at the origin and looking toward `dir`. This is a version of
/// [`UnitQuat::face_towards()`] that better matches our mathematical conventions.
///
/// It maps [`forward_vec()`] (-Z axis) to the direction `dir`.
///
/// # Arguments
/// * dir - The look direction. It does not need to be normalized.
/// * up - The vertical direction. It does not need to be normalized.
///   The only requirement of this parameter is to not be collinear to `dir`.
///   Non-collinearity is not guaranteed to be checked.
///
/// # Example
/// ```
/// # use approx::assert_relative_eq;
/// # use nalgebra::{Unit, Vector3};
/// # use skeletal_model::prelude::UnitQuat;
/// # use skeletal_model::conventions::{up_vec, forward_vec, look_towards};
/// let dir = Vector3::new(1.0, 2.0, 3.0);
/// let q = look_towards(&dir, &up_vec());
///
/// // Make `dir` have a length of one, also known as a unit vector.
/// let unit_dir = Unit::new_normalize(dir);
/// assert_relative_eq!((q * forward_vec()), unit_dir);
/// ```
#[inline]
pub fn look_towards(dir: &Vector3<f32>, up: &Vector3<f32>) -> UnitQuat {
	debug_assert!(
		approx::relative_ne!(dir.cross(up), Vector3::zero()),
		"`dir` and `up` were collinear!",
	);
	UnitQuat::face_towards(&-dir, up)
}

#[cfg(test)]
mod tests {
	use crate::prelude::*;

	use approx::assert_relative_eq;
	use nalgebra::Vector3;
	use std::f32::consts::FRAC_PI_2;

	use super::look_towards;

	/// Example and sanity check of how to use various functions from `nalgebra` to
	/// describe rotations.
	#[test]
	fn check_rotation_builders() {
		/// Contains a group of rotations generated by various functions. Each should be
		/// identical.
		struct Rotations {
			/// The description of the rotation
			desc: &'static str,
			/// The result of [`look_towards()`]
			look_towards: UnitQuat,
			/// The result of [`UnitQuat::look_at_rh()`]
			look_at: UnitQuat,
			/// The result of [`UnitQuat::face_towards()`]
			face_towards: UnitQuat,
			/// The result of [`UnitQuat::from_axis_angle`]
			axis_angle: UnitQuat,
			/// The result of [`UnitQuat::from_euler_angles`]
			euler: UnitQuat,
		}

		// Build the set of rotations to check
		let rotations = vec![
			Rotations {
				desc: "Pitch up 90 degrees",
				look_towards: look_towards(&up_vec(), &-forward_vec()),
				look_at: UnitQuat::look_at_rh(&-Vector3::y_axis(), &-Vector3::z_axis()),
				face_towards: UnitQuat::face_towards(
					&-Vector3::y_axis(),
					&Vector3::z_axis(),
				),
				axis_angle: UnitQuat::from_axis_angle(&Vector3::x_axis(), FRAC_PI_2),
				euler: UnitQuat::from_euler_angles(FRAC_PI_2, 0., 0.),
			},
			Rotations {
				desc: "Pitch down 90 degrees",
				look_towards: look_towards(&-up_vec(), &forward_vec()),
				look_at: UnitQuat::look_at_rh(&Vector3::y_axis(), &Vector3::z_axis()),
				face_towards: UnitQuat::face_towards(
					&Vector3::y_axis(),
					&-Vector3::z_axis(),
				),
				axis_angle: UnitQuat::from_axis_angle(&Vector3::x_axis(), -FRAC_PI_2),
				euler: UnitQuat::from_euler_angles(-FRAC_PI_2, 0., 0.),
			},
			Rotations {
				desc: "Yaw right 90 degrees",
				look_towards: look_towards(&right_vec(), &up_vec()),
				look_at: UnitQuat::look_at_rh(&-Vector3::x_axis(), &Vector3::y_axis()),
				face_towards: UnitQuat::face_towards(
					&-Vector3::x_axis(),
					&Vector3::y_axis(),
				),
				axis_angle: UnitQuat::from_axis_angle(&Vector3::y_axis(), -FRAC_PI_2),
				euler: UnitQuat::from_euler_angles(0., -FRAC_PI_2, 0.),
			},
			Rotations {
				desc: "Yaw left 90 degrees",
				look_towards: look_towards(&-right_vec(), &up_vec()),
				look_at: UnitQuat::look_at_rh(&Vector3::x_axis(), &Vector3::y_axis()),
				face_towards: UnitQuat::face_towards(
					&Vector3::x_axis(),
					&Vector3::y_axis(),
				),
				axis_angle: UnitQuat::from_axis_angle(&Vector3::y_axis(), FRAC_PI_2),
				euler: UnitQuat::from_euler_angles(0., FRAC_PI_2, 0.),
			},
			Rotations {
				desc: "Roll clockwise 90 degrees",
				look_towards: look_towards(&forward_vec(), &right_vec()),
				look_at: UnitQuat::look_at_rh(&-Vector3::z_axis(), &-Vector3::x_axis()),
				face_towards: UnitQuat::face_towards(
					&Vector3::z_axis(),
					&Vector3::x_axis(),
				),
				axis_angle: UnitQuat::from_axis_angle(&Vector3::z_axis(), -FRAC_PI_2),
				euler: UnitQuat::from_euler_angles(0., 0., -FRAC_PI_2),
			},
			Rotations {
				desc: "Roll counter-clockwise 90 degrees",
				look_towards: look_towards(&forward_vec(), &-right_vec()),
				look_at: UnitQuat::look_at_rh(&-Vector3::z_axis(), &Vector3::x_axis()),
				face_towards: UnitQuat::face_towards(
					&Vector3::z_axis(),
					&-Vector3::x_axis(),
				),
				axis_angle: UnitQuat::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2),
				euler: UnitQuat::from_euler_angles(0., 0., FRAC_PI_2),
			},
		];

		for r in rotations {
			// Check that all 3 coordinate axes are rotated to same direction
			println!("Checking rotation: {}", r.desc);
			for axis in [Vector3::x_axis(), Vector3::y_axis(), Vector3::z_axis()] {
				println!("Testing axis: {axis:?}");
				assert_relative_eq!(r.axis_angle * axis, r.look_at * axis);
				assert_relative_eq!(r.axis_angle * axis, r.euler * axis);
				assert_relative_eq!(r.axis_angle * axis, r.face_towards * axis);
				assert_relative_eq!(r.axis_angle * axis, r.look_towards * axis);
			}

			// Check that the angles between rotations are zero
			assert_relative_eq!(r.axis_angle.angle_to(&r.look_at), 0.0);
			assert_relative_eq!(r.axis_angle.angle_to(&r.euler), 0.0);
			assert_relative_eq!(r.axis_angle.angle_to(&r.face_towards), 0.0);
			assert_relative_eq!(r.axis_angle.angle_to(&r.look_towards), 0.0);
		}
	}
}
