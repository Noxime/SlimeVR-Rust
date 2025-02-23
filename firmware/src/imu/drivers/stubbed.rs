use crate::imu::{FusedImu, Quat};

use defmt::debug;
use embedded_hal::blocking::delay::DelayMs;
use firmware_protocol::ImuType;

/// Fakes an IMU for easier testing.
struct FakeImu;

impl FusedImu for FakeImu {
	type Error = ();

	const IMU_TYPE: ImuType = ImuType::Unknown(0xFF);

	fn quat(&mut self) -> nb::Result<Quat, Self::Error> {
		Ok(Quat::identity())
	}
}

#[allow(dead_code)]
pub fn new_imu(
	_i2c: impl crate::aliases::I2c,
	_delay: &mut impl DelayMs<u32>,
) -> impl crate::imu::FusedImu {
	debug!("Created FakeImu");
	FakeImu
}
