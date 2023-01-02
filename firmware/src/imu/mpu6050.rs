use super::{Imu, Quat};
use crate::aliases::I2c;
use crate::utils;

use dcmimu::DCMIMU;
use defmt::{debug, trace};
use embassy_futures::yield_now;
use embassy_time::Instant;
use embedded_hal::blocking::delay::DelayMs;
use firmware_protocol::ImuType;
use mpu9250::{
	self, AccelDataRate, Device, Dlpf, Error, GyroTempDataRate, I2cDevice,
	ImuMeasurements, Mpu9250, MpuConfig, Releasable, G,
};

pub struct Mpu6050<I: I2c> {
	last: Instant,
	mpu: Mpu9250<I2cDevice<I>, mpu9250::Imu>,
	dcm: DCMIMU,
}
impl<I: I2c> Mpu6050<I> {
	pub fn new(
		i2c: I,
		delay: &mut impl DelayMs<u8>,
	) -> Result<Self, mpu9250::Error<<I as I2c>::Error>> {
		// Roughly 100tps
		let dlpf = Dlpf::_2;

		let mut mpu = Mpu9250::imu(
			i2c,
			delay,
			MpuConfig::imu()
				.accel_data_rate(AccelDataRate::DlpfConf(dlpf))
				.gyro_temp_data_rate(GyroTempDataRate::DlpfConf(dlpf)),
		)
		.unwrap();

		let _ = mpu.calibrate_at_rest::<_, [f32; 3]>(delay);

		let dcm = DCMIMU::new();

		Ok(Self {
			last: Instant::now(),
			mpu,
			dcm,
		})
	}
}

impl<I: I2c> Imu for Mpu6050<I> {
	type Error = mpu9250::Error<<I as I2c>::Error>;

	const IMU_TYPE: ImuType = ImuType::Mpu6050;

	fn quat(&mut self) -> nb::Result<Quat, Self::Error> {
		let ImuMeasurements {
			accel: [ax, ay, az],
			gyro: [gx, gy, gz],
			temp,
		} = self.mpu.all::<[f32; 3]>().unwrap();

		let elapsed = self.last.elapsed();
		self.last += elapsed;

		let (euler, _biases) = self.dcm.update(
			(gx, gy, gz),
			(ax, ay, az),
			elapsed.as_micros() as f32 / 1_000_000.0,
		);

		Ok(Quat::from_euler_angles(euler.roll, euler.pitch, euler.yaw))
	}
}

pub fn new_imu(
	i2c: impl crate::aliases::I2c,
	delay: &mut impl DelayMs<u8>,
) -> impl crate::imu::Imu {
	Mpu6050::new(i2c, delay).unwrap()
}
