//! An implementation of SlimeVR firmware, written in Rust.

#![no_std]
#![no_main]
// Needed for embassy macros
#![feature(type_alias_impl_trait)]
// Needed to use `alloc` + `no_std`
#![feature(alloc_error_handler)]
#![deny(unsafe_op_in_unsafe_fn)]

mod aliases;
mod globals;
mod imu;
mod networking;
mod peripherals;
mod utils;

#[cfg(bbq)]
mod bbq_logger;

use defmt::debug;
use embassy_executor::Executor;

use embedded_hal::blocking::delay::DelayMs;

use imu::imu_task;
use networking::{network_task, protocol_task, Packets};
use static_cell::StaticCell;

#[cfg(cortex_m)]
use cortex_m_rt::entry;
#[cfg(riscv)]
use riscv_rt::entry;
#[cfg(esp_xtensa)]
use xtensa_lx_rt::entry;

#[entry]
fn main() -> ! {
	#[cfg(bbq)]
	let bbq = defmt_bbq::init().unwrap();

	self::globals::setup();
	debug!("Booted");
	defmt::trace!("Trace");

	let p = self::peripherals::ඞ::get_peripherals();
	#[allow(unused)]
	let (bbq_peripheral, mut p) = p.bbq_peripheral();

	p.delay.delay_ms(500u32);
	debug!("Initialized peripherals");

	static PACKETS: StaticCell<Packets> = StaticCell::new();
	let packets: &'static Packets = PACKETS.init(Packets::new());

	static EXECUTOR: StaticCell<Executor> = StaticCell::new();
	EXECUTOR.init(Executor::new()).run(move |s| {
		s.spawn(network_task(packets)).unwrap();
		s.spawn(protocol_task(packets)).unwrap();
		s.spawn(imu_task(packets, p.i2c, p.delay)).unwrap();
		#[cfg(bbq)]
		s.spawn(logger_task(bbq, bbq_peripheral)).unwrap();
	});
}

#[cfg(bbq)]
#[task]
async fn logger_task(
	bbq: defmt_bbq::DefmtConsumer,
	logger_peripheral: crate::aliases::ඞ::BbqPeripheralConcrete<'static>,
) {
	crate::bbq_logger::ඞ::logger_task(bbq, logger_peripheral).await;
}
