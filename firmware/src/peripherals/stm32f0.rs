use super::Peripherals;
use crate::aliases::ඞ::DelayConcrete;
use crate::aliases::ඞ::I2cConcrete;
use crate::aliases::ඞ::UartConcrete;
use crate::aliases::ඞ::UsbDriverConcrete;

use defmt::debug;
use embassy_stm32::dma::NoDma;
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::interrupt;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{self, Uart};
use embassy_stm32::usb::Driver;

pub fn get_peripherals() -> Peripherals<
	I2cConcrete<'static>,
	DelayConcrete,
	UartConcrete<'static>,
	UsbDriverConcrete<'static>,
> {
	let p = embassy_stm32::init(Default::default());

	// IDK how this works, code is from here:
	// https://github.com/embassy-rs/embassy/blob/f109e73c6d7ef2ad93102b7c8223f5cef30ef36f/examples/nrf/src/bin/twim.rs
	let i2c = {
		let config = i2c::Config::default();
		let irq = interrupt::take!(I2C1);
		// Twim::new(p.TWISPI0, irq, p.P0_03, p.P0_04, config)
		I2c::new(
			p.I2C1,
			p.PB6,
			p.PB7,
			irq,
			NoDma,
			NoDma,
			Hertz::khz(400),
			config,
		)
	};
	debug!("Initialized i2c");

	let delay = embassy_time::Delay;
	debug!("Initialized delay");

	let usart = {
		let irq = interrupt::take!(USART1);
		let mut config = usart::Config::default();
		config.parity = usart::Parity::ParityNone;
		config.baudrate = 115200;
		Uart::new(p.USART1, p.PA10, p.PA9, irq, NoDma, NoDma, config)
	};
	debug!("Initialized usart");

	let usb_driver = {
		let irq = interrupt::take!(USB);
		Driver::new(p.USB, irq, p.PA12, p.PA11)
	};
	debug!("Initialized usb_driver");

	let p = Peripherals::new();
	p.i2c(i2c).delay(delay).uart(usart).usb_driver(usb_driver)
}
