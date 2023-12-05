#![no_std]
#![no_main]

/*
* Basic blinking external LED using GPIO pin example.
* WARNING: requires a LED to be wired to physical PIN9 with at least
* a 320 Ohm resistor in series similar to
* https://create.arduino.cc/projecthub/rowan07/make-a-simple-led-circuit-ce8308
*/

extern crate panic_halt;

use hifive1::hal::delay::Sleep;
use hifive1::hal::prelude::*;
use hifive1::hal::DeviceResources;
use hifive1::pin;
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let dr = DeviceResources::take().unwrap();
    let p = dr.peripherals;
    let pins = dr.pins;

    // Configure clocks
    let clocks = hifive1::clock::configure(p.PRCI, p.AONCLK, 320.mhz().into());

    let mut redled = pins.pin0.into_output();
    let mut blueled = pins.pin1.into_output();
    let mut greenled = pins.pin2.into_output();

    // get the sleep struct
    let mut delay = hifive1::hal::delay::Delay::new();

    const PERIOD: u32 = 1000; // 1s

    let mut count = 0;
    loop {
        // make leds toggle depending on count
        if count % 3 == 0 {
            redled.set_high().unwrap();
            blueled.set_low().unwrap();
            greenled.set_low().unwrap();
        } else if count % 3 == 1 {
            redled.set_low().unwrap();
            blueled.set_low().unwrap();
            greenled.set_high().unwrap();
        } else {
            redled.set_low().unwrap();
            blueled.set_high().unwrap();
            greenled.set_low().unwrap();
        }
        count += 1;

        // sleep for 1s
        delay.delay_ms(PERIOD);
    }
}
