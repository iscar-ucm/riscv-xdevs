#![no_std]
#![no_main]

extern crate panic_halt;

use hifive1::hal::prelude::*;
use hifive1::hal::DeviceResources;
use hifive1::sprintln;
use riscv_rt::entry;
use riscv_xdevs::*;

/// Closure for RT simulation on targets with `std`.
/// It sleeps until the next state transition.
///
pub fn sleep<T: xdevs::aux::Port>(t_start: f64, time_scale: f64) -> impl FnMut(f64, &mut T) -> f64 {
    fn secf64_to_usecu64(t: f64) -> u32 {
        (t * 1_000_000.0) as u32
    }

    let mut delay = hifive1::hal::delay::Delay::new();
    let mut last_vt = t_start;

    move |t_next, _| -> f64 {
        let e = secf64_to_usecu64((t_next - last_vt) * time_scale);
        delay.delay_us(e);
        last_vt = t_next;

        t_next
    }
}

#[entry]
fn main() -> ! {
    let dr = DeviceResources::take().unwrap();
    let p = dr.peripherals;

    // Configure clocks
    let clocks = hifive1::clock::configure(p.PRCI, p.AONCLK, 320.mhz().into());

    // Configure stdout for debugging
    let gpio = dr.pins;
    hifive1::stdout::configure(
        p.UART0,
        hifive1::pin!(gpio, uart0_tx),
        hifive1::pin!(gpio, uart0_rx),
        115_200.bps(),
        clocks,
    );

    let period = 1.;
    let proc_time = 1.1;
    let obs_time = 10.;

    let generator = generator::Generator::new(generator::GeneratorState::new(period));
    let processor = processor::Processor::new(processor::ProcessorState::new(proc_time));
    let transducer = transducer::Transducer::new(transducer::TransducerState::new(obs_time));

    let ef = EF::new(generator, transducer);
    let efp = EFP::new(ef, processor);

    let mut simulator = xdevs::simulator::Simulator::new(efp);
    let delay = sleep(0.0, 1.0);

    sprintln!("Simulating for {} time units", 100.0);

    // simulator.simulate_vt(0.0, 100.0);
    simulator.simulate_rt(0.0, 100.0, delay, |_| {});

    sprintln!("Simulation finished");

    loop {}
}
