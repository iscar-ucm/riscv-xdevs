#![no_std]
#![no_main]

use hifive1::hal::e310x::CLINT;
use hifive1::hal::prelude::*;
use hifive1::hal::DeviceResources;
use riscv_rt::entry;
use riscv_xdevs::*;

#[cfg(not(feature = "qemu"))]
extern crate panic_halt;

/// Sleep closure for RT simulation on SiFive E310x boards.
/// This is based on busy loops, and interrupts are not used.
/// While this approach reduces the jitter, it incurs a high CPU load.
pub fn wait_poll<T: xdevs::aux::Bag>() -> impl FnMut(f64, &mut T) -> f64 {
    // reset CLINT's mtime register (this is done only once)
    let mtime = CLINT::mtimer().mtime;
    mtime.write(0);
    // closure for RT simulation (this is called in every simulation step)
    move |t_next, _| -> f64 {
        // wait until next tick in busy loop
        let next_tick = t_next as u64 * CLINT::freq() as u64;
        while mtime.read() < next_tick {}
        // check jitter
        let jitter = (mtime.read() - next_tick) * 1_000_000 / CLINT::freq() as u64;
        println!("jitter: {} us", jitter);
        // return next simulation time
        t_next
    }
}

#[entry]
fn main() -> ! {
    let dr = DeviceResources::take().unwrap();
    let p = dr.peripherals;

    // Configure clocks
    let _clocks = hifive1::clock::configure(p.PRCI, p.AONCLK, 320.mhz().into());

    let gpio = dr.pins;

    // Configure red LED pin
    let redled = gpio.pin0.into_output();
    let mut greenled = gpio.pin1.into_output();

    // Configure stdout for debugging (only on real hardware)
    #[cfg(not(feature = "qemu"))]
    hifive1::stdout::configure(
        p.UART0,
        hifive1::pin!(gpio, uart0_tx),
        hifive1::pin!(gpio, uart0_rx),
        115_200.bps(),
        _clocks,
    );

    let period = 1.;
    let proc_time = 1.1;
    let obs_time = 10.;
    let t_sim = 15.;

    let generator = generator::Generator::new(generator::GeneratorState::new(period));
    let processor = processor::Processor::new(processor::ProcessorState::new(proc_time, redled));
    let transducer = transducer::Transducer::new(transducer::TransducerState::new(obs_time));

    let ef = EF::new(generator, transducer);
    let efp = EFP::new(ef, processor);

    let mut simulator = xdevs::simulator::Simulator::new(efp);

    let wait = wait_poll();

    println!("Simulating for {} seconds", t_sim);

    simulator.simulate_rt(0.0, t_sim, wait, |_| {});

    println!("Simulation finished");

    greenled.set_high().unwrap();

    exit(0);
}
