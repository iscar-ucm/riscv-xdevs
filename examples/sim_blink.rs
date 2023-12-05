#![no_std]
#![no_main]

extern crate panic_halt;

use hifive1::hal::prelude::*;
use hifive1::hal::DeviceResources;
use hifive1::sprintln;
use riscv_rt::entry;
use riscv_xdevs::*;

/// Closure for RT simulation on SiFive E310x boards.
pub fn sleep<T: xdevs::aux::Port>(
    t_start: f64,
    time_scale: f64,
    max_jitter_us: Option<u64>,
) -> impl FnMut(f64, &mut T) -> f64 {
    #[cfg(not(feature = "qemu"))]
    const FREQ: u64 = 32_768;
    #[cfg(feature = "qemu")]
    const FREQ: u64 = 10_000_000;

    fn secf64_to_ticku64(t: f64) -> u64 {
        t as u64 * FREQ
    }

    let max_jitter_ticks = max_jitter_us.map(|j| j * FREQ / 1_000_000);

    let mtimer = hifive1::hal::e310x::CLINT::mtimer();
    let (_mtimecmp, mtime) = (mtimer.mtimecmp0, mtimer.mtime);

    mtime.write(0);

    move |t_next, _| -> f64 {
        let next_tick = secf64_to_ticku64((t_next - t_start) * time_scale);
        while mtime.read() < next_tick {}
        // check jitter
        if let Some(max_jitter) = max_jitter_ticks {
            let jitter = mtime.read() - next_tick;
            if jitter > max_jitter {
                let jitter_us = jitter * 1_000_000 / FREQ;
                sprintln!("too high jitter: {} us", jitter_us);
                panic!();
            }
        }

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
    let max_jitter_us = Some(1800);

    let generator = generator::Generator::new(generator::GeneratorState::new(period));
    let processor = processor::Processor::new(processor::ProcessorState::new(proc_time));
    let transducer = transducer::Transducer::new(transducer::TransducerState::new(obs_time));

    let ef = EF::new(generator, transducer);
    let efp = EFP::new(ef, processor);

    let mut simulator = xdevs::simulator::Simulator::new(efp);

    let delay = sleep(0.0, 1., max_jitter_us);

    sprintln!("Simulating for {} time units", 100.0);

    // simulator.simulate_vt(0.0, 100.0);
    simulator.simulate_rt(0.0, 100.0, delay, |_| {});

    sprintln!("Simulation finished");

    loop {}
}
