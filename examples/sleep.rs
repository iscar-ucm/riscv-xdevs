#![no_std]
#![no_main]

#[cfg(not(feature = "qemu"))]
extern crate panic_halt;

use hifive1::hal::e310x::CLINT;

use hifive1::hal::prelude::*;
use hifive1::hal::DeviceResources;
use riscv_rt::entry;
use riscv_xdevs::*;

/// Machine timer interrupt handler.
/// This function is called when the machine timer interrupt is triggered.
/// It fills the MTIMECMP0 register with the maximum value to disable the timer.
#[no_mangle]
#[allow(non_snake_case)]
fn MachineTimer() {
    CLINT::mtimecmp0().write(u64::MAX);
}

/// Closure for RT simulation on SiFive E310x boards.
pub fn wait_sleep<T: xdevs::aux::Bag>(
    t_start: f64,
    t_scale: f64,
    max_jitter_us: Option<u64>,
) -> impl FnMut(f64, &mut T) -> f64 {
    let mtimer = hifive1::hal::e310x::CLINT::mtimer();
    let (mtimecmp, mtime) = (mtimer.mtimecmp0, mtimer.mtime);
    mtime.write(0);

    move |t_next, _| -> f64 {
        // configure machine timer interrupt and sleep until next tick
        let next_tick = secf64_to_ticku64((t_next - t_start) * t_scale);
        while mtime.read() < next_tick {
            mtimecmp.write(next_tick);
            unsafe {
                CLINT::mtimer_enable();
                riscv::asm::wfi();
            }
        }
        CLINT::mtimer_disable(); // make sure interrupts are disabled after sleep

        // check jitter (if necessary)
        if let Some(max_jitter) = max_jitter_us {
            let jitter = (mtime.read() - next_tick) * 1_000_000 / CLINT::freq() as u64;
            println!("jitter: {} us", jitter);
            if jitter > max_jitter {
                panic!("jitter is too high");
            }
        }
        t_next
    }
}

#[entry]
fn main() -> ! {
    let dr = DeviceResources::take().unwrap();
    let p = dr.peripherals;
    let gpio = dr.pins;

    // Configure clocks
    let _clocks = hifive1::clock::configure(p.PRCI, p.AONCLK, 320.mhz().into());

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

    println!("Building model");

    let period = 1.;
    let proc_time = 1.1;
    let obs_time = 10.;
    let t_sim = 15.;
    let max_jitter_us = Some(7000);

    let generator = generator::Generator::new(generator::GeneratorState::new(period));
    let processor = processor::Processor::new(processor::ProcessorState::new(proc_time, redled));
    let transducer = transducer::Transducer::new(transducer::TransducerState::new(obs_time));

    let ef = EF::new(generator, transducer);
    let efp = EFP::new(ef, processor);

    let mut simulator = xdevs::simulator::Simulator::new(efp);

    let wait = wait_sleep(0.0, 1., max_jitter_us);

    println!("Enabling machine interrupts");
    unsafe { riscv::register::mstatus::set_mie() };

    println!("Simulating for {} time units", t_sim);
    simulator.simulate_rt(0.0, t_sim, wait, |_| {});

    println!("Simulation finished");

    greenled.set_high().unwrap();

    exit(0);
}
