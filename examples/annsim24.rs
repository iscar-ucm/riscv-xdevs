#![no_std]
#![no_main]

#[cfg(not(feature = "qemu"))]
extern crate panic_halt;

use hifive1::hal::{
    e310x::{Interrupt, Priority, CLINT, GPIO0, PLIC},
    prelude::*,
    DeviceResources,
};

use riscv_rt::entry;
use riscv_xdevs::*;

use portable_atomic::{AtomicBool, Ordering::*};
use xdevs::aux::Bag;

/// atomic variable to communicate between [GPIO9] interrupt handler and input_handler function
static PRESSED: AtomicBool = AtomicBool::new(false);

/// GPIO9 interrupt handler.
/// It sets the atomic variable PRESSED to true and clears the interrupt pending flag.
#[no_mangle]
#[allow(non_snake_case)]
fn GPIO9() {
    unsafe { (*GPIO0::ptr()).fall_ip.write(|w| w.bits(1 << 9)) };
    PRESSED.store(true, Release);
}

/// Machine timer interrupt handler.
/// It fills CLINT's MTIMECMP0 register with the maximum value to clear the interrupt.
#[no_mangle]
#[allow(non_snake_case)]
fn MachineTimer() {
    CLINT::mtimecmp0().write(u64::MAX);
}

/// Closure for RT simulation on SiFive E310x boards.
pub fn wait_until() -> impl FnMut(f64, &mut PTInput) -> f64 {
    let mut count = 0;
    let mtimer = hifive1::hal::e310x::CLINT::mtimer();
    let (mtimecmp, mtime) = (mtimer.mtimecmp0, mtimer.mtime);
    mtime.write(0);
    let mut debounce = mtime.read();

    move |t_next, input| -> f64 {
        // translate next simulation time to next CLINT tick
        let next_tick = t_next as u64 * CLINT::freq() as u64;
        // wait for event (either button press or next CLINT tick)
        while mtime.read() < next_tick {
            // check if button was pressed and inject event and break if so
            if PRESSED
                .compare_exchange(true, false, Acquire, Relaxed)
                .is_ok()
            {
                let now = mtime.read();
                if now - debounce > CLINT::freq() as u64 {
                    debounce = now;
                    if input.in_job.add_value(count).is_ok() {
                        count += 1;
                        break;
                    } else {
                        println!("Error: input buffer full");
                    }
                }
            }
            // schedule CLINT's machine timer interrupt
            mtimecmp.write(next_tick);
            unsafe {
                CLINT::mtimer_enable();
                riscv::asm::wfi();
            }
        }
        CLINT::mtimer_disable();
        //compute current simulation time from current CLINT tick and return it
        let current_tick = mtime.read();
        if current_tick < next_tick {
            current_tick as f64 / CLINT::freq() as f64
        } else {
            t_next
        }
    }
}

pub fn propagate_output(mut blueled: BlueLed) -> impl FnMut(&PTOutput) {
    move |o| {
        if !o.is_empty() {
            blueled.set_high().unwrap();
        }
    }
}

#[entry]
fn main() -> ! {
    let dr = DeviceResources::take().unwrap();
    let p = dr.peripherals;
    let gpio = dr.pins;

    // Configure clocks
    let clocks = hifive1::clock::configure(p.PRCI, p.AONCLK, 320.mhz().into());

    // Make sure PLIC is reset and disabled
    PLIC::disable();
    let ctx = PLIC::ctx0();
    ctx.enables().disable_all::<Interrupt>();

    // Configure button pin for interrupt in falling edge
    gpio.pin9.into_pull_up_input();
    unsafe {
        let gpio_block = &*GPIO0::ptr();
        // Enable GPIO fall interrupts
        gpio_block.fall_ie.write(|w| w.bits(1 << 9));
        gpio_block.rise_ie.write(|w| w.bits(0x0));
        // Clear pending interrupts from previous states
        gpio_block.fall_ip.write(|w| w.bits(0xffffffff));
        gpio_block.rise_ip.write(|w| w.bits(0x0fffffff));
    }

    // Configure LED pins for output
    let redled = gpio.pin0.into_output();
    let blueled = gpio.pin1.into_output();
    let mut greenled = gpio.pin2.into_output();

    // Configure stdout for debugging (only on real hardware)
    #[cfg(not(feature = "qemu"))]
    hifive1::stdout::configure(
        p.UART0,
        hifive1::pin!(gpio, uart0_tx),
        hifive1::pin!(gpio, uart0_rx),
        115_200.bps(),
        clocks,
    );

    println!("Building model");
    let proc_time = 2.1; // time to process a job
    let obs_time = 10.; // time before transducer collects observations
    let t_sim = 15.; // simulation time

    let processor = processor::Processor::new(processor::ProcessorState::new(proc_time, redled));
    let transducer = transducer::Transducer::new(transducer::TransducerState::new(obs_time));
    let pt = PT::new(processor, transducer);
    let mut simulator = xdevs::simulator::Simulator::new(pt);

    println!("Enabling interrupts");
    unsafe {
        PLIC::priorities().set_priority(Interrupt::GPIO9, Priority::P2);
        ctx.threshold().set_threshold(Priority::P0);
        ctx.enables().enable(Interrupt::GPIO9);
        PLIC::enable();
        riscv::register::mstatus::set_mie();
    };

    println!("Simulating for {} time units", t_sim);
    simulator.simulate_rt(0.0, t_sim, wait_until(), propagate_output(blueled));
    println!("Simulation finished");

    greenled.set_high().unwrap();
    exit(0);
}
