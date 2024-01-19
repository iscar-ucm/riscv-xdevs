#![no_std]

use hifive1::hal::gpio::*;

pub type RedLed = gpio0::Pin0<Output<Regular<NoInvert>>>;
pub type BlueLed = gpio0::Pin1<Output<Regular<NoInvert>>>;
pub type GreenLed = gpio0::Pin2<Output<Regular<NoInvert>>>;

#[macro_export]
macro_rules! println {
    ($($arg:tt)*) => {
        #[cfg(feature = "qemu")]
        semihosting::println!($($arg)*);
        #[cfg(not(feature = "qemu"))]
        hifive1::sprintln!($($arg)*);
    };
}

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {
        #[cfg(feature = "qemu")]
        semihosting::print!($($arg)*);
        #[cfg(not(feature = "qemu"))]
        hifive1::sprint!($($arg)*);
    };
}

#[inline]
pub fn secf64_to_ticku64(t: f64) -> u64 {
    t as u64 * hifive1::hal::e310x::CLINT::freq() as u64
}

#[inline]
pub fn ticku64_to_secf64(t: u64) -> f64 {
    t as f64 / hifive1::hal::e310x::CLINT::freq() as f64
}

#[inline]
#[allow(unused_variables)]
pub fn exit(code: i32) -> ! {
    match () {
        #[cfg(feature = "qemu")]
        () => semihosting::process::exit(code),
        #[cfg(not(feature = "qemu"))]
        () => loop {
            unsafe { riscv::asm::wfi() };
        },
    }
}

pub mod generator {

    pub struct GeneratorState {
        sigma: f64,
        period: f64,
        count: usize,
    }

    impl GeneratorState {
        pub fn new(period: f64) -> Self {
            Self {
                sigma: 0.0,
                period,
                count: 0,
            }
        }
    }

    xdevs::component!(
        ident = Generator,
        input = {
            in_stop<bool>,
        },
        output = {
            out_job<usize>,
        },
        state = GeneratorState,
    );

    impl xdevs::Atomic for Generator {
        fn delta_int(state: &mut Self::State) {
            state.count += 1;
            state.sigma = state.period;
        }

        fn lambda(state: &Self::State, output: &mut Self::Output) {
            println!("[G] sending job {}", state.count);
            output.out_job.add_value(state.count).unwrap();
        }

        fn ta(state: &Self::State) -> f64 {
            state.sigma
        }

        fn delta_ext(state: &mut Self::State, e: f64, x: &Self::Input) {
            state.sigma -= e;
            if let Some(&stop) = x.in_stop.get_values().last() {
                println!("[G] received stop: {}", stop);
                if stop {
                    state.sigma = f64::INFINITY;
                }
            }
        }
    }
}

pub mod processor {
    use super::RedLed;
    use hifive1::hal::prelude::*;

    pub struct ProcessorState {
        sigma: f64,
        time: f64,
        job: Option<usize>,
        redled: RedLed,
    }

    impl ProcessorState {
        pub fn new(time: f64, redled: RedLed) -> Self {
            Self {
                sigma: 0.0,
                time,
                job: None,
                redled,
            }
        }
    }

    xdevs::component!(
        ident = Processor,
        input = {
            in_job<usize, 1>
        },
        output = {
            out_job<usize>
        },
        state = ProcessorState,
    );

    impl xdevs::Atomic for Processor {
        fn stop(state: &mut Self::State) {
            // make sure the red led is off
            state.redled.set_low().unwrap();
        }

        fn delta_int(state: &mut Self::State) {
            state.sigma = f64::INFINITY;
            if let Some(job) = state.job {
                println!("[P] processed job {}", job);
                state.job = None;
                state.redled.set_low().unwrap();
            }
        }

        fn lambda(state: &Self::State, output: &mut Self::Output) {
            if let Some(job) = state.job {
                output.out_job.add_value(job).unwrap();
            }
        }

        fn ta(state: &Self::State) -> f64 {
            state.sigma
        }

        fn delta_ext(state: &mut Self::State, e: f64, x: &Self::Input) {
            state.sigma -= e;
            if let Some(&job) = x.in_job.get_values().last() {
                print!("[P] received job {}", job);
                if state.job.is_none() {
                    println!(" (idle)");
                    state.job = Some(job);
                    state.sigma = state.time;
                    state.redled.set_high().unwrap();
                } else {
                    println!(" (busy)");
                }
            }
        }
    }
}

pub mod transducer {

    pub struct TransducerState {
        sigma: f64,
        clock: f64,
        n_gen: usize,
        n_proc: usize,
    }

    impl TransducerState {
        pub fn new(obs_time: f64) -> Self {
            Self {
                sigma: obs_time,
                clock: 0.0,
                n_gen: 0,
                n_proc: 0,
            }
        }
    }

    xdevs::component!(
        ident = Transducer,
        input = {
            in_gen<usize, 2>,
            in_proc<usize, 1>,
        },
        output = {
            out_stop<bool>
        },
        state = TransducerState,
    );

    impl xdevs::Atomic for Transducer {
        fn delta_int(state: &mut Self::State) {
            state.clock += state.sigma;
            let (acceptance, throughput) = if state.n_proc > 0 {
                (
                    state.n_proc as f64 / state.n_gen as f64,
                    state.n_proc as f64 / state.clock,
                )
            } else {
                (0.0, 0.0)
            };
            println!(
                "[T] acceptance: {:.2}, throughput: {:.2}",
                acceptance, throughput
            );
            state.sigma = f64::INFINITY;
        }

        fn lambda(_state: &Self::State, output: &mut Self::Output) {
            output.out_stop.add_value(true).unwrap();
        }

        fn ta(state: &Self::State) -> f64 {
            state.sigma
        }

        fn delta_ext(state: &mut Self::State, e: f64, x: &Self::Input) {
            state.sigma -= e;
            state.clock += e;
            state.n_gen += x.in_gen.get_values().len();
            state.n_proc += x.in_proc.get_values().len();
        }
    }
}

xdevs::component!(
    ident = PT,
    input = {
        in_job<usize, 1>,
    },
    output = {
        out_stop<bool, 1>,
    },
    components = {
        processor: processor::Processor,
        transducer: transducer::Transducer,
    },
    couplings = {
        in_job -> processor.in_job,
        in_job -> transducer.in_gen,
        processor.out_job -> transducer.in_proc,
        transducer.out_stop -> out_stop,
    }
);

xdevs::component!(
    ident = GPT,
    components = {
        generator: generator::Generator,
        processor: processor::Processor,
        transducer: transducer::Transducer,
    },
    couplings = {
        generator.out_job -> processor.in_job,
        processor.out_job -> transducer.in_proc,
        generator.out_job -> transducer.in_gen,
        transducer.out_stop -> generator.in_stop,
    }
);

xdevs::component!(
    ident = EF,
    input = {
        in_processor<usize, 1>,
    },
    output = {
        out_generator<usize, 1>,
    },
    components = {
        generator: generator::Generator,
        transducer: transducer::Transducer,
    },
    couplings = {
        in_processor -> transducer.in_proc,
        generator.out_job -> transducer.in_gen,
        transducer.out_stop -> generator.in_stop,
        generator.out_job -> out_generator,
    }
);

xdevs::component!(
    ident = EFP,
    components = {
        ef: EF,
        processor: processor::Processor,
    },
    couplings = {
        ef.out_generator -> processor.in_job,
        processor.out_job -> ef.in_processor,
    }
);
