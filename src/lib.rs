#![no_std]

#[no_mangle]
fn fmin(a: f64, b: f64) -> f64 {
    if a < b {
        a
    } else {
        b
    }
}

#[no_mangle]
fn fmax(a: f64, b: f64) -> f64 {
    if a > b {
        a
    } else {
        b
    }
}

use hifive1::{sprint, sprintln};

pub mod generator {
    use super::sprintln;

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
            sprintln!("[G] sending job {}", state.count);
            output.out_job.add_value(state.count).unwrap();
        }

        fn ta(state: &Self::State) -> f64 {
            state.sigma
        }

        fn delta_ext(state: &mut Self::State, e: f64, x: &Self::Input) {
            state.sigma -= e;
            if let Some(&stop) = x.in_stop.get_values().last() {
                sprintln!("[G] received stop: {}", stop);
                if stop {
                    state.sigma = f64::INFINITY;
                }
            }
        }
    }
}

pub mod processor {
    use super::{sprint, sprintln};

    pub struct ProcessorState {
        sigma: f64,
        time: f64,
        job: Option<usize>,
    }

    impl ProcessorState {
        pub fn new(time: f64) -> Self {
            Self {
                sigma: 0.0,
                time,
                job: None,
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
        fn delta_int(state: &mut Self::State) {
            state.sigma = f64::INFINITY;
            if let Some(job) = state.job {
                sprintln!("[P] processed job {}", job);
                state.job = None;
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
                sprint!("[P] received job {}", job);
                if state.job.is_none() {
                    sprintln!(" (idle)");
                    state.job = Some(job);
                    state.sigma = state.time;
                } else {
                    sprintln!(" (busy)");
                }
            }
        }
    }
}

pub mod transducer {
    use super::sprintln;

    pub struct TransducerState {
        sigma: f64,
        clock: f64,
        n_generated: usize,
        n_processed: usize,
    }

    impl TransducerState {
        pub fn new(obs_time: f64) -> Self {
            Self {
                sigma: obs_time,
                clock: 0.0,
                n_generated: 0,
                n_processed: 0,
            }
        }
    }

    xdevs::component!(
        ident = Transducer,
        input = {
            in_generator<usize, 1>,
            in_processor<usize, 1>,
        },
        output = {
            out_stop<bool>
        },
        state = TransducerState,
    );

    impl xdevs::Atomic for Transducer {
        fn delta_int(state: &mut Self::State) {
            state.clock += state.sigma;
            let (acceptance, throughput) = if state.n_processed > 0 {
                (
                    state.n_processed as f64 / state.n_generated as f64,
                    state.n_processed as f64 / state.clock,
                )
            } else {
                (0.0, 0.0)
            };
            sprintln!(
                "[T] acceptance: {:.2}, throughput: {:.2}",
                acceptance,
                throughput
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
            state.n_generated += x.in_generator.get_values().len();
            state.n_processed += x.in_processor.get_values().len();
        }
    }
}

xdevs::component!(
    ident = GPT,
    components = {
        generator: generator::Generator,
        processor: processor::Processor,
        transducer: transducer::Transducer,
    },
    couplings = {
        generator.out_job -> processor.in_job,
        processor.out_job -> transducer.in_processor,
        generator.out_job -> transducer.in_generator,
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
        in_processor -> transducer.in_processor,
        generator.out_job -> transducer.in_generator,
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
