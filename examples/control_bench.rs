//! Explicit hardware check. `cargo run -r --example control_bench` MOVES THE MOTOR.
#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use defmt_rtt as _;
use dynamixel_f_rs::control_table::BitsW;
use indicator::Indicator;
use panic_halt as _;
use stm32g0::stm32g030::interrupt;

#[path = "../src/app.rs"]
mod app;
#[path = "../src/control.rs"]
mod control;
#[path = "../src/dc_motor_driver_stm32g0.rs"]
mod dc_motor_driver_stm32g0;
#[path = "../src/indicator.rs"]
mod indicator;
use dc_motor_driver_stm32g0 as hw;

type FirmwareApp =
    app::App<hw::Led0, hw::Led1, hw::Uart2, hw::LocalClock, hw::DcPwm, hw::EncoderPeripheral>;
// duration (ms), operating mode, goal: encoder counts or counts/second.
const IDENTIFY: bool = option_env!("CONTROL_IDENTIFY").is_some();
const VELOCITY_STUDY: bool = option_env!("CONTROL_VELOCITY_STUDY").is_some();
const VELOCITY_VERIFY: bool = option_env!("CONTROL_VELOCITY_VERIFY").is_some();
const STEPS: &[(u32, u8, i32)] = if IDENTIFY {
    &[
        (500, 16, 0),
        (500, 16, 300),
        (300, 16, 0),
        (500, 16, -300),
        (300, 16, 0),
        (500, 16, 360),
        (300, 16, 0),
        (500, 16, -360),
        (300, 16, 0),
        (500, 16, 400),
        (300, 16, 0),
        (500, 16, -400),
        (300, 16, 0),
        (500, 16, 443),
        (300, 16, 0),
        (500, 16, -443),
        (300, 16, 0),
    ]
} else if VELOCITY_VERIFY {
    &[
        (500, 1, 0),
        (4000, 1, 126),
        (1000, 1, 0),
        (4000, 1, -126),
        (1000, 1, 0),
        (6000, 1, 63),
        (1000, 1, 0),
        (6000, 1, -63),
        (1000, 1, 0),
        (6000, 1, 32),
        (1000, 1, 0),
        (6000, 1, -32),
        (1000, 1, 0),
        (3000, 1, 252),
        (3000, 1, -252),
        (1000, 1, 0),
        (2000, 1, 630),
        (2000, 1, -630),
        (1000, 1, 0),
    ]
} else if VELOCITY_STUDY {
    &[
        (500, 1, 0),
        (3000, 1, 252),
        (1000, 1, 0),
        (3000, 1, -252),
        (1000, 1, 0),
        (3000, 1, 126),
        (1000, 1, 0),
        (3000, 1, -126),
        (1000, 1, 0),
        (3000, 1, 63),
        (1000, 1, 0),
        (3000, 1, -63),
        (1000, 1, 0),
        (2000, 1, 504),
        (2000, 1, -504),
        (1000, 1, 0),
    ]
} else if option_env!("CONTROL_POSITION_VERIFY").is_some() {
    &[
        (500, 3, 0),
        (10000, 3, 1),
        (10000, 3, 2),
        (10000, 3, 3),
        (8000, 3, 103),
        (8000, 3, 309),
        (8000, 3, 206),
        (8000, 3, 103),
        (8000, 3, 0),
    ]
} else if option_env!("CONTROL_POSITION_STUDY").is_some() {
    &[
        (500, 3, 0),
        (8000, 3, 103),
        (8000, 3, 206),
        (8000, 3, 103),
        (8000, 3, 0),
        (10000, 3, 1),
        (10000, 3, 2),
        (10000, 3, 1),
        (10000, 3, 0),
    ]
} else {
    &[
        (500, 3, 0),
        (2500, 3, 103),
        (2500, 3, 206),
        (2500, 3, 0),
        (1800, 1, 252),
        (800, 1, 0),
        (1800, 1, -252),
        (800, 1, 0),
        (1800, 1, 126),
        (800, 1, 0),
        (1800, 1, -126),
        (800, 1, 0),
    ]
};
struct Bench {
    app: FirmwareApp,
    tick: u32,
    elapsed: u32,
    step: usize,
    done: Option<bool>,
}
static BENCH: Mutex<RefCell<Option<Bench>>> = Mutex::new(RefCell::new(None));

#[interrupt]
fn TIM14() {
    hw::timer_interrupt_task();
    free(|cs| {
        let mut state = BENCH.borrow(cs).borrow_mut();
        let bench = match state.as_mut() {
            Some(bench) => bench,
            None => return,
        };
        if bench.done.is_some() {
            return;
        }
        if bench.elapsed == STEPS[bench.step].0 {
            bench.step += 1;
            bench.elapsed = 0;
        }
        if bench.step == STEPS.len() {
            bench
                .app
                .control_table()
                .modify(|_, w| w.torque_enable().bits(0));
            bench.app.control();
            bench.done = Some(true);
            return;
        }
        let (_, mode, goal) = STEPS[bench.step];
        if bench.elapsed == 0 {
            bench.app.control_table().modify(|_, w| {
                w.operating_mode()
                    .bits(mode)
                    .torque_enable()
                    .bits(if bench.step == 0 { 0 } else { 1 })
                    .goal_position()
                    .bits(if mode == 3 { goal } else { 0 })
                    .goal_velocity()
                    .bits(if mode == 1 { goal } else { 0 })
                    .goal_pwm()
                    .bits(if mode == 16 {
                        goal as i16
                    } else {
                        control::PWM_RAW_MAX
                    })
            });
            defmt::info!("STEP {} mode={} goal={}", bench.step, mode, goal);
        }
        bench.app.control();
        let r = bench.app.control_table().read();
        let position = r.present_position();
        let velocity = r.present_velocity();
        let position_guard = if VELOCITY_STUDY || VELOCITY_VERIFY {
            3300
        } else {
            825
        };
        if position.unsigned_abs() > position_guard
            || velocity.unsigned_abs() > 1300
            || (bench.step == 1
                && bench.elapsed == 800
                && mode == 3
                && goal >= 20
                && position < goal.min(20))
        {
            bench
                .app
                .control_table()
                .modify(|_, w| w.torque_enable().bits(0));
            bench.app.control();
            bench.done = Some(false);
            defmt::error!("GUARD p={} v={}", position, velocity);
            return;
        }
        if bench.tick % 10 == 0 {
            let (raw, clock) = {
                let peripheral = hw::G_PERIPHERAL.borrow(cs).borrow();
                let p = peripheral.as_ref().unwrap();
                (
                    p.TIM3.cnt.read().cnt_l().bits(),
                    p.TIM16.cnt.read().cnt().bits(),
                )
            };
            defmt::info!(
                "DATA {},{},{},{},{},{},{},{}",
                bench.tick,
                bench.step,
                goal,
                position,
                velocity,
                r.present_pwm(),
                raw,
                clock
            );
        }
        bench.tick += 1;
        bench.elapsed += 1;
    });
}

#[entry]
fn main() -> ! {
    let perip = stm32g0::stm32g030::Peripherals::take().unwrap();
    let mut core = stm32g0::stm32g030::CorePeripherals::take().unwrap();
    hw::clock_init(&perip, &mut core);
    hw::init_g_peripheral(perip);
    free(|cs| {
        let led0 = hw::Led0::new();
        led0.init();
        led0.off();
        let led1 = hw::Led1::new();
        led1.init();
        led1.off();
        let md = hw::DcPwm::new();
        md.init();
        let enc = hw::EncoderPeripheral::new();
        enc.init();
        let uart = hw::Uart2::new();
        uart.init();
        let clock = hw::LocalClock::new();
        clock.init();
        let app = app::App::new(
            led0,
            led1,
            uart,
            clock,
            md,
            enc,
            control::DEFAULT_CONTROL_PERIOD_SECONDS,
        );
        app.init();
        let r = app.control_table().read();
        defmt::info!(
            "DEFAULT pos={}/{}/{} vel={}/{} pwm={} native-counts cpr=825 pos-fc={} vel-fc={}",
            r.position_pgain(),
            r.position_igain(),
            r.position_dgain(),
            r.velocity_pgain(),
            r.velocity_igain(),
            r.pwm_limit(),
            app::POSITION_D_FILTER_CUTOFF_HZ as u16,
            app::VELOCITY_FILTER_CUTOFF_HZ as u16
        );
        BENCH.borrow(cs).replace(Some(Bench {
            app,
            tick: 0,
            elapsed: 0,
            step: 0,
            done: None,
        }));
    });
    hw::start_control_timer();
    loop {
        let done = free(|cs| BENCH.borrow(cs).borrow().as_ref().and_then(|b| b.done));
        if let Some(passed) = done {
            defmt::info!("DONE completed={}", passed);
            cortex_m_semihosting::debug::exit(if passed {
                cortex_m_semihosting::debug::EXIT_SUCCESS
            } else {
                cortex_m_semihosting::debug::EXIT_FAILURE
            });
            loop {
                cortex_m::asm::wfi();
            }
        }
        // Keep the debug bus accessible while probe-rs drains RTT.
        cortex_m::asm::nop();
    }
}
