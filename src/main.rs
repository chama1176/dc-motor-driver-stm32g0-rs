#![no_std]
#![no_main]

use indicator::Indicator;
// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use defmt_rtt as _;

use core::cell::RefCell;
use core::fmt::Write;
use core::ops::DerefMut;

use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;

use stm32g0::stm32g030::interrupt;
use stm32g0::stm32g030::Interrupt::EXTI0_1;
use stm32g0::stm32g030::Interrupt::TIM14;

use crate::dc_motor_driver::DcMotorDriver;

mod app;
mod indicator;
mod dc_motor_driver_stm32g0;
mod dc_motor_driver;

static G_APP: Mutex<
    RefCell<
        Option<
            app::App<
                dc_motor_driver_stm32g0::Led0,
                dc_motor_driver_stm32g0::Led1,
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));

// 4Mbps = 0.25us = 250ns
// 0.25 x 8bit(1Byte) x 4? = 8us?


#[interrupt]
fn TIM14() {
    dc_motor_driver_stm32g0::timer_interrupt_task();
}

#[entry]
fn main() -> ! {
    use stm32g0::stm32g030;

    defmt::info!("Hello from STM32G0!");
    // stm32f401モジュールより、ペリフェラルの入り口となるオブジェクトを取得する。
    let perip = stm32g030::Peripherals::take().unwrap();
    let mut core_perip = stm32g030::CorePeripherals::take().unwrap();

    dc_motor_driver_stm32g0::clock_init(&perip, &mut core_perip);
    // dc_motor_driver_stm32g0::exti_init(&perip, &mut core_perip);

    // init g peripheral
    dc_motor_driver_stm32g0::init_g_peripheral(perip);

    let led0 = dc_motor_driver_stm32g0::Led0::new();
    led0.init();
    led0.off();
    let led1 = dc_motor_driver_stm32g0::Led1::new();
    led1.init();
    led1.off();
    let md = dc_motor_driver_stm32g0::DcPwm::new();
    md.init();
    let enc = dc_motor_driver_stm32g0::EncoderPeripheral::new();
    enc.init();

    md.set_pwm(1.0, 0.5);
    let app = app::App::new(led0, led1);
    free(|cs| G_APP.borrow(cs).replace(Some(app)));

    let mut t = 0;
    free(|cs| {
        match dc_motor_driver_stm32g0::G_PERIPHERAL
            .borrow(cs)
            .borrow()
            .as_ref()
        {
            None => (),
            Some(perip) => {
                t = perip.TIM16.cnt.read().cnt().bits();
                // t = perip.TIM14.cnt.read().cnt().bits();
            }
        }
    });
    let mut prev = t;

    loop {
        free(|cs| {
            match dc_motor_driver_stm32g0::G_PERIPHERAL
                .borrow(cs)
                .borrow()
                .as_ref()
            {
                None => (),
                Some(perip) => {
                    t = perip.TIM16.cnt.read().cnt().bits();
                    // t = perip.TIM14.cnt.read().cnt().bits();
                }
            }
        });


        if t.wrapping_sub(prev) > 500 {
            free(|cs| match G_APP.borrow(cs).borrow_mut().deref_mut() {
                None => (),
                Some(app) => {
                    app.periodic_task();
                }
            });

            free(|cs| {
                match dc_motor_driver_stm32g0::G_PERIPHERAL
                    .borrow(cs)
                    .borrow()
                    .as_ref()
                {
                    None => (),
                    Some(perip) => {
                        let cnt_l = perip.TIM3.cnt.read().cnt_l().bits();
                        let cnt_h = perip.TIM3.cnt.read().cnt_h().bits();
                        defmt::info!("cnt_l: {}, cnt_h: {}", cnt_l, cnt_h);
                    }
                }
            });


            prev = t;
        }
    }
}
