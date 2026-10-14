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
use core::ops::DerefMut;

use control::DEFAULT_CONTROL_PERIOD_SECONDS;
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;

use stm32g0::stm32g030::interrupt;

mod app;
mod control;
mod dc_motor_driver_stm32g0;
mod indicator;

type FirmwareApp = app::App<
    dc_motor_driver_stm32g0::Led0,
    dc_motor_driver_stm32g0::Led1,
    dc_motor_driver_stm32g0::Uart2,
    dc_motor_driver_stm32g0::LocalClock,
    dc_motor_driver_stm32g0::DcPwm,
    dc_motor_driver_stm32g0::EncoderPeripheral,
>;

static G_APP: Mutex<RefCell<Option<FirmwareApp>>> = Mutex::new(RefCell::new(None));

// TODO: タイマ割り込みで定期処理[App]
#[interrupt]
fn TIM14() {
    dc_motor_driver_stm32g0::timer_interrupt_task();
    free(|cs| match G_APP.borrow(cs).borrow_mut().deref_mut() {
        None => (),
        Some(app) => {
            app.control();
        }
    });
}

// TODO: Dynamixel通信処理[App]
#[interrupt]
fn USART2() {
    let mut recieved_flag = false;
    // FIFOの最大サイズ+1までループでチェックする
    for _ in 0..=9 {
        let mut data: u8 = 0;
        let mut return_flag = false;
        free(|cs| {
            match dc_motor_driver_stm32g0::G_PERIPHERAL
                .borrow(cs)
                .borrow()
                .as_ref()
            {
                None => {
                    // 初期化がまだなのでreturnしてよい。
                    defmt::info!("Not initialized yet. Return");
                    return_flag = true;
                }
                Some(perip) => {
                    let uart = &perip.USART2;
                    // RXFNEで割り込みが入っていることの確認
                    if uart.isr.read().rxne().bit_is_clear() {
                        // RXFNE
                        // bufferが空になったらreturn
                        return_flag = true;
                    } else {
                        data = uart.rdr.read().rdr().bits() as u8;
                        // defmt::info!("get: 0x{:x}", data);
                        recieved_flag = true;
                    }
                }
            }
        });
        // free内部からいきなりreturnできないのでflagを使う
        if return_flag {
            if recieved_flag {
                // defmt::info!("recieved flag is true");
                break;
            } else {
                return;
                // TODO: ここあやしい
            }
        }
        free(|cs| match G_APP.borrow(cs).borrow_mut().deref_mut() {
            None => {
                // 初期化がまだなのでreturnしてよい。
                defmt::info!("Not initialized yet. Return");
                return_flag = true;
            }
            Some(app) => {
                app.enque_uart(data);
                // defmt::info!("enqueue");
            }
        });
        if return_flag {
            return;
        }
    }

    free(|cs| match G_APP.borrow(cs).borrow_mut().deref_mut() {
        None => (),
        Some(app) => {
            app.parse_uart_task();
            // defmt::info!("parse uart task finished.");
        }
    });

    // // ここまで来ているということは正常に空にできていない
    // defmt::error!("Something went wrong when clearing fifo.");
    // unreachable!();
    // free(|cs| match dc_motor_driver_stm32g0::G_PERIPHERAL.borrow(cs).borrow().as_ref() {
    //     None => (),
    //     Some(perip) => {
    //         //    The RXFNE flag can also be cleared by writing 1 to the RXFRQ in the USART_RQR register
    //         let uart = &perip.USART1;
    //         uart.rqr.write(|w| w.rxfrq().set_bit());
    //     }
    // });
}

#[entry]
fn main() -> ! {
    use stm32g0::stm32g030;

    defmt::info!("Hello from STM32G0!");
    // ペリフェラルの入り口となるオブジェクトを取得する。
    let perip = stm32g030::Peripherals::take().unwrap();
    let mut core_perip = stm32g030::CorePeripherals::take().unwrap();

    dc_motor_driver_stm32g0::clock_init(&perip, &mut core_perip);
    // dc_motor_driver_stm32g0::exti_init(&perip, &mut core_perip);

    // init g peripheral
    dc_motor_driver_stm32g0::init_g_peripheral(perip);

    free(|cs| {
        // G_APPの初期化が終わるまで割り込み処理が実行されないようにクリティカルセクションを取る

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

        let uart_rs485 = dc_motor_driver_stm32g0::Uart2::new();
        uart_rs485.init();
        let clock: dc_motor_driver_stm32g0::LocalClock = dc_motor_driver_stm32g0::LocalClock::new();
        clock.init();
        let app = app::App::new(
            led0,
            led1,
            uart_rs485,
            clock,
            md,
            enc,
            DEFAULT_CONTROL_PERIOD_SECONDS,
        );
        app.init();
        G_APP.borrow(cs).replace(Some(app));
    });

    // Enable reception only after G_APP is ready for the USART2 interrupt handler.
    dc_motor_driver_stm32g0::enable_usart2_interrupt(&mut core_perip);

    dc_motor_driver_stm32g0::start_control_timer();

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
                    // LED toggle
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
