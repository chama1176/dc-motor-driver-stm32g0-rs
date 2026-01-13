// interfaces
use crate::indicator::Indicator;
use crate::dc_motor_driver::DcMotorDriver;

//
use core::cell::RefCell;
use core::fmt::{self, Write};
use core::time::Duration;

use stm32g0::stm32g030::{Interrupt, tim14};
use stm32g0::stm32g030::Peripherals;
use stm32g0::stm32g030::NVIC;
use stm32g0::stm32g030::{exti, CorePeripherals};

use cortex_m::interrupt::{free, Mutex};

pub static G_PERIPHERAL: Mutex<RefCell<Option<stm32g0::stm32g030::Peripherals>>> =
    Mutex::new(RefCell::new(None));

pub fn init_g_peripheral(perip: Peripherals) {
    free(|cs| G_PERIPHERAL.borrow(cs).replace(Some(perip)));
}

pub fn clock_init(perip: &Peripherals, core_perip: &mut CorePeripherals) {
    perip.RCC.cr.modify(|_, w| w.hsebyp().set_bit());
    perip.RCC.cr.modify(|_, w| w.hseon().set_bit());
    while perip.RCC.cr.read().hserdy().bit_is_clear() {}

    // Disable the PLL
    perip.RCC.cr.modify(|_, w| w.pllon().clear_bit());
    // Wait until PLL is fully stopped
    while perip.RCC.cr.read().pllrdy().bit_is_set() {}
    perip
        .RCC
        .pllsyscfgr
        .modify(|_, w| unsafe { w.pllsrc().bits(0b11) }); // HSE
    perip
        .RCC
        .pllsyscfgr
        .modify(|_, w| unsafe { w.pllm().bits(0b011) }); // div4
    perip
        .RCC
        .pllsyscfgr
        .modify(|_, w| unsafe { w.plln().bits(16) }); // div16
    perip
        .RCC
        .pllsyscfgr
        .modify(|_, w| unsafe { w.pllr().bits(0b010) }); // div3

    perip.RCC.cr.modify(|_, w| w.pllon().set_bit());
    while perip.RCC.cr.read().pllrdy().bit_is_clear() {}
    perip.RCC.pllsyscfgr.modify(|_, w| w.pllren().set_bit());

    perip
        .FLASH
        .acr
        .modify(|_, w| unsafe { w.latency().bits(4) });
    while perip.FLASH.acr.read().latency().bits() != 4 {
        defmt::info!("latency bit: {}", perip.FLASH.acr.read().latency().bits());
    }

    perip.RCC.cfgr.modify(|_, w| unsafe { w.sw().bits(0b010) }); // PLL
                                                                 // perip.RCC.cfgr.modify(|_, w| w.sw().hse());
    defmt::debug!("sw bit: {}", perip.RCC.cfgr.read().sw().bits());
    while !(perip.RCC.cfgr.read().sw().bits() == 0b010) {}
    while !(perip.RCC.cfgr.read().sws().bits() == 0b010) {
        defmt::info!("sw bit: {}", perip.RCC.cfgr.read().sw().bits());
        defmt::info!("sws bit: {}", perip.RCC.cfgr.read().sws().bits());
    }
    // while !perip.RCC.cfgr.read().sws().is_hse() {}

    perip.RCC.apbenr2.modify(|_, w| w.tim16en().set_bit());
    perip.RCC.apbenr2.modify(|_, w| w.tim14en().set_bit());

    let tim16 = &perip.TIM16;
    tim16.psc.modify(|_, w| unsafe { w.bits(64_000 - 1) }); // 1kHz
                                                           // tim16.arr.modify(|_, w| unsafe { w.bits(1000 - 1) });    // 1kHz

    // tim16.dier.modify(|_, w| w.uie().set_bit());
    tim16.cr1.modify(|_, w| w.cen().set_bit());
    let tim14 = &perip.TIM14;
    tim14.psc.modify(|_, w| unsafe { w.bits(64 - 1) }); // 1us
    tim14.arr.modify(|_, w| unsafe { w.bits(10 - 1) }); // 10us
    tim14.cr1.modify(|_, w| w.urs().set_bit()); // UGによるSW割り込みをOFFにする

    // ARPE: ARRのPreloadは不要（どっちでもいい）なのでそのままにしておく
    // UDIS
    tim14.cr1.modify(|_, w| w.udis().clear_bit());
    tim14.dier.modify(|_, w| w.uie().set_bit());

    // 割り込み設定
    unsafe {
        core_perip.NVIC.set_priority(Interrupt::TIM16, 0);
        NVIC::unmask(Interrupt::TIM16);
        core_perip.NVIC.set_priority(Interrupt::TIM14, 2);
        NVIC::unmask(Interrupt::TIM14);
    }
}


pub fn timer_interrupt_task() {
    free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
        None => (),
        Some(perip) => {
            // 高速化のためにべた書きする

            let tim14 = &perip.TIM14;
            // Stop TIM14
            tim14.cr1.modify(|_, w| w.cen().clear_bit());
            // Clear tif
            tim14.sr.modify(|_, w| w.uif().clear_bit());
            // Toggle trigger out
            let gpioa = &perip.GPIOA;
            if gpioa.odr.read().odr6().is_low() {
                gpioa.bsrr.write(|w| w.bs6().set());
            } else {
                gpioa.bsrr.write(|w| w.br6().reset());
            }
        }
    });
}

pub struct EncoderPeripheral {}
impl<'a> EncoderPeripheral {
    pub fn new() -> Self {
        Self {}
    }
    pub fn init(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                // GPIOポートの電源投入(クロックの有効化)
                perip.RCC.iopenr.modify(|_, w| w.iopaen().set_bit());

                let gpioa = &perip.GPIOA;
                // PWM pin
                gpioa.moder.modify(|_, w| w.moder6().alternate());
                gpioa.moder.modify(|_, w| w.moder7().alternate());
                gpioa.afrl.modify(|_, w| w.afsel6().af1()); // TIM3 CH1
                gpioa.afrl.modify(|_, w| w.afsel7().af1()); // TIM3 CH2
                gpioa.ospeedr.modify(|_, w| w.ospeedr6().very_high_speed());
                gpioa.ospeedr.modify(|_, w| w.ospeedr7().very_high_speed());

                perip.RCC.apbenr1.modify(|_, w| w.tim3en().set_bit());

                // For PWM
                let tim = &perip.TIM3;
                // tim.psc.modify(|_, w| unsafe { w.bits(7 - 1) });
                // tim.arr.modify(|_, w| unsafe { w.bits(800 - 1) }); // 25kHz

                // 1. Select the proper TI1x source.
                tim.tisel.modify(|_, w| unsafe { w.ti1sel().bits(0b0000) });
                tim.tisel.modify(|_, w| unsafe { w.ti2sel().bits(0b0000) });

                // 2. Select the active input: TIMx_CCR1 must be linked to the TI1 input, 
                // the channel is configured in input and the TIMx_CCR1 register becomes read-only.
                tim.ccmr1_input().modify(|_, w| unsafe { w.cc1s().bits(0b01) });
                tim.ccmr1_input().modify(|_, w| unsafe { w.cc2s().bits(0b01) });

                // 3. Program the appropriate input filter duration in relation with the signal connected to the
                // timer. IC1F bits in the TIMx_CCMR1 register.
                // no filter

                // 4. Select the edge of the active transition on the TIx channel 
                // by writing the CC1P and CC1NP bit in the TIMx_CCER register.
                tim.ccer.modify(|_, w| w.cc1p().bit(false) );
                tim.ccer.modify(|_, w| w.cc1np().bit(false) );
                tim.ccer.modify(|_, w| w.cc2p().bit(false) );
                tim.ccer.modify(|_, w| w.cc2np().bit(false) );

                // 5. Program the input prescaler. In our example, we wish the capture to be performed at
                // each valid transition, so the prescaler is disabled (write IC1PS bits to 00 in the
                // TIMx_CCMR1 register).
                tim.ccmr1_input().modify(|_, w| unsafe { w.ic1psc().bits(0b00) });
                tim.ccmr1_input().modify(|_, w| unsafe { w.ic2psc().bits(0b00) });

                // 6. Enable capture from the counter into the capture register by setting the CC1E bit in the
                // TIMx_CCER register.
                // CCxE enable output
                tim.ccer.modify(|_, w| w.cc1e().set_bit());
                tim.ccer.modify(|_, w| w.cc2e().set_bit());

                // 7. If needed, enable the related interrupt request by setting the CC1IE bit in the
                // TIMx_DIER register, and/or the DMA request by setting the CC1DE bit in the
                // TIMx_DIER register
                // no interrupt

                // • SMS= 011 (TIMx_SMCR register, both inputs are active on both rising and falling
                // edges)
                tim.smcr.modify(|_, w| unsafe { w.sms().bits(0b0011) });


                // OCxM mode
                // tim.ccmr1_output().modify(|_, w| w.oc1m().pwm_mode1());
                // tim.ccmr1_output().modify(|_, w| w.oc2m().pwm_mode1());
                // CCRx
                // tim.ccr1.modify(|_, w| unsafe { w.ccr1().bits(0) }); // x/800
                // tim.ccr2.modify(|_, w| unsafe { w.ccr2().bits(0) }); // x/800

                // Set polarity
                // tim.ccer.modify(|_, w| w.cc1p().clear_bit());
                // tim.ccer.modify(|_, w| w.cc2p().clear_bit());
                // PWM mode
                // tim.cr1.modify(|_, w| unsafe { w.cms().bits(0b00) }); 

                // enable tim
                tim.cr1.modify(|_, w| w.cen().set_bit());
                // Main output enable
                // tim.bdtr.modify(|_, w| w.moe().set_bit());
            }
        });
    }
}


pub struct DcPwm {}
impl<'a> DcPwm {
    pub fn new() -> Self {
        Self {}
    }
    pub fn init(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                // GPIOポートの電源投入(クロックの有効化)
                perip.RCC.iopenr.modify(|_, w| w.iopaen().set_bit());
                perip.RCC.iopenr.modify(|_, w| w.iopben().set_bit());

                let gpioa = &perip.GPIOA;
                let gpiob = &perip.GPIOB;
                // PWM pin
                gpioa.moder.modify(|_, w| w.moder8().alternate());
                gpiob.moder.modify(|_, w| w.moder3().alternate());
                gpioa.afrh.modify(|_, w| w.afsel8().af2()); // TIM1 CH1
                gpiob.afrl.modify(|_, w| w.afsel3().af1()); // TIM1 CH2
                gpioa.ospeedr.modify(|_, w| w.ospeedr8().very_high_speed());
                gpiob.ospeedr.modify(|_, w| w.ospeedr3().very_high_speed());

                perip.RCC.apbenr2.modify(|_, w| w.tim1en().set_bit());

                // For PWM
                let tim = &perip.TIM1;
                tim.psc.modify(|_, w| unsafe { w.bits(7 - 1) });
                tim.arr.modify(|_, w| unsafe { w.bits(800 - 1) }); // 25kHz

                // OCxM mode
                tim.ccmr1_output().modify(|_, w| w.oc1m().pwm_mode1());
                tim.ccmr1_output().modify(|_, w| w.oc2m().pwm_mode1());
                // CCRx
                tim.ccr1.modify(|_, w| unsafe { w.ccr1().bits(0) }); // x/800
                tim.ccr2.modify(|_, w| unsafe { w.ccr2().bits(0) }); // x/800

                // Set polarity
                // tim.ccer.modify(|_, w| w.cc1p().clear_bit());
                // tim.ccer.modify(|_, w| w.cc2p().clear_bit());
                // PWM mode
                // tim.cr1.modify(|_, w| unsafe { w.cms().bits(0b00) }); 

                // enable tim
                tim.cr1.modify(|_, w| w.cen().set_bit());
                // Main output enable
                tim.bdtr.modify(|_, w| w.moe().set_bit());
                // CCxE enable output
                tim.ccer.modify(|_, w| w.cc1e().set_bit());
                tim.ccer.modify(|_, w| w.cc2e().set_bit());
            }
        });
    }
}

impl DcMotorDriver for DcPwm {
    fn enable(&self) {}
    fn disable(&self) {}
    /// 0~1
    fn set_pwm(&self, direction: f32, value: f32) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let tim = &perip.TIM1;
                if direction >= 0.0 {
                    tim.ccr1
                        .modify(|_, w| unsafe { w.ccr1().bits((value * 800.) as u16) }); // x/800
                    tim.ccr2.modify(|_, w| unsafe { w.ccr2().bits(0) });
                } else {
                    tim.ccr2
                        .modify(|_, w| unsafe { w.ccr2().bits((value * 800.) as u16) }); // x/800
                    tim.ccr1.modify(|_, w| unsafe { w.ccr1().bits(0) });
                }
            }
        });
    }
}


pub struct Led0 {}

impl Indicator for Led0 {
    fn on(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let gpioa = &perip.GPIOA;
                gpioa.bsrr.write(|w| w.br4().reset());
            }
        });
    }
    fn off(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let gpioa = &perip.GPIOA;
                gpioa.bsrr.write(|w| w.bs4().set());
            }
        });
    }
    fn toggle(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let gpioa = &perip.GPIOA;
                if gpioa.odr.read().odr4().is_low() {
                    gpioa.bsrr.write(|w| w.bs4().set());
                } else {
                    gpioa.bsrr.write(|w| w.br4().reset());
                }
            }
        });
    }
}

impl Led0 {
    pub fn new() -> Self {
        Self {}
    }

    pub fn init(&self) {
        free(|cs| {
            match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
                None => (),
                Some(perip) => {
                    // GPIOポートの電源投入(クロックの有効化)
                    perip.RCC.iopenr.modify(|_, w| w.iopaen().set_bit());
                    // gpioモード変更
                    let gpioa = &perip.GPIOA;
                    gpioa.moder.modify(|_, w| w.moder4().output());
                }
            }
        });
    }
}

pub struct Led1 {}

impl Indicator for Led1 {
    fn on(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let gpioa = &perip.GPIOA;
                gpioa.bsrr.write(|w| w.br5().reset());
            }
        });
    }
    fn off(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let gpioa = &perip.GPIOA;
                gpioa.bsrr.write(|w| w.bs5().set());
            }
        });
    }
    fn toggle(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let gpioa = &perip.GPIOA;
                if gpioa.odr.read().odr5().is_low() {
                    gpioa.bsrr.write(|w| w.bs5().set());
                } else {
                    gpioa.bsrr.write(|w| w.br5().reset());
                }
            }
        });
    }
}

impl Led1 {
    pub fn new() -> Self {
        Self {}
    }

    pub fn init(&self) {
        free(|cs| {
            match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
                None => (),
                Some(perip) => {
                    // GPIOポートの電源投入(クロックの有効化)
                    perip.RCC.iopenr.modify(|_, w| w.iopaen().set_bit());
                    // gpioモード変更
                    let gpioa = &perip.GPIOA;
                    gpioa.moder.modify(|_, w| w.moder5().output());
                }
            }
        });
    }
}
