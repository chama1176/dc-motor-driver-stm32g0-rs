#![allow(dead_code)]
// use core::convert::TryInto;

// use dynamixel_f_rs::control_table::BitsW;

use crate::indicator::Indicator;
use motml::motor_driver::DcMotorDriver;

pub struct App<T0, T1, I, C, D>
where
    T0: Indicator,
    T1: Indicator,
    I: dynamixel_f_rs::BufferInterface+dynamixel_f_rs::QueueInterface,
    C: dynamixel_f_rs::Clock,
    D: DcMotorDriver,
{
    led0: T0,
    led1: T1,
    dxl: dynamixel_f_rs::DynamixelProtocolHandler<I, C>,
    md: D,
}

impl<T0, T1, I, C, D> App<T0, T1, I, C, D>
where
    T0: Indicator,
    T1: Indicator,
    I: dynamixel_f_rs::BufferInterface+dynamixel_f_rs::QueueInterface,
    C: dynamixel_f_rs::Clock,
    D: DcMotorDriver,
{
    pub fn new(
        led0: T0, 
        led1: T1,
        #[allow(unused_mut)]
        mut buffer_interface: I, 
        clock: C,
        md: D,
    ) -> Self {
        let ctd = dynamixel_f_rs::ControlTableData::new();
        let dxl =
            dynamixel_f_rs::DynamixelProtocolHandler::new(buffer_interface, clock, 115200, ctd);
        Self {
            led0,
            led1,
            dxl,
            md,
        }
    }
    pub fn periodic_task(&self) {
        self.led0.toggle();

        if self.dxl.ctd.read().led() == 1{
            self.led1.on();
        } else {
            self.led1.off();
        }
    }

    pub fn control(&self) {
        self.md.set_pwm(1.0, 0.0);
    }

    pub fn enque_uart(&mut self, data: u8) {
        self.dxl.uart.enqueue(data).unwrap();
    }

    pub fn parse_uart_task(&mut self) {
        // Dxl処理(受信があった場合自動返信するはず)
        let r = self.dxl.parse_data();
        match r {
            Ok(_) => {
            }
            Err(_e) => {
                defmt::info!("error");
            }
        }
    }

}
