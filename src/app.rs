use core::convert::TryInto;

use dynamixel_f_rs::control_table::BitsW;

use crate::indicator::Indicator;

pub struct App<T0, T1, I, C>
where
    T0: Indicator,
    T1: Indicator,
    I: dynamixel_f_rs::BufferInterface+dynamixel_f_rs::QueueInterface,
    C: dynamixel_f_rs::Clock,
{
    led0: T0,
    led1: T1,
    dxl: dynamixel_f_rs::DynamixelProtocolHandler<I, C>,
}

impl<T0, T1, I, C> App<T0, T1, I, C>
where
    T0: Indicator,
    T1: Indicator,
    I: dynamixel_f_rs::BufferInterface+dynamixel_f_rs::QueueInterface,
    C: dynamixel_f_rs::Clock,
{
    pub fn new(
        led0: T0, 
        led1: T1,
        mut buffer_interface: I,
        clock: C,
    ) -> Self {
        let ctd = dynamixel_f_rs::ControlTableData::new();
        let dxl =
            dynamixel_f_rs::DynamixelProtocolHandler::new(buffer_interface, clock, 115200, ctd);
        Self {
            led0,
            led1,
            dxl,
        }
    }
    pub fn periodic_task(&self) {
        self.led0.toggle();
        self.led1.toggle();
    }
}
