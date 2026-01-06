use core::convert::TryInto;

use crate::indicator::Indicator;

pub struct App<T0, T1>
where
    T0: Indicator,
    T1: Indicator,
{
    led0: T0,
    led1: T1,
}

impl<T0, T1> App<T0, T1>
where
    T0: Indicator,
    T1: Indicator,
{
    pub fn new(led0: T0, led1: T1) -> Self {
        Self {
            led0,
            led1,
        }
    }
    pub fn periodic_task(&self) {
        self.led0.toggle();
        self.led1.toggle();
    }
}
