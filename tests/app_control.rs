use super::*;
use core::{cell::Cell, time::Duration};
use std::rc::Rc;

struct Led;
impl Indicator for Led {
    fn on(&self) {}
    fn off(&self) {}
    fn toggle(&self) {}
}
struct Uart;
impl dynamixel_f_rs::BufferInterface for Uart {
    fn write_byte(&mut self, _: u8) {}
    fn write_bytes(&mut self, _: &[u8]) {}
    fn read_byte(&mut self) -> Option<u8> {
        None
    }
    fn read_bytes(&mut self, _: &mut [u8]) -> Option<usize> {
        None
    }
    fn clear_read_buf(&mut self) {}
}
impl dynamixel_f_rs::QueueInterface for Uart {
    fn enqueue(&mut self, _: u8) -> Result<(), ()> {
        Ok(())
    }
}
struct Clock;
impl dynamixel_f_rs::Clock for Clock {
    fn get_current_time(&self) -> Duration {
        Duration::ZERO
    }
}
#[derive(Default)]
struct Motor {
    enabled: Cell<bool>,
    direction: Cell<f32>,
    duty: Cell<f32>,
}
struct Driver(Rc<Motor>);
impl DcMotorDriver for Driver {
    fn enable(&self) {
        self.0.enabled.set(true);
    }
    fn disable(&self) {
        self.0.enabled.set(false);
    }
    fn set_pwm(&self, direction: f32, duty: f32) {
        self.0.direction.set(direction);
        self.0.duty.set(duty);
    }
}
struct Sensor(Rc<Cell<Option<u16>>>);
impl Encoder<u16> for Sensor {
    fn get_angle(&self) -> Option<u16> {
        self.0.get()
    }
    fn reset_error(&self) {}
}
type TestApp = App<Led, Led, Uart, Clock, Driver, Sensor>;
fn fixture() -> (TestApp, Rc<Motor>, Rc<Cell<Option<u16>>>) {
    let motor = Rc::new(Motor::default());
    let sensor = Rc::new(Cell::new(Some(0)));
    let mut app = App::new(
        Led,
        Led,
        Uart,
        Clock,
        Driver(motor.clone()),
        Sensor(sensor.clone()),
        0.001,
    );
    app.init();
    app.control();
    (app, motor, sensor)
}

#[test]
fn boot_and_torque_off_disable_output() {
    let (mut app, motor, _) = fixture();
    assert!(!motor.enabled.get());
    let defaults = app.control_table().read();
    assert_eq!(
        (
            defaults.position_pgain(),
            defaults.position_igain(),
            defaults.position_dgain()
        ),
        (8192, 11830, 32000)
    );
    assert_eq!(
        (defaults.velocity_pgain(), defaults.velocity_igain()),
        (160, 1640)
    );
    assert_eq!(defaults.pwm_limit(), 443);
    app.dxl
        .ctd
        .modify(|_, w| w.torque_enable().bits(1).goal_position().bits(100));
    app.control();
    assert!(motor.duty.get() > 0.0);
    app.dxl.ctd.modify(|_, w| w.torque_enable().bits(0));
    app.control();
    assert!(!motor.enabled.get());
    assert_eq!(motor.duty.get(), 0.0);
    assert_eq!(app.dxl.ctd.read().present_pwm(), 0);
}

#[test]
fn reports_one_output_revolution_after_825_encoder_counts() {
    let (mut app, _, sensor) = fixture();
    sensor.set(Some(825));
    app.control();
    assert_eq!(app.control_table().read().present_position(), 825);
}

#[test]
fn position_feedback_reverses_output_after_overshoot() {
    let (mut app, motor, sensor) = fixture();
    app.dxl.ctd.modify(|_, w| {
        w.torque_enable()
            .bits(1)
            .goal_position()
            .bits(100)
            .pwm_limit()
            .bits(50)
    });
    app.control();
    assert_eq!(app.dxl.ctd.read().present_pwm(), 50);
    assert_eq!(motor.direction.get(), 1.0);
    sensor.set(Some(120));
    app.control();
    assert_eq!(app.dxl.ctd.read().present_position(), 120);
    assert_eq!(motor.direction.get(), -1.0);
    assert!(motor.duty.get() <= 50.0 / 885.0);
}

#[test]
fn velocity_control_accepts_both_signs_and_limits_goal() {
    let (mut app, motor, _) = fixture();
    app.dxl.ctd.modify(|_, w| {
        w.operating_mode()
            .bits(1)
            .torque_enable()
            .bits(1)
            .goal_velocity()
            .bits(1000)
            .velocity_limit()
            .bits(30)
            .velocity_pgain()
            .bits(128)
            .velocity_igain()
            .bits(0)
    });
    app.control();
    assert_eq!(app.dxl.ctd.read().present_pwm(), 30);
    app.dxl.ctd.modify(|_, w| w.goal_velocity().bits(-1000));
    app.control();
    assert_eq!(app.dxl.ctd.read().present_pwm(), -30);
    assert_eq!(motor.direction.get(), -1.0);
}

#[test]
fn mode_change_clears_velocity_integral() {
    let (mut app, _, _) = fixture();
    app.dxl.ctd.modify(|_, w| {
        w.operating_mode()
            .bits(1)
            .torque_enable()
            .bits(1)
            .goal_velocity()
            .bits(10)
    });
    for _ in 0..1000 {
        app.control();
    }
    assert!(app.dxl.ctd.read().present_pwm() > 10);
    app.dxl
        .ctd
        .modify(|_, w| w.operating_mode().bits(3).goal_position().bits(0));
    app.control();
    app.dxl
        .ctd
        .modify(|_, w| w.operating_mode().bits(1).goal_velocity().bits(0));
    app.control();
    assert_eq!(app.dxl.ctd.read().present_pwm(), 0);
}

#[test]
fn missing_encoder_or_unsupported_mode_stops_motor() {
    let (mut app, motor, sensor) = fixture();
    app.dxl.ctd.modify(|_, w| {
        w.operating_mode()
            .bits(16)
            .torque_enable()
            .bits(1)
            .goal_pwm()
            .bits(-100)
    });
    app.control();
    assert!(motor.enabled.get());
    sensor.set(None);
    app.control();
    assert!(!motor.enabled.get());
    sensor.set(Some(0));
    app.dxl.ctd.modify(|_, w| w.operating_mode().bits(99));
    app.control();
    assert!(!motor.enabled.get());
}
