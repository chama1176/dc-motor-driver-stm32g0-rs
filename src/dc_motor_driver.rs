pub trait DcMotorDriver {
    fn enable(&self);
    fn disable(&self);
    fn set_pwm(&self, direction: f32, value: f32);
}
