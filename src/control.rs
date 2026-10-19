pub const DEFAULT_CONTROL_PERIOD_SECONDS: f32 = 0.001;
pub const PWM_RAW_MAX: i16 = 885;
// Motor encoder: 11 pulses/rev, quadrature x4, reduction ratio 18.75:1.
pub const ENCODER_COUNTS_PER_OUTPUT_REVOLUTION: f32 = 11.0 * 4.0 * 18.75; // 825

const POSITION_P_GAIN_SCALE: f32 = 128.0;
const POSITION_I_GAIN_SCALE: f32 = 65_536.0;
const POSITION_D_GAIN_SCALE: f32 = 16.0;

pub struct VelocityPi {
    integral_error: f32,
}

impl VelocityPi {
    pub const fn new() -> Self {
        Self {
            integral_error: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.integral_error = 0.0;
    }

    pub fn update(
        &mut self,
        goal: i32,
        velocity: f32,
        p_gain_raw: u16,
        i_gain_raw: u16,
        output_limit: f32,
        dt_seconds: f32,
    ) -> f32 {
        if output_limit <= 0.0 || dt_seconds <= 0.0 {
            self.reset();
            return 0.0;
        }
        let error = goal as f32 - velocity;
        let proportional = p_gain_raw as f32 / 128.0 * error;
        let ki = i_gain_raw as f32 / 65_536.0;
        let integral = self.integral_error + error * dt_seconds / DEFAULT_CONTROL_PERIOD_SECONDS;
        let candidate = proportional + ki * integral;
        if !((candidate > output_limit && error > 0.0)
            || (candidate < -output_limit && error < 0.0))
        {
            self.integral_error = integral;
        }
        clamp(
            proportional + ki * self.integral_error,
            -output_limit,
            output_limit,
        )
    }
}

pub struct EncoderTracker {
    previous_raw: Option<u16>,
    position: i32,
}

impl EncoderTracker {
    pub const fn new() -> Self {
        Self {
            previous_raw: None,
            position: 0,
        }
    }

    pub fn reset(&mut self) {
        self.previous_raw = None;
        self.position = 0;
    }

    pub fn update(&mut self, raw: u16) -> (i32, i32) {
        let delta = match self.previous_raw {
            Some(previous) => raw.wrapping_sub(previous) as i16 as i32,
            None => 0,
        };

        self.previous_raw = Some(raw);
        self.position = self.position.saturating_add(delta);
        (self.position, delta)
    }
}

pub struct LowPassFilter {
    value: Option<f32>,
}

impl LowPassFilter {
    pub const fn new() -> Self {
        Self { value: None }
    }

    pub fn reset(&mut self) {
        self.value = None;
    }

    pub fn update(&mut self, input: f32, dt_seconds: f32, cutoff_hz: f32) -> f32 {
        if dt_seconds <= 0.0 || cutoff_hz <= 0.0 {
            self.value = Some(input);
            return input;
        }

        let time_constant = 1.0 / (2.0 * core::f32::consts::PI * cutoff_hz);
        let alpha = dt_seconds / (time_constant + dt_seconds);
        let output = match self.value {
            Some(previous) => previous + alpha * (input - previous),
            None => input,
        };
        self.value = Some(output);
        output
    }
}

pub struct PositionPid {
    integral_error: f32,
}

pub struct PositionPidParameters {
    pub p_gain_raw: u16,
    pub i_gain_raw: u16,
    pub d_gain_raw: u16,
    pub output_limit: f32,
    pub dt_seconds: f32,
    pub velocity_counts_per_second: f32,
}

impl PositionPid {
    pub const fn new() -> Self {
        Self {
            integral_error: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.integral_error = 0.0;
    }

    pub fn update(&mut self, goal: i32, position: i32, parameters: PositionPidParameters) -> f32 {
        if parameters.output_limit <= 0.0 || parameters.dt_seconds <= 0.0 {
            self.reset();
            return 0.0;
        }

        let error = goal.saturating_sub(position) as f32;
        let kp = parameters.p_gain_raw as f32 / POSITION_P_GAIN_SCALE;
        let ki = parameters.i_gain_raw as f32 / POSITION_I_GAIN_SCALE;
        let kd = parameters.d_gain_raw as f32 / POSITION_D_GAIN_SCALE;
        let proportional = kp * error;
        // Reference the raw control-table gains to a 1 ms control cycle,
        // while allowing the actual control period to be configured.
        let sample_scale = parameters.dt_seconds / DEFAULT_CONTROL_PERIOD_SECONDS;
        // Reuse filtered encoder velocity instead of differentiating quantized positions.
        let derivative =
            -kd * parameters.velocity_counts_per_second * DEFAULT_CONTROL_PERIOD_SECONDS;

        let candidate_integral = self.integral_error + error * sample_scale;
        let candidate_output = proportional + ki * candidate_integral + derivative;
        let drives_further_into_saturation = (candidate_output > parameters.output_limit
            && error > 0.0)
            || (candidate_output < -parameters.output_limit && error < 0.0);
        if !drives_further_into_saturation {
            self.integral_error = candidate_integral;
        }

        clamp(
            proportional + ki * self.integral_error + derivative,
            -parameters.output_limit,
            parameters.output_limit,
        )
    }
}

pub fn clamp(value: f32, minimum: f32, maximum: f32) -> f32 {
    if value < minimum {
        minimum
    } else if value > maximum {
        maximum
    } else {
        value
    }
}

pub fn velocity_from_encoder_delta(encoder_delta: i32, control_period_seconds: f32) -> f32 {
    if control_period_seconds <= 0.0 {
        return 0.0;
    }
    encoder_delta as f32 / control_period_seconds
}

pub fn pwm_limit(configured_limit: u16, goal_pwm: i16) -> i16 {
    let hardware_limit = i32::from(PWM_RAW_MAX);
    let configured = i32::from(configured_limit).min(hardware_limit);
    let goal = i32::from(goal_pwm).abs().min(hardware_limit);
    configured.min(goal) as i16
}

pub fn clamp_i16(value: i16, minimum: i16, maximum: i16) -> i16 {
    if value < minimum {
        minimum
    } else if value > maximum {
        maximum
    } else {
        value
    }
}

pub fn clamp_i32(value: i32, minimum: i32, maximum: i32) -> i32 {
    if minimum > maximum {
        value
    } else if value < minimum {
        minimum
    } else if value > maximum {
        maximum
    } else {
        value
    }
}

pub fn u32_to_i32(value: u32) -> i32 {
    if value > i32::MAX as u32 {
        i32::MAX
    } else {
        value as i32
    }
}

#[cfg(test)]
mod tests {
    use super::{
        clamp_i16, clamp_i32, pwm_limit, velocity_from_encoder_delta, EncoderTracker,
        LowPassFilter, PositionPid, PositionPidParameters, VelocityPi,
        DEFAULT_CONTROL_PERIOD_SECONDS, ENCODER_COUNTS_PER_OUTPUT_REVOLUTION, PWM_RAW_MAX,
    };

    fn parameters(p: u16, i: u16, d: u16, output_limit: f32) -> PositionPidParameters {
        PositionPidParameters {
            p_gain_raw: p,
            i_gain_raw: i,
            d_gain_raw: d,
            output_limit,
            dt_seconds: DEFAULT_CONTROL_PERIOD_SECONDS,
            velocity_counts_per_second: 0.0,
        }
    }

    #[test]
    fn tracks_forward_counter_wrap() {
        let mut tracker = EncoderTracker::new();

        assert_eq!(tracker.update(u16::MAX - 1), (0, 0));
        assert_eq!(tracker.update(1), (3, 3));
    }

    #[test]
    fn tracks_reverse_counter_wrap() {
        let mut tracker = EncoderTracker::new();

        assert_eq!(tracker.update(1), (0, 0));
        assert_eq!(tracker.update(u16::MAX - 1), (-3, -3));
    }

    #[test]
    fn limits_proportional_output() {
        let mut pid = PositionPid::new();

        assert_eq!(pid.update(100, 0, parameters(128, 0, 0, 50.0)), 50.0);
        assert_eq!(pid.update(-100, 0, parameters(128, 0, 0, 50.0)), -50.0);
    }

    #[test]
    fn differentiates_the_measurement() {
        let mut pid = PositionPid::new();

        assert_eq!(pid.update(0, 0, parameters(0, 0, 16, 100.0)), 0.0);
        let mut moving = parameters(0, 0, 16, 100.0);
        moving.velocity_counts_per_second = 2000.0;
        assert_eq!(pid.update(0, 2, moving), -2.0);
    }

    #[test]
    fn position_derivative_uses_physical_velocity_without_setpoint_kick() {
        let mut pid = PositionPid::new();
        assert_eq!(pid.update(1000, 0, parameters(0, 0, 16, 100.0)), 0.0);
        let mut moving = parameters(0, 0, 16, 100.0);
        moving.dt_seconds = 0.002;
        moving.velocity_counts_per_second = 2000.0;
        assert_eq!(pid.update(1000, 0, moving), -2.0);
    }

    #[test]
    fn prevents_integral_windup_while_saturated() {
        let mut pid = PositionPid::new();

        assert_eq!(pid.update(10, 0, parameters(0, u16::MAX, 0, 5.0)), 0.0);
        assert_eq!(pid.update(0, 0, parameters(0, u16::MAX, 0, 5.0)), 0.0);
    }

    #[test]
    fn reset_clears_accumulated_error() {
        let mut pid = PositionPid::new();

        let first = pid.update(1, 0, parameters(0, u16::MAX, 0, 100.0));
        assert!(pid.update(1, 0, parameters(0, u16::MAX, 0, 100.0)) > first);
        pid.reset();
        assert_eq!(pid.update(1, 0, parameters(0, u16::MAX, 0, 100.0)), first);
    }

    #[test]
    fn filters_velocity_changes() {
        let mut filter = LowPassFilter::new();

        assert_eq!(filter.update(0.0, 0.001, 1000.0), 0.0);
        let filtered = filter.update(100.0, 0.001, 1000.0);
        assert!(filtered > 0.0 && filtered < 100.0);
    }

    #[test]
    fn reports_velocity_in_encoder_counts_per_second() {
        assert_eq!(ENCODER_COUNTS_PER_OUTPUT_REVOLUTION, 825.0);
        assert_eq!(velocity_from_encoder_delta(1, 0.001), 1.0 / 0.001);
        assert_eq!(velocity_from_encoder_delta(-2, 0.002), -2.0 / 0.002);
        assert_eq!(velocity_from_encoder_delta(825, 1.0), 825.0);
        assert_eq!(velocity_from_encoder_delta(1, 0.0), 0.0);
    }

    #[test]
    fn preserves_integral_output_when_the_position_error_reaches_zero() {
        let mut pid = PositionPid::new();
        let output = pid.update(1, 0, parameters(0, 4096, 0, 100.0));
        assert!(output > 0.0);
        assert_eq!(pid.update(1, 1, parameters(0, 4096, 0, 100.0)), output);
    }

    #[test]
    fn limits_pwm_and_position_boundaries() {
        assert_eq!(pwm_limit(1000, i16::MIN), PWM_RAW_MAX);
        assert_eq!(pwm_limit(400, 300), 300);
        assert_eq!(clamp_i16(500, -300, 300), 300);
        assert_eq!(clamp_i32(-1, 0, 4095), 0);
        assert_eq!(clamp_i32(5000, 0, 4095), 4095);
    }

    #[test]
    fn velocity_integral_unwinds_after_saturation() {
        let mut pi = VelocityPi::new();
        for _ in 0..1000 {
            assert_eq!(pi.update(100, 0.0, 128, 4096, 50.0, 0.001), 50.0);
        }
        assert_eq!(pi.update(0, 0.0, 128, 4096, 50.0, 0.001), 0.0);
    }

    #[test]
    fn velocity_integral_uses_period_and_resets() {
        let mut pi = VelocityPi::new();
        let mut slower = VelocityPi::new();
        pi.update(10, 0.0, 0, 4096, 100.0, 0.001);
        let fast = pi.update(10, 0.0, 0, 4096, 100.0, 0.001);
        let slow = slower.update(10, 0.0, 0, 4096, 100.0, 0.002);
        assert_eq!(fast, slow);
        pi.reset();
        assert_eq!(pi.update(0, 0.0, 128, 4096, 100.0, 0.001), 0.0);
        assert_eq!(pi.update(10, 0.0, 128, 4096, 0.0, 0.001), 0.0);
        assert_eq!(pi.update(10, 0.0, 128, 4096, 100.0, 0.0), 0.0);
    }
}
