pub const DEFAULT_CONTROL_PERIOD_SECONDS: f32 = 0.001;
pub const PWM_RAW_MAX: i16 = 885;
pub const POSITION_UNITS_PER_REVOLUTION: f32 = 4096.0;
pub const VELOCITY_RPM_PER_UNIT: f32 = 0.229;

const POSITION_P_GAIN_SCALE: f32 = 128.0;
const POSITION_I_GAIN_SCALE: f32 = 65_536.0;
const POSITION_D_GAIN_SCALE: f32 = 16.0;

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
    previous_position: Option<i32>,
}

pub struct PositionPidParameters {
    pub p_gain_raw: u16,
    pub i_gain_raw: u16,
    pub d_gain_raw: u16,
    pub output_limit: f32,
    pub dt_seconds: f32,
}

impl PositionPid {
    pub const fn new() -> Self {
        Self {
            integral_error: 0.0,
            previous_position: None,
        }
    }

    pub fn reset(&mut self) {
        self.integral_error = 0.0;
        self.previous_position = None;
    }

    pub fn update(&mut self, goal: i32, position: i32, parameters: PositionPidParameters) -> f32 {
        if parameters.output_limit <= 0.0 || parameters.dt_seconds <= 0.0 {
            self.reset();
            return 0.0;
        }

        let error = goal.saturating_sub(position) as f32;
        let position_delta = match self.previous_position {
            Some(previous) => position.saturating_sub(previous) as f32,
            None => 0.0,
        };
        self.previous_position = Some(position);

        let kp = parameters.p_gain_raw as f32 / POSITION_P_GAIN_SCALE;
        let ki = parameters.i_gain_raw as f32 / POSITION_I_GAIN_SCALE;
        let kd = parameters.d_gain_raw as f32 / POSITION_D_GAIN_SCALE;
        let proportional = kp * error;
        // Keep the XC330-compatible gains referenced to a 1 ms control cycle,
        // while allowing the actual control period to be configured.
        let sample_scale = parameters.dt_seconds / DEFAULT_CONTROL_PERIOD_SECONDS;
        let derivative = -kd * position_delta / sample_scale;

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

pub fn position_from_encoder(encoder_position: i32, encoder_counts_per_revolution: f32) -> i32 {
    if encoder_counts_per_revolution <= 0.0 {
        return 0;
    }
    (encoder_position as f32 * POSITION_UNITS_PER_REVOLUTION / encoder_counts_per_revolution) as i32
}

pub fn velocity_from_encoder_delta(
    encoder_delta: i32,
    encoder_counts_per_revolution: f32,
    control_period_seconds: f32,
) -> f32 {
    if encoder_counts_per_revolution <= 0.0 || control_period_seconds <= 0.0 {
        return 0.0;
    }
    let revolutions_per_minute =
        encoder_delta as f32 * 60.0 / (encoder_counts_per_revolution * control_period_seconds);
    revolutions_per_minute / VELOCITY_RPM_PER_UNIT
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
        clamp_i16, clamp_i32, position_from_encoder, pwm_limit, velocity_from_encoder_delta,
        EncoderTracker, LowPassFilter, PositionPid, PositionPidParameters,
        DEFAULT_CONTROL_PERIOD_SECONDS, PWM_RAW_MAX,
    };

    fn parameters(p: u16, i: u16, d: u16, output_limit: f32) -> PositionPidParameters {
        PositionPidParameters {
            p_gain_raw: p,
            i_gain_raw: i,
            d_gain_raw: d,
            output_limit,
            dt_seconds: DEFAULT_CONTROL_PERIOD_SECONDS,
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
        assert_eq!(pid.update(0, 2, parameters(0, 0, 16, 100.0)), -2.0);
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

        assert!(pid.update(1, 0, parameters(0, u16::MAX, 0, 100.0)) > 0.0);
        pid.reset();
        assert_eq!(pid.update(0, 0, parameters(0, u16::MAX, 0, 100.0)), 0.0);
    }

    #[test]
    fn filters_velocity_changes() {
        let mut filter = LowPassFilter::new();

        assert_eq!(filter.update(0.0, 0.001, 1000.0), 0.0);
        let filtered = filter.update(100.0, 0.001, 1000.0);
        assert!(filtered > 0.0 && filtered < 100.0);
    }

    #[test]
    fn converts_encoder_values_to_xc330_units() {
        assert_eq!(position_from_encoder(2048, 4096.0), 2048);
        assert_eq!(position_from_encoder(-4096, 4096.0), -4096);

        let velocity = velocity_from_encoder_delta(4096, 4096.0, 60.0);
        assert!((velocity - 1.0 / 0.229).abs() < 0.0001);
    }

    #[test]
    fn limits_pwm_and_position_boundaries() {
        assert_eq!(pwm_limit(1000, i16::MIN), PWM_RAW_MAX);
        assert_eq!(pwm_limit(400, 300), 300);
        assert_eq!(clamp_i16(500, -300, 300), 300);
        assert_eq!(clamp_i32(-1, 0, 4095), 0);
        assert_eq!(clamp_i32(5000, 0, 4095), 4095);
    }
}
