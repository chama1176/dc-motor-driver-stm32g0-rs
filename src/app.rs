#![allow(dead_code)]
use crate::control::{
    clamp_i16, clamp_i32, pwm_limit, u32_to_i32, velocity_from_encoder_delta, EncoderTracker,
    LowPassFilter, PositionPid, PositionPidParameters, VelocityPi,
    ENCODER_COUNTS_PER_OUTPUT_REVOLUTION, PWM_RAW_MAX,
};
use crate::indicator::Indicator;
use dynamixel_f_rs::control_table::BitsW;
use motml::encoder::Encoder;
use motml::motor_driver::DcMotorDriver;

const OPERATING_MODE_POSITION: u8 = 3;
const OPERATING_MODE_VELOCITY: u8 = 1;
const OPERATING_MODE_PWM: u8 = 16;
// Position: encoder counts. Velocity: encoder counts/second.
pub(crate) const VELOCITY_FILTER_CUTOFF_HZ: f32 = 10.0;
pub(crate) const POSITION_D_FILTER_CUTOFF_HZ: f32 = 20.0;

#[cfg(test)]
#[path = "../tests/app_control.rs"]
mod tests;

pub struct App<T0, T1, I, C, D, E>
where
    T0: Indicator,
    T1: Indicator,
    I: dynamixel_f_rs::BufferInterface + dynamixel_f_rs::QueueInterface,
    C: dynamixel_f_rs::Clock,
    D: DcMotorDriver,
    E: Encoder<u16>,
{
    led0: T0,
    led1: T1,
    dxl: dynamixel_f_rs::DynamixelProtocolHandler<I, C>,
    md: D,
    encoder: E,
    encoder_tracker: EncoderTracker,
    velocity_filter: LowPassFilter,
    position_pid: PositionPid,
    velocity_pi: VelocityPi,
    control_period_seconds: f32,
    realtime_tick: u16,
    last_torque_enabled: bool,
    last_operating_mode: u8,
}

impl<T0, T1, I, C, D, E> App<T0, T1, I, C, D, E>
where
    T0: Indicator,
    T1: Indicator,
    I: dynamixel_f_rs::BufferInterface + dynamixel_f_rs::QueueInterface,
    C: dynamixel_f_rs::Clock,
    D: DcMotorDriver,
    E: Encoder<u16>,
{
    pub fn new(
        led0: T0,
        led1: T1,
        buffer_interface: I,
        clock: C,
        md: D,
        encoder: E,
        control_period_seconds: f32,
    ) -> Self {
        let ctd = dynamixel_f_rs::ControlTableData::new();

        let dxl =
            dynamixel_f_rs::DynamixelProtocolHandler::new(buffer_interface, clock, 115200, ctd);
        Self {
            led0,
            led1,
            dxl,
            md,
            encoder,
            encoder_tracker: EncoderTracker::new(),
            velocity_filter: LowPassFilter::new(),
            position_pid: PositionPid::new(),
            velocity_pi: VelocityPi::new(),
            control_period_seconds,
            realtime_tick: 0,
            last_torque_enabled: false,
            last_operating_mode: OPERATING_MODE_POSITION,
        }
    }

    pub fn init(&self) {
        // ctdの初期値は全部0
        self.dxl.ctd.modify(|_, w| w.id().bits(1));

        self.dxl.ctd.modify(|_, w| w.model_number().bits(0x04BA)); // 0x04BA: XC330-T181-T
        self.dxl.ctd.modify(|_, w| w.firmware_version().bits(0x30));
        self.dxl.ctd.modify(|_, w| w.baud_rate().bits(0x02)); // 0x02: 115200bps
        self.dxl.ctd.modify(|_, w| w.return_delay_time().bits(0x00));
        self.dxl.ctd.modify(|_, w| w.drive_mode().bits(0x00));
        self.dxl.ctd.modify(|_, w| w.operating_mode().bits(0x03)); // 0x03: position control
        self.dxl.ctd.modify(|_, w| w.secondary_id().bits(0xFF));
        self.dxl.ctd.modify(|_, w| w.protocol_type().bits(0x02));
        self.dxl.ctd.modify(|_, w| w.moving_threshold().bits(32));
        self.dxl.ctd.modify(|_, w| w.temperature_limit().bits(0x46)); // 70℃
        self.dxl.ctd.modify(|_, w| w.max_voltage_limit().bits(0x8C)); // 14V
        self.dxl.ctd.modify(|_, w| w.min_voltage_limit().bits(0x37)); // 5.5V
        self.dxl.ctd.modify(|_, w| w.pwm_limit().bits(443));
        self.dxl.ctd.modify(|_, w| {
            w.max_position_limit()
                .bits(ENCODER_COUNTS_PER_OUTPUT_REVOLUTION as u32)
        });
        self.dxl.ctd.modify(|_, w| w.min_position_limit().bits(0));
        self.dxl.ctd.modify(|_, w| w.status_return_level().bits(2));

        self.dxl.ctd.modify(|_, w| {
            w.torque_enable()
                .bits(0)
                .goal_pwm()
                .bits(PWM_RAW_MAX)
                .velocity_limit()
                .bits(630)
                .velocity_pgain()
                .bits(160)
                .velocity_igain()
                .bits(1640)
                .position_pgain()
                .bits(8192)
                .position_igain()
                .bits(11830)
                .position_dgain()
                .bits(32000)
                .present_position()
                .bits(0)
        });
    }

    pub fn periodic_task(&self) {
        self.led0.toggle();

        if self.dxl.ctd.read().led() == 1 {
            self.led1.on();
        } else {
            self.led1.off();
        }
    }

    pub fn control_table(&self) -> &dynamixel_f_rs::ControlTableData {
        &self.dxl.ctd
    }

    pub fn control(&mut self) {
        self.realtime_tick = self.realtime_tick.wrapping_add(1);

        let raw_position = match self.encoder.get_angle() {
            Some(position) => position,
            None => {
                self.stop_motor();
                self.encoder_tracker.reset();
                self.velocity_filter.reset();
                self.dxl.ctd.modify(|_, w| {
                    w.realtime_tick()
                        .bits(self.realtime_tick)
                        .moving()
                        .bits(0)
                        .present_pwm()
                        .bits(0)
                        .present_velocity()
                        .bits(0)
                });
                return;
            }
        };

        let (present_position, encoder_delta) = self.encoder_tracker.update(raw_position);
        let table = self.dxl.ctd.read();
        let operating_mode = table.operating_mode();
        let raw_velocity = velocity_from_encoder_delta(encoder_delta, self.control_period_seconds);
        let filtered_velocity = self.velocity_filter.update(
            raw_velocity,
            self.control_period_seconds,
            if operating_mode == OPERATING_MODE_VELOCITY {
                VELOCITY_FILTER_CUTOFF_HZ
            } else {
                POSITION_D_FILTER_CUTOFF_HZ
            },
        );
        let present_velocity = filtered_velocity as i32;
        let torque_enabled = table.torque_enable() == 1;

        if torque_enabled != self.last_torque_enabled || operating_mode != self.last_operating_mode
        {
            self.position_pid.reset();
            self.velocity_pi.reset();
        }

        let present_pwm = if !torque_enabled {
            self.stop_motor();
            0
        } else {
            match operating_mode {
                OPERATING_MODE_VELOCITY => {
                    let limit = i32::from(table.velocity_limit());
                    let goal = clamp_i32(table.goal_velocity(), -limit, limit);
                    let output = self.velocity_pi.update(
                        goal,
                        filtered_velocity,
                        table.velocity_pgain(),
                        table.velocity_igain(),
                        pwm_limit(table.pwm_limit(), table.goal_pwm()) as f32,
                        self.control_period_seconds,
                    ) as i16;
                    self.apply_pwm(output);
                    output
                }
                OPERATING_MODE_POSITION => {
                    let minimum = u32_to_i32(table.min_position_limit());
                    let maximum = u32_to_i32(table.max_position_limit());
                    let goal = clamp_i32(table.goal_position(), minimum, maximum);
                    let output_limit = pwm_limit(table.pwm_limit(), table.goal_pwm()) as f32;
                    let output = self.position_pid.update(
                        goal,
                        present_position,
                        PositionPidParameters {
                            p_gain_raw: table.position_pgain(),
                            i_gain_raw: table.position_igain(),
                            d_gain_raw: table.position_dgain(),
                            output_limit,
                            dt_seconds: self.control_period_seconds,
                            velocity_counts_per_second: filtered_velocity,
                        },
                    ) as i16;
                    self.apply_pwm(output);
                    output
                }
                OPERATING_MODE_PWM => {
                    self.position_pid.reset();
                    let limit = pwm_limit(table.pwm_limit(), PWM_RAW_MAX);
                    let output = clamp_i16(table.goal_pwm(), -limit, limit);
                    self.apply_pwm(output);
                    output
                }
                _ => {
                    self.stop_motor();
                    0
                }
            }
        };

        self.last_torque_enabled = torque_enabled;
        self.last_operating_mode = operating_mode;

        let moving = if i64::from(present_velocity).abs() > i64::from(table.moving_threshold()) {
            1
        } else {
            0
        };
        self.dxl.ctd.modify(|_, w| {
            w.realtime_tick()
                .bits(self.realtime_tick)
                .moving()
                .bits(moving)
                .present_pwm()
                .bits(present_pwm)
                .present_velocity()
                .bits(present_velocity)
                .present_position()
                .bits(present_position)
        });
    }

    pub fn enque_uart(&mut self, data: u8) {
        self.dxl.uart.enqueue(data).unwrap();
    }

    pub fn parse_uart_task(&mut self) {
        // Dxl処理(受信があった場合自動返信するはず)
        let r = self.dxl.parse_data();
        match r {
            Ok(_) => {}
            Err(_e) => {
                defmt::info!("error");
            }
        }
    }

    fn apply_pwm(&self, raw_pwm: i16) {
        let direction = if raw_pwm < 0 { -1.0 } else { 1.0 };
        let duty = i32::from(raw_pwm).abs() as f32 / PWM_RAW_MAX as f32;
        self.md.enable();
        self.md.set_pwm(direction, duty);
    }

    fn stop_motor(&mut self) {
        self.position_pid.reset();
        self.velocity_pi.reset();
        self.md.set_pwm(1.0, 0.0);
        self.md.disable();
    }
}
