#![no_std]

use core::marker::PhantomData;
use joint::Joint;
use pimoroni_servo2040::hal::pio::SM0;
use pimoroni_servo2040::pac::PIO0;
use pimoroni_servo2040::NUM_SERVOS;
use servo_pio::calibration::AngularCalibration;
use servo_pio::servo_cluster::ServoCluster;
use state::{RobotState, StateMachine, Translation3, UnitQuaternion, Vector3};
pub mod analog;
pub mod analog_mux;
pub mod calibrations;
pub mod calibrator;
pub mod comms;
pub mod debouncer;
pub mod flexible_input;
pub mod joint;
pub mod log;

const DEFAULT_ANIM_TIME: u32 = 1000;
const CALIB_ANIM_TIME: u32 = 100;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CalKind {
    Min,
    Mid,
    Max,
    Home,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CalibState {
    pub leg: usize,
    pub joint: usize,
    pub cal_kind: CalKind,
    pub anim: Anim<f32, Bezier, CALIB_ANIM_TIME>,
}

pub struct State<const NUM_SERVOS_PER_LEG: usize, const NUM_LEGS: usize> {
    pub live_state: RobotState<Joint, NUM_SERVOS_PER_LEG, NUM_LEGS>,
    pub translation_anim: Option<Anim<Translation3<f32>, Bezier, DEFAULT_ANIM_TIME>>,
    pub rotation_anim: Option<Anim<UnitQuaternion<f32>, Bezier, DEFAULT_ANIM_TIME>>,
    pub leg_radius_anim: Option<Anim<f32, Bezier, DEFAULT_ANIM_TIME>>,
    pub calib_update: Option<CalibState>,
    pub from_previous_state: Option<StateMachine>,
}

impl<const NUM_SERVOS_PER_LEG: usize, const NUM_LEGS: usize> State<NUM_SERVOS_PER_LEG, NUM_LEGS> {
    pub fn new(robot_state: RobotState<Joint, NUM_SERVOS_PER_LEG, NUM_LEGS>) -> Self {
        Self {
            live_state: robot_state,
            translation_anim: None,
            rotation_anim: None,
            leg_radius_anim: None,
            calib_update: None,
            from_previous_state: None,
        }
    }

    pub fn update(
        &mut self,
        delta: f32,
        servo_cluster: &mut ServoCluster<{ NUM_SERVOS as usize }, PIO0, SM0, AngularCalibration>,
    ) {
        if let Some(ref mut anim) = self.translation_anim {
            self.live_state.body_translation = anim.proceed(delta);
            if anim.finished() {
                self.translation_anim = None;
            }
        }
        if let Some(ref mut anim) = self.rotation_anim {
            self.live_state.body_rotation = anim.proceed(delta);
            if anim.finished() {
                self.rotation_anim = None;
            }
        }
        if let Some(ref mut anim) = self.leg_radius_anim {
            self.live_state.leg_radius = anim.proceed(delta);
            if anim.finished() {
                self.leg_radius_anim = None;
            }
        }
        if let (Some(cal_update), joints) =
            (self.calib_update.as_mut(), &mut self.live_state.joints)
        {
            let joint = &mut joints[cal_update.leg][cal_update.joint];
            let calibration = joint.cal_mut();
            let pulse = cal_update.anim.proceed(delta);
            log::info!(
                "Proceeded by {} to {} of {}",
                delta,
                cal_update.anim.elapsed(),
                cal_update.anim.duration()
            );
            match cal_update.cal_kind {
                CalKind::Min => {
                    calibration.cal.inner_mut().set_min_pulse(pulse);
                }
                CalKind::Mid => {
                    calibration.cal.inner_mut().set_mid_pulse(pulse);
                }
                CalKind::Max => {
                    calibration.cal.inner_mut().set_max_pulse(pulse);
                }
                CalKind::Home => {
                    calibration.home_pulse = pulse;
                }
            }
            log::info!("Animating calibration {:?} to {}", cal_update, pulse);
            *servo_cluster.calibration_mut(joint.servo()) = joint.calibrating_cal().cal;
            servo_cluster.set_pulse(joint.servo(), pulse, true);
            if cal_update.anim.finished() {
                self.calib_update = None;
            }
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Anim<T, E, const UDURATION_MS: u32> {
    start: T,
    end: T,
    elapsed_ms: f32,
    _phantom: PhantomData<E>,
}

pub trait Interp {
    fn interp(&self, other: &Self, t: f32) -> Self;
}

impl<T, E, const UDURATION_MS: u32> Anim<T, E, UDURATION_MS>
where
    T: Interp,
    E: Ease,
{
    const DURATION_MS: f32 = UDURATION_MS as f32;
    fn new(start: T, end: T) -> Self {
        Self {
            start,
            end,
            elapsed_ms: 0.0,
            _phantom: PhantomData,
        }
    }

    pub fn elapsed(&self) -> f32 {
        self.elapsed_ms
    }

    pub const fn duration(&self) -> f32 {
        Self::DURATION_MS
    }

    pub fn proceed(&mut self, t: f32) -> T {
        self.elapsed_ms = (self.elapsed_ms + t).min(Self::DURATION_MS);
        self.start.interp(
            &self.end,
            <E as Ease>::ease(self.elapsed_ms / Self::DURATION_MS),
        )
    }

    pub fn finished(&self) -> bool {
        self.elapsed_ms == Self::DURATION_MS
    }
}

impl Interp for f32 {
    fn interp(&self, other: &Self, t: f32) -> Self {
        self + (other - self) * t
    }
}

impl Interp for Vector3<f32> {
    fn interp(&self, other: &Self, t: f32) -> Self {
        self.lerp(other, t)
    }
}

impl Interp for Translation3<f32> {
    fn interp(&self, other: &Self, t: f32) -> Self {
        self.vector.lerp(&other.vector, t).into()
    }
}

impl Interp for UnitQuaternion<f32> {
    fn interp(&self, other: &Self, t: f32) -> Self {
        self.slerp(other, t)
    }
}

pub trait Ease {
    fn ease(t: f32) -> f32;
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Bezier;

impl Ease for Bezier {
    fn ease(t: f32) -> f32 {
        t * t * (3.0 - 2.0 * t)
    }
}
