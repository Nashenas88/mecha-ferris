use nalgebra::{Point3, Rotation3, Scale3, Translation3, Unit, Vector3};

use crate::ExpensiveMath;

const NUM_LEGS: usize = 6;
const TURNS: u8 = 5;
const TURNING_ANGLE: f32 = core::f32::consts::PI / TURNS as f32;
const RETURN_HEIGHT: f32 = 35.0;
const STEP_LENGTH: f32 = 50.0;

#[derive(Copy, Clone)]
pub struct AnimationManager {
    animation: Animation,
    steps: u8,
}

impl Default for AnimationManager {
    fn default() -> Self {
        Self::new()
    }
}

impl AnimationManager {
    pub fn new() -> Self {
        Self {
            animation: Animation::Walk(true),
            steps: 1,
        }
    }

    pub fn animation(&self) -> Animation {
        self.animation
    }

    pub fn home<T>(&mut self, walking_radius: f32, interpolation: f32) -> Targets
    where
        T: ExpensiveMath<f32>,
    {
        self.animation = Animation::Stand(4);
        self.animation.targets::<T>(walking_radius, interpolation)
    }

    fn loops(&self) -> u8 {
        match self.animation {
            Animation::Walk(_) => 5,
            Animation::Stand(_) => 1,
            Animation::Transition(_) => 1,
            Animation::RotateCW => TURNS,
            Animation::RotateCCW => TURNS,
        }
    }

    pub fn next_animation(&mut self) -> (bool, f64) {
        let reset = if self.steps >= self.loops() {
            self.animation = self.animation.next();
            self.steps = 1;
            true
        } else {
            self.steps += 1;
            false
        };

        (reset, self.animation.duration())
    }

    pub fn duration(&self) -> f64 {
        self.animation.duration()
    }

    pub fn targets<T>(&self, walking_radius: f32, time: f32) -> Targets
    where
        T: ExpensiveMath<f32>,
    {
        self.animation.targets::<T>(walking_radius, time)
    }
}

#[derive(Copy, Clone, Debug)]
pub enum Animation {
    Walk(bool),
    Stand(u8),
    Transition(u8),
    RotateCW,
    RotateCCW,
}

pub type Targets = [Point3<f32>; NUM_LEGS];

impl Animation {
    pub(crate) fn duration(&self) -> f64 {
        match self {
            Animation::Walk(_) => 2.0,
            Animation::Stand(_) => 1.0,
            Animation::Transition(_) => 0.5,
            Animation::RotateCW => 3.0,
            Animation::RotateCCW => 3.0,
        }
    }

    fn next(&self) -> Self {
        match self {
            Animation::Walk(true) => Animation::Transition(0),
            Animation::Transition(0) => Animation::Stand(1),
            Animation::Stand(1) => Animation::Transition(1),
            Animation::Transition(1) => Animation::RotateCW,
            Animation::RotateCW => Animation::Transition(2),
            Animation::Transition(2) => Animation::Stand(2),
            Animation::Stand(2) => Animation::Transition(3),
            Animation::Transition(3) => Animation::Walk(false),
            Animation::Walk(false) => Animation::Transition(4),
            Animation::Transition(4) => Animation::Stand(3),
            Animation::Stand(3) => Animation::Transition(5),
            Animation::Transition(5) => Animation::RotateCCW,
            Animation::RotateCCW => Animation::Transition(6),
            Animation::Transition(6) => Animation::Stand(4),
            Animation::Stand(4) => Animation::Transition(7),
            Animation::Transition(7) => Animation::Walk(true),
            Animation::Stand(_) | Animation::Transition(_) => unsafe {
                core::hint::unreachable_unchecked()
            },
        }
    }

    fn prev(&self) -> Self {
        match self {
            Animation::Walk(true) => Animation::Transition(7),
            Animation::Transition(0) => Animation::Walk(true),
            Animation::Stand(1) => Animation::Transition(0),
            Animation::Transition(1) => Animation::Stand(1),
            Animation::RotateCW => Animation::Transition(1),
            Animation::Transition(2) => Animation::RotateCW,
            Animation::Stand(2) => Animation::Transition(2),
            Animation::Transition(3) => Animation::Stand(2),
            Animation::Walk(false) => Animation::Transition(3),
            Animation::Transition(4) => Animation::Walk(false),
            Animation::Stand(3) => Animation::Transition(4),
            Animation::Transition(5) => Animation::Stand(3),
            Animation::RotateCCW => Animation::Transition(5),
            Animation::Transition(6) => Animation::RotateCCW,
            Animation::Stand(4) => Animation::Transition(6),
            Animation::Transition(7) => Animation::Stand(4),
            Animation::Stand(_) | Animation::Transition(_) => unsafe {
                core::hint::unreachable_unchecked()
            },
        }
    }

    pub(crate) fn targets<T>(&self, walking_radius: f32, time: f32) -> Targets
    where
        T: ExpensiveMath<f32>,
    {
        let interp_on_circle = time * core::f32::consts::TAU;
        match self {
            Animation::Walk(_) => self.walk_targets::<T>(walking_radius, interp_on_circle),
            Animation::Stand(_) => self.stand_targets::<T>(walking_radius),
            Animation::RotateCW => self.rotate_cw_targets::<T>(walking_radius, interp_on_circle),
            Animation::RotateCCW => self.rotate_ccw_targets::<T>(walking_radius, interp_on_circle),
            Animation::Transition(_) => {
                let prev = self.prev().targets::<T>(walking_radius, 1.0);
                let next = self.next().targets::<T>(walking_radius, 0.0);
                let mut targets = [Point3::<f32>::new(0.0, 0.0, 0.0); 6];
                for (target, (prev, next)) in targets
                    .iter_mut()
                    .zip(prev.into_iter().zip(next.into_iter()))
                {
                    *target = prev.coords.lerp(&next.coords, time).into();
                }
                targets
            }
        }
    }

    fn walk_targets<T>(&self, walking_radius: f32, time: f32) -> Targets
    where
        T: ExpensiveMath<f32>,
    {
        let time = (time + core::f32::consts::FRAC_PI_2) % core::f32::consts::TAU;
        let offset = (time + core::f32::consts::PI) % core::f32::consts::TAU;
        // Alternate steps dragging back then moving in a circle above the step.
        if time < core::f32::consts::PI {
            [
                point_on_circle::<T>(
                    walking_radius * T::cos(0.0_f32),
                    0.0,
                    walking_radius * T::sin(0.0_f32),
                    STEP_LENGTH,
                    time,
                ),
                point_on_line::<T>(
                    walking_radius * T::cos(core::f32::consts::FRAC_PI_3),
                    0.0,
                    walking_radius * T::sin(core::f32::consts::FRAC_PI_3),
                    STEP_LENGTH,
                    offset,
                ),
                point_on_circle::<T>(
                    walking_radius * T::cos(2.0 * core::f32::consts::FRAC_PI_3),
                    0.0,
                    walking_radius * T::sin(2.0 * core::f32::consts::FRAC_PI_3),
                    STEP_LENGTH,
                    time,
                ),
                point_on_line::<T>(
                    walking_radius * T::cos(core::f32::consts::PI),
                    0.0,
                    walking_radius * T::sin(core::f32::consts::PI),
                    STEP_LENGTH,
                    offset,
                ),
                point_on_circle::<T>(
                    walking_radius * T::cos(4.0 * core::f32::consts::FRAC_PI_3),
                    0.0,
                    walking_radius * T::sin(4.0 * core::f32::consts::FRAC_PI_3),
                    STEP_LENGTH,
                    time,
                ),
                point_on_line::<T>(
                    walking_radius * T::cos(5.0 * core::f32::consts::FRAC_PI_3),
                    0.0,
                    walking_radius * T::sin(5.0 * core::f32::consts::FRAC_PI_3),
                    STEP_LENGTH,
                    offset,
                ),
            ]
        } else {
            [
                point_on_line::<T>(
                    walking_radius * T::cos(0.0_f32),
                    0.0,
                    walking_radius * T::sin(0.0_f32),
                    STEP_LENGTH,
                    time,
                ),
                point_on_circle::<T>(
                    walking_radius * T::cos(core::f32::consts::FRAC_PI_3),
                    0.0,
                    walking_radius * T::sin(core::f32::consts::FRAC_PI_3),
                    STEP_LENGTH,
                    offset,
                ),
                point_on_line::<T>(
                    walking_radius * T::cos(2.0 * core::f32::consts::FRAC_PI_3),
                    0.0,
                    walking_radius * T::sin(2.0 * core::f32::consts::FRAC_PI_3),
                    STEP_LENGTH,
                    time,
                ),
                point_on_circle::<T>(
                    walking_radius * T::cos(core::f32::consts::PI),
                    0.0,
                    walking_radius * T::sin(core::f32::consts::PI),
                    STEP_LENGTH,
                    offset,
                ),
                point_on_line::<T>(
                    walking_radius * T::cos(4.0 * core::f32::consts::FRAC_PI_3),
                    0.0,
                    walking_radius * T::sin(4.0 * core::f32::consts::FRAC_PI_3),
                    STEP_LENGTH,
                    time,
                ),
                point_on_circle::<T>(
                    walking_radius * T::cos(5.0 * core::f32::consts::FRAC_PI_3),
                    0.0,
                    walking_radius * T::sin(5.0 * core::f32::consts::FRAC_PI_3),
                    STEP_LENGTH,
                    offset,
                ),
            ]
        }
    }

    fn stand_targets<T>(&self, walking_radius: f32) -> Targets
    where
        T: ExpensiveMath<f32>,
    {
        [
            Point3::new(
                walking_radius * T::sin(0.0_f32),
                0.0,
                walking_radius * T::cos(0.0_f32),
            ),
            Point3::new(
                walking_radius * T::sin(core::f32::consts::FRAC_PI_3),
                0.0,
                walking_radius * T::cos(core::f32::consts::FRAC_PI_3),
            ),
            Point3::new(
                walking_radius * T::sin(2.0 * core::f32::consts::FRAC_PI_3),
                0.0,
                walking_radius * T::cos(2.0 * core::f32::consts::FRAC_PI_3),
            ),
            Point3::new(
                walking_radius * T::sin(core::f32::consts::PI),
                0.0,
                walking_radius * T::cos(core::f32::consts::PI),
            ),
            Point3::new(
                walking_radius * T::sin(4.0 * core::f32::consts::FRAC_PI_3),
                0.0,
                walking_radius * T::cos(4.0 * core::f32::consts::FRAC_PI_3),
            ),
            Point3::new(
                walking_radius * T::sin(5.0 * core::f32::consts::FRAC_PI_3),
                0.0,
                walking_radius * T::cos(5.0 * core::f32::consts::FRAC_PI_3),
            ),
        ]
    }

    fn rotate_cw_targets<T>(&self, walking_radius: f32, time: f32) -> Targets
    where
        T: ExpensiveMath<f32>,
    {
        let time = (time + core::f32::consts::FRAC_PI_2) % core::f32::consts::TAU;
        // Alternate steps dragging back then moving in a circle above the step.
        if time < core::f32::consts::PI {
            [
                return_from_arc::<T>(walking_radius, 0.0, time),
                point_on_arc::<T>(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    core::f32::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    core::f32::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    time,
                ),
                return_from_arc::<T>(walking_radius, 2.0 * core::f32::consts::FRAC_PI_3, time),
                point_on_arc::<T>(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    3.0 * core::f32::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    3.0 * core::f32::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    time,
                ),
                return_from_arc::<T>(walking_radius, 4.0 * core::f32::consts::FRAC_PI_3, time),
                point_on_arc::<T>(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    5.0 * core::f32::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    5.0 * core::f32::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    time,
                ),
            ]
        } else {
            [
                point_on_arc::<T>(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    -TURNING_ANGLE / 2.0,
                    TURNING_ANGLE / 2.0,
                    time - core::f32::consts::PI,
                ),
                return_from_arc::<T>(
                    walking_radius,
                    core::f32::consts::FRAC_PI_3,
                    time - core::f32::consts::PI,
                ),
                point_on_arc::<T>(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    2.0 * core::f32::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    2.0 * core::f32::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    time - core::f32::consts::PI,
                ),
                return_from_arc::<T>(
                    walking_radius,
                    core::f32::consts::PI,
                    time - core::f32::consts::PI,
                ),
                point_on_arc::<T>(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    4.0 * core::f32::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    4.0 * core::f32::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    time - core::f32::consts::PI,
                ),
                return_from_arc::<T>(
                    walking_radius,
                    5.0 * core::f32::consts::FRAC_PI_3,
                    time - core::f32::consts::PI,
                ),
            ]
        }
    }

    fn rotate_ccw_targets<T>(&self, walking_radius: f32, time: f32) -> Targets
    where
        T: ExpensiveMath<f32>,
    {
        let time = (time + core::f32::consts::FRAC_PI_2) % core::f32::consts::TAU;
        // Alternate steps dragging back then moving in a circle above the step.
        if time < core::f32::consts::PI {
            [
                return_from_arc::<T>(walking_radius, 0.0, core::f32::consts::PI - time),
                point_on_arc::<T>(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    core::f32::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    core::f32::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    time,
                ),
                return_from_arc::<T>(
                    walking_radius,
                    2.0 * core::f32::consts::FRAC_PI_3,
                    core::f32::consts::PI - time,
                ),
                point_on_arc::<T>(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    3.0 * core::f32::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    3.0 * core::f32::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    time,
                ),
                return_from_arc::<T>(
                    walking_radius,
                    4.0 * core::f32::consts::FRAC_PI_3,
                    core::f32::consts::PI - time,
                ),
                point_on_arc::<T>(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    5.0 * core::f32::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    5.0 * core::f32::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    time,
                ),
            ]
        } else {
            [
                point_on_arc::<T>(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    TURNING_ANGLE / 2.0,
                    -TURNING_ANGLE / 2.0,
                    time - core::f32::consts::PI,
                ),
                return_from_arc::<T>(
                    walking_radius,
                    core::f32::consts::FRAC_PI_3,
                    core::f32::consts::TAU - time,
                ),
                point_on_arc::<T>(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    2.0 * core::f32::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    2.0 * core::f32::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    time - core::f32::consts::PI,
                ),
                return_from_arc::<T>(
                    walking_radius,
                    core::f32::consts::PI,
                    core::f32::consts::TAU - time,
                ),
                point_on_arc::<T>(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    4.0 * core::f32::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    4.0 * core::f32::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    time - core::f32::consts::PI,
                ),
                return_from_arc::<T>(
                    walking_radius,
                    5.0 * core::f32::consts::FRAC_PI_3,
                    core::f32::consts::TAU - time,
                ),
            ]
        }
    }
}

fn point_on_circle<T>(z: f32, y: f32, x: f32, radius: f32, angle: f32) -> Point3<f32>
where
    T: ExpensiveMath<f32>,
{
    Point3::new(
        x,
        y + RETURN_HEIGHT * T::sin(angle),
        z + radius * T::cos(angle),
    )
}

fn return_from_arc<T>(radius: f32, arc_angle: f32, interpolation: f32) -> Point3<f32>
where
    T: ExpensiveMath<f32>,
{
    let path_radius = radius * T::sin(TURNING_ANGLE / 2.0);
    let point = Point3::new(
        0.0,
        path_radius * T::sin(interpolation),
        path_radius * T::cos(interpolation),
    )
    .rotate(&Vector3::y_axis(), core::f32::consts::FRAC_PI_2);
    Translation3::new(0.0, 0.0, radius * T::cos(TURNING_ANGLE / 2.0))
        .transform_point(&point)
        .rotate(&Vector3::y_axis(), arc_angle)
        .scale(1.0, RETURN_HEIGHT / path_radius, 1.0)
}

fn point_on_arc<T>(
    z: f32,
    y: f32,
    x: f32,
    radius: f32,
    arc_start: f32,
    arc_end: f32,
    interpolation: f32,
) -> Point3<f32>
where
    T: ExpensiveMath<f32>,
{
    let interpolation = interpolation / core::f32::consts::PI;
    let angle = arc_start * (1.0 - interpolation) + arc_end * interpolation;
    Point3::new(x + radius * T::sin(angle), y, z + radius * T::cos(angle))
}

fn point_on_line<T>(z: f32, y: f32, x: f32, length: f32, t: f32) -> Point3<f32>
where
    T: ExpensiveMath<f32>,
{
    Point3::new(x, y, z + length * T::cos(t))
}

trait PointExt {
    fn scale(&self, x: f32, y: f32, z: f32) -> Self;
    fn rotate(&self, axis: &Unit<Vector3<f32>>, rad: f32) -> Self;
}

impl PointExt for Point3<f32> {
    fn scale(&self, x: f32, y: f32, z: f32) -> Self {
        Scale3::new(x, y, z).transform_point(self)
    }

    fn rotate(&self, axis: &Unit<Vector3<f32>>, rad: f32) -> Self {
        Rotation3::from_axis_angle(axis, rad).transform_point(self)
    }
}
