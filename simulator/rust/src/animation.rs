use kinematics::{Point3, Rotation3, Scale3, Translation3, Unit, Vector3};

#[derive(Copy, Clone)]
pub(crate) struct AnimationManager {
    animation: Animation,
    steps: u8,
}

impl AnimationManager {
    pub(crate) fn new() -> Self {
        Self {
            animation: Animation::Walk(true),
            steps: 0,
        }
    }

    pub(crate) fn next(&mut self) -> (bool, f64) {
        let reset = if self.steps
            >= match self.animation {
                Animation::Walk(_) => 10,
                Animation::Stand(_) => 1,
                Animation::RotateCW => 4,
                Animation::RotateCCW => 4,
            } {
            self.animation.next();
            self.steps = 0;
            true
        } else {
            self.steps += 1;
            false
        };

        (reset, self.animation.duration())
    }

    pub(crate) fn duration(&self) -> f64 {
        self.animation.duration()
    }

    pub(crate) fn targets(&self, walking_radius: f32, time: f64) -> Targets {
        self.animation.targets(walking_radius, time)
    }
}

#[derive(Copy, Clone)]
pub(crate) enum Animation {
    Walk(bool),
    Stand(u8),
    RotateCW,
    RotateCCW,
}

type Targets = [Point3<f32>; 6];

const TURNING_ANGLE: f64 = core::f64::consts::FRAC_PI_4;
const RETURN_SCALE: f32 = 0.1;
const STEP_LENGTH: f32 = 50.0;

impl Animation {
    pub(crate) fn duration(&self) -> f64 {
        match self {
            Animation::Walk(_) => 2.0,
            Animation::Stand(_) => 0.5,
            Animation::RotateCW => 5.0,
            Animation::RotateCCW => 5.0,
        }
    }

    pub(crate) fn next(&mut self) {
        *self = match *self {
            Animation::Walk(true) => Animation::Stand(1),
            Animation::Stand(1) => Animation::RotateCW,
            Animation::RotateCW => Animation::Stand(2),
            Animation::Stand(2) => Animation::Walk(false),
            Animation::Walk(false) => Animation::Stand(3),
            Animation::Stand(3) => Animation::RotateCCW,
            Animation::RotateCCW => Animation::Stand(4),
            Animation::Stand(4) => Animation::Walk(true),
            Animation::Stand(_) => unsafe { core::hint::unreachable_unchecked() },
        };
    }

    pub(crate) fn targets(&self, walking_radius: f32, time: f64) -> Targets {
        match self {
            Animation::Walk(_) => self.walk_targets(walking_radius, time),
            Animation::Stand(_) => self.stand_targets(walking_radius),
            Animation::RotateCW => self.rotate_cw_targets(walking_radius, time),
            Animation::RotateCCW => self.rotate_ccw_targets(walking_radius, time),
        }
    }

    fn walk_targets(&self, walking_radius: f32, time: f64) -> Targets {
        let time = (time + core::f64::consts::FRAC_PI_2) % core::f64::consts::TAU;
        let offset = (time + core::f64::consts::PI) % core::f64::consts::TAU;
        // Alternate steps dragging back then moving in a circle above the step.
        if time < core::f64::consts::PI {
            [
                point_on_circle(
                    walking_radius * 0.0_f32.cos(),
                    0.0,
                    walking_radius * 0.0_f32.sin(),
                    STEP_LENGTH,
                    time,
                ),
                point_on_line(
                    walking_radius * std::f32::consts::FRAC_PI_3.cos(),
                    0.0,
                    walking_radius * std::f32::consts::FRAC_PI_3.sin(),
                    STEP_LENGTH,
                    offset,
                ),
                point_on_circle(
                    walking_radius * (2.0 * std::f32::consts::FRAC_PI_3).cos(),
                    0.0,
                    walking_radius * (2.0 * std::f32::consts::FRAC_PI_3).sin(),
                    STEP_LENGTH,
                    time,
                ),
                point_on_line(
                    walking_radius * std::f32::consts::PI.cos(),
                    0.0,
                    walking_radius * std::f32::consts::PI.sin(),
                    STEP_LENGTH,
                    offset,
                ),
                point_on_circle(
                    walking_radius * (4.0 * std::f32::consts::FRAC_PI_3).cos(),
                    0.0,
                    walking_radius * (4.0 * std::f32::consts::FRAC_PI_3).sin(),
                    STEP_LENGTH,
                    time,
                ),
                point_on_line(
                    walking_radius * (5.0 * std::f32::consts::FRAC_PI_3).cos(),
                    0.0,
                    walking_radius * (5.0 * std::f32::consts::FRAC_PI_3).sin(),
                    STEP_LENGTH,
                    offset,
                ),
            ]
        } else {
            [
                point_on_line(
                    walking_radius * 0.0_f32.cos(),
                    0.0,
                    walking_radius * 0.0_f32.sin(),
                    STEP_LENGTH,
                    time,
                ),
                point_on_circle(
                    walking_radius * std::f32::consts::FRAC_PI_3.cos(),
                    0.0,
                    walking_radius * std::f32::consts::FRAC_PI_3.sin(),
                    STEP_LENGTH,
                    offset,
                ),
                point_on_line(
                    walking_radius * (2.0 * std::f32::consts::FRAC_PI_3).cos(),
                    0.0,
                    walking_radius * (2.0 * std::f32::consts::FRAC_PI_3).sin(),
                    STEP_LENGTH,
                    time,
                ),
                point_on_circle(
                    walking_radius * std::f32::consts::PI.cos(),
                    0.0,
                    walking_radius * std::f32::consts::PI.sin(),
                    STEP_LENGTH,
                    offset,
                ),
                point_on_line(
                    walking_radius * (4.0 * std::f32::consts::FRAC_PI_3).cos(),
                    0.0,
                    walking_radius * (4.0 * std::f32::consts::FRAC_PI_3).sin(),
                    STEP_LENGTH,
                    time,
                ),
                point_on_circle(
                    walking_radius * (5.0 * std::f32::consts::FRAC_PI_3).cos(),
                    0.0,
                    walking_radius * (5.0 * std::f32::consts::FRAC_PI_3).sin(),
                    STEP_LENGTH,
                    offset,
                ),
            ]
        }
    }

    fn stand_targets(&self, walking_radius: f32) -> Targets {
        [
            Point3::new(
                walking_radius * 0.0_f32.sin(),
                0.0,
                walking_radius * 0.0_f32.cos(),
            ),
            Point3::new(
                walking_radius * std::f32::consts::FRAC_PI_3.sin(),
                0.0,
                walking_radius * std::f32::consts::FRAC_PI_3.cos(),
            ),
            Point3::new(
                walking_radius * (2.0 * std::f32::consts::FRAC_PI_3).sin(),
                0.0,
                walking_radius * (2.0 * std::f32::consts::FRAC_PI_3).cos(),
            ),
            Point3::new(
                walking_radius * std::f32::consts::PI.sin(),
                0.0,
                walking_radius * std::f32::consts::PI.cos(),
            ),
            Point3::new(
                walking_radius * (4.0 * std::f32::consts::FRAC_PI_3).sin(),
                0.0,
                walking_radius * (4.0 * std::f32::consts::FRAC_PI_3).cos(),
            ),
            Point3::new(
                walking_radius * (5.0 * std::f32::consts::FRAC_PI_3).sin(),
                0.0,
                walking_radius * (5.0 * std::f32::consts::FRAC_PI_3).cos(),
            ),
        ]
    }

    fn rotate_cw_targets(&self, walking_radius: f32, time: f64) -> Targets {
        let time = (time + core::f64::consts::FRAC_PI_2) % core::f64::consts::TAU;
        // Alternate steps dragging back then moving in a circle above the step.
        if time < core::f64::consts::PI {
            [
                return_from_arc(walking_radius, 0.0, time),
                point_on_arc(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    std::f64::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    std::f64::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    time,
                ),
                return_from_arc(walking_radius, 2.0 * std::f32::consts::FRAC_PI_3, time),
                point_on_arc(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    3.0 * std::f64::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    3.0 * std::f64::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    time,
                ),
                return_from_arc(walking_radius, 4.0 * std::f32::consts::FRAC_PI_3, time),
                point_on_arc(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    5.0 * std::f64::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    5.0 * std::f64::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    time,
                ),
            ]
        } else {
            [
                point_on_arc(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    -TURNING_ANGLE / 2.0,
                    TURNING_ANGLE / 2.0,
                    time - core::f64::consts::PI,
                ),
                return_from_arc(
                    walking_radius,
                    std::f32::consts::FRAC_PI_3,
                    time - core::f64::consts::PI,
                ),
                point_on_arc(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    2.0 * std::f64::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    2.0 * std::f64::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    time - core::f64::consts::PI,
                ),
                return_from_arc(
                    walking_radius,
                    std::f32::consts::PI,
                    time - core::f64::consts::PI,
                ),
                point_on_arc(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    4.0 * std::f64::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    4.0 * std::f64::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    time - core::f64::consts::PI,
                ),
                return_from_arc(
                    walking_radius,
                    5.0 * std::f32::consts::FRAC_PI_3,
                    time - core::f64::consts::PI,
                ),
            ]
        }
    }

    fn rotate_ccw_targets(&self, walking_radius: f32, time: f64) -> Targets {
        let time = (time + core::f64::consts::FRAC_PI_2) % core::f64::consts::TAU;
        // Alternate steps dragging back then moving in a circle above the step.
        if time < core::f64::consts::PI {
            [
                return_from_arc(walking_radius, 0.0, std::f64::consts::PI - time),
                point_on_arc(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    std::f64::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    std::f64::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    time,
                ),
                return_from_arc(
                    walking_radius,
                    2.0 * std::f32::consts::FRAC_PI_3,
                    std::f64::consts::PI - time,
                ),
                point_on_arc(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    3.0 * std::f64::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    3.0 * std::f64::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    time,
                ),
                return_from_arc(
                    walking_radius,
                    4.0 * std::f32::consts::FRAC_PI_3,
                    std::f64::consts::PI - time,
                ),
                point_on_arc(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    5.0 * std::f64::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    5.0 * std::f64::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    time,
                ),
            ]
        } else {
            [
                point_on_arc(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    TURNING_ANGLE / 2.0,
                    -TURNING_ANGLE / 2.0,
                    time - core::f64::consts::PI,
                ),
                return_from_arc(
                    walking_radius,
                    std::f32::consts::FRAC_PI_3,
                    std::f64::consts::TAU - time,
                ),
                point_on_arc(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    2.0 * std::f64::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    2.0 * std::f64::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    time - core::f64::consts::PI,
                ),
                return_from_arc(
                    walking_radius,
                    std::f32::consts::PI,
                    std::f64::consts::TAU - time,
                ),
                point_on_arc(
                    0.0,
                    0.0,
                    0.0,
                    walking_radius,
                    4.0 * std::f64::consts::FRAC_PI_3 + TURNING_ANGLE / 2.0,
                    4.0 * std::f64::consts::FRAC_PI_3 - TURNING_ANGLE / 2.0,
                    time - core::f64::consts::PI,
                ),
                return_from_arc(
                    walking_radius,
                    5.0 * std::f32::consts::FRAC_PI_3,
                    std::f64::consts::TAU - time,
                ),
            ]
        }
    }
}

fn point_on_circle(z: f32, y: f32, x: f32, radius: f32, angle: f64) -> Point3<f32> {
    Point3::new(
        x,
        y + radius * angle.sin() as f32,
        z + radius * angle.cos() as f32,
    )
}

fn return_from_arc(radius: f32, arc_angle: f32, interpolation: f64) -> Point3<f32> {
    let path_radius = radius as f64 * (TURNING_ANGLE / 2.0).sin();
    let point = Point3::new(
        0.0,
        (path_radius * interpolation.sin()) as f32,
        (path_radius * interpolation.cos()) as f32,
    )
    .rotate(&Vector3::y_axis(), std::f32::consts::FRAC_PI_2);
    Translation3::new(0.0, 0.0, radius * (TURNING_ANGLE as f32 / 2.0).cos())
        .transform_point(&point)
        .rotate(&Vector3::y_axis(), arc_angle)
        .scale(1.0, RETURN_SCALE, 1.0)
}

fn point_on_arc(
    z: f32,
    y: f32,
    x: f32,
    radius: f32,
    arc_start: f64,
    arc_end: f64,
    interpolation: f64,
) -> Point3<f32> {
    let interpolation = interpolation / core::f64::consts::PI;
    let angle = arc_start * (1.0 - interpolation) + arc_end * interpolation;
    Point3::new(
        x + radius * angle.sin() as f32,
        y,
        z + radius * angle.cos() as f32,
    )
}

fn point_on_line(z: f32, y: f32, x: f32, length: f32, t: f64) -> Point3<f32> {
    Point3::new(x, y, z + length * t.cos() as f32)
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
