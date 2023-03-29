use defmt::Format;
use servo_pio::calibration::{AngularCalibration, Calibration, Point};

use crate::{NUM_LEGS, NUM_SERVOS_PER_LEG};

#[derive(Format)]
pub struct CalData {
    pub name: &'static str,
    pub home_pulse: f32,
    pub cal: Calibration<AngularCalibration>,
    pub typ: JointType,
}

#[derive(Format, Debug)]
pub enum JointType {
    Tibia,
    Femur,
    Coxa,
}

impl JointType {
    pub fn min_angle(&self) -> f32 {
        match self {
            JointType::Tibia => -55.0,
            JointType::Femur => 0.0,
            JointType::Coxa => 40.0,
        }
    }

    pub fn mid_angle(&self) -> f32 {
        (self.min_angle() + self.max_angle()) / 2.0
    }

    pub fn max_angle(&self) -> f32 {
        match self {
            JointType::Tibia => 125.0,
            JointType::Femur => 180.0,
            JointType::Coxa => 140.0,
        }
    }

    pub fn home_angle(&self) -> f32 {
        match self {
            JointType::Tibia => 0.0,
            JointType::Femur => self.mid_angle(),
            JointType::Coxa => self.mid_angle(),
        }
    }
}

macro_rules! calib {
    (
        $joint_type:ident,
        $name:literal: $home_pulse:literal,
        $min_pulse:literal,
        $mid_pulse:literal,
        $max_pulse:literal,
    ) => {
        (CalData {
            name: $name,
            home_pulse: $home_pulse,
            typ: JointType::$joint_type,
            cal: Calibration::builder(AngularCalibration::new(
                Point {
                    pulse: $min_pulse,
                    value: $min_pulse,
                },
                Point {
                    pulse: $mid_pulse,
                    value: $mid_pulse,
                },
                Point {
                    pulse: $max_pulse,
                    value: $max_pulse,
                },
            ))
            .limit_lower()
            .limit_upper()
            .build(),
        })
    };
}

pub fn calibrations() -> [[CalData; NUM_SERVOS_PER_LEG]; NUM_LEGS] {
    [
        // Leg 1
        [
            calib!(Tibia, "X": 1045.0, 450.0, 1492.5, 2535.0,),
            calib!(Femur, "A": 1395.0, 448.0, 1485.5, 2523.0,),
            calib!(Coxa, "O": 1521.0, 700.0, 1498.0, 2296.0,),
        ],
        // Leg 2
        [
            calib!(Tibia, "T": 1076.0, 450.0, 1447.5, 2445.0,),
            calib!(Femur, "J": 1485.0, 445.0, 1487.5, 2530.0,),
            calib!(Coxa, "B": 1402.0, 710.0, 1506.0, 2302.0,),
        ],
        // Leg 3
        [
            calib!(Tibia, "W": 1078.0, 450.0, 1493.5, 2537.0,),
            calib!(Femur, "N": 1445.0, 450.0, 1482.5, 2515.0,),
            calib!(Coxa, "C": 1550.0, 709.0, 1494.5, 2280.0,),
        ],
        // Leg 4
        [
            calib!(Tibia, "S": 1795.0, 450.0, 1495.0, 2540.0,),
            calib!(Femur, "Q": 1480.0, 450.0, 1487.5, 2525.0,),
            calib!(Coxa, "F": 1545.0, 710.0, 1505.5, 2300.0,),
        ],
        // Leg 5
        [
            calib!(Tibia, "U": 1850.0, 450.0, 1492.5, 2535.0,),
            calib!(Femur, "D": 1555.0, 440.0, 1475.0, 2510.0,),
            calib!(Coxa, "G": 1440.0, 710.0, 1510.0, 2310.0,),
        ],
        // Leg 6
        [
            calib!(Tibia, "V": 1884.0, 450.0, 1495.0, 2550.0,),
            calib!(Femur, "H": 1563.0, 450.0, 1492.5, 2535.0,),
            calib!(Coxa, "L": 1560.0, 705.0, 1510.0, 2315.0,),
        ],
    ]
}

pub fn map_float(value: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
    ((value - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min
}
