use servo_pio::calibration::{AngularCalibration, Calibration};

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CalData {
    pub name: &'static str,
    pub home_pulse: f32,
    pub cal: Calibration<AngularCalibration>,
    pub typ: JointType,
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

pub fn map_float(value: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
    ((value - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min
}

// macro_rules! calib {
//     (
//         $min_pulse:literal, $min_value:literal;
//         $mid_pulse:literal, $mid_value:literal;
//         $max_pulse:literal, $max_value:literal;
//     ) => {
//         Calibration::builder(AngularCalibration::new(
//             Point {
//                 pulse: $min_pulse,
//                 value: $min_value,
//             },
//             Point {
//                 pulse: $mid_pulse,
//                 value: $mid_value,
//             },
//             Point {
//                 pulse: $max_pulse,
//                 value: $max_value,
//             },
//         ))
//         .limit_lower()
//         .limit_upper()
//         .build()
//     };
// }

// pub fn calibrations() -> [Calibration<AngularCalibration>; NUM_SERVOS] {
//     [
//         // Leg 1
//         // Tibia
//         calib!(
//             558.0, -55.0;
//             1382.0, 35.0;
//             2325.0, 125.0;
//         ),
//         // Femur
//         calib!(
//             2353.0, -90.0;
//             1375.0, 0.0;
//             450.0, 90.0; // 4436 orig
//         ),
//         // Coxa
//         calib!(
//             1106.0, 40.0;
//             1535.0, 90.0;
//             1964.0, 140.0;
//         ),
//         //
//         // Leg 2
//         // Tibia
//         calib!(
//             571.0, -55.0;
//             1475.0, 35.0;
//             2366.0, 125.0;
//         ),
//         // Femur
//         calib!(
//             2436.0, -90.0;
//             1533.0, 0.0;
//             554.0, 90.0;
//         ),
//         // Coxa
//         calib!(
//             1008.0, 40.0;
//             1397.0, 90.0;
//             1783.0, 140.0;
//         ),
//         //
//         // Leg 3
//         // Tibia
//         calib!(
//             620.0, -55.0;
//             1739.0, 35.0;
//             2442.0, 125.0;
//         ),
//         // Femur
//         calib!(
//             2467.0, -90.0;
//             1456.0, 0.0;
//             525.0, 90.0;
//         ),
//         // Coxa
//         calib!(
//             1200.0, 40.0;
//             1534.0, 90.0;
//             1797.0, 140.0;
//         ),
//         // Leg 4
//         // Tibia
//         calib!(
//             2250.0, -55.0;
//             1362.0, 35.0;
//             454.0, 125.0;
//         ),
//         // Femur
//         calib!(
//             523.0, -90.0;
//             1449.0, 0.0;
//             2399.0, 90.0;
//         ),
//         // Coxa
//         calib!(
//             1160.0, 40.0;
//             1543.0, 90.0;
//             1870.0, 140.0;
//         ),
//         //
//         // Leg 5
//         // Tibia
//         calib!(
//             2327.0, -55.0;
//             1484.0, 35.0;
//             568.0, 125.0;
//         ),
//         // Femur
//         calib!(
//             626.0, -90.0;
//             1455.0, 0.0;
//             2480.0, 90.0; // 2535 original
//         ),
//         // Coxa
//         calib!(
//             1073.0, 40.0;
//             1405.0, 90.0;
//             1760.0, 140.0;
//         ),
//         //
//         // Leg 6
//         // Tibia
//         calib!(
//             2342.0, -55.0;
//             1345.0, 35.0;
//             662.0, 125.0;
//         ),
//         // Femur
//         calib!(
//             547.0, -90.0;
//             1558.0, 0.0;
//             2470.0, 90.0;
//         ),
//         // Coxa
//         calib!(
//             1220.0, 40.0;
//             1552.0, 90.0;
//             1881.0, 140.0;
//         ),
//     ]
// }
