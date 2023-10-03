use crate::{NUM_LEGS, NUM_SERVOS_PER_LEG};
use mecha_ferris::calibrations::{CalData, JointType};
use mecha_ferris::joint::Joint;
use servo_pio::calibration::{AngularCalibration, Calibration, Point};
use servo_pio::servo_cluster::ServoIdx;

macro_rules! calib {
    // Used for calibrating. The value is the same as the pulse.
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
    // Used for motion. The values are tied to joint angles.
    (
        $joint_type:ident,
        $name:literal: $home_pulse:literal,
        $min_pulse:literal => $min_value:literal,
        $mid_pulse:literal => $mid_value:literal,
        $max_pulse:literal => $max_value:literal,
    ) => {
        (CalData {
            name: $name,
            home_pulse: $home_pulse,
            typ: JointType::$joint_type,
            cal: Calibration::builder(AngularCalibration::new(
                Point {
                    pulse: $min_pulse,
                    value: $min_value,
                },
                Point {
                    pulse: $mid_pulse,
                    value: $mid_value,
                },
                Point {
                    pulse: $max_pulse,
                    value: $max_value,
                },
            ))
            .limit_lower()
            .limit_upper()
            .build(),
        })
    };
}

pub fn calibrating_calibrations() -> [[CalData; NUM_SERVOS_PER_LEG]; NUM_LEGS] {
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

pub fn calibrations() -> [[CalData; NUM_SERVOS_PER_LEG]; NUM_LEGS] {
    [
        // Leg 1
        [
            calib!(Tibia, "X": 1045.0,
                558.0 => -55.0,
                1382.0 => 35.0,
                2325.0 => 125.0,
            ),
            calib!(Femur, "A": 1395.0,
                2353.0 => -90.0,
                1375.0 => 0.0,
                450.0 => 90.0, // 4436 original
            ),
            calib!(Coxa, "O": 1521.0,
                1106.0 => 40.0,
                1535.0 => 90.0,
                1964.0 => 140.0,
            ),
        ],
        //
        // Leg 2
        [
            calib!(Tibia, "T": 1076.0,
                571.0 => -55.0,
                1475.0 => 35.0,
                2366.0 => 125.0,
            ),
            calib!(Femur, "J": 1485.0,
                2436.0 => -90.0,
                1533.0 => 0.0,
                554.0 => 90.0,
            ),
            calib!(Coxa, "B": 1402.0,
                1008.0 => 40.0,
                1397.0 => 90.0,
                1783.0 => 140.0,
            ),
        ],
        //
        // Leg 3
        [
            calib!(Tibia, "W": 1078.0,
                620.0 => -55.0,
                1739.0 => 35.0,
                2442.0 => 125.0,
            ),
            calib!(Femur, "N": 1445.0,
                2467.0 => -90.0,
                1456.0 => 0.0,
                525.0 => 90.0,
            ),
            calib!(Coxa, "C": 1550.0,
                1200.0 => 40.0,
                1534.0 => 90.0,
                1797.0 => 140.0,
            ),
        ],
        // Leg 4
        [
            calib!(Tibia, "S": 1795.0,
                2250.0 => -55.0,
                1362.0 => 35.0,
                454.0 => 125.0,
            ),
            calib!(Femur, "Q": 1480.0,
                523.0 => -90.0,
                1449.0 => 0.0,
                2399.0 => 90.0,
            ),
            calib!(Coxa, "F": 1545.0,
                1160.0 => 40.0,
                1543.0 => 90.0,
                1870.0 => 140.0,
            ),
        ],
        //
        // Leg 5
        [
            calib!(Tibia, "U": 1850.0,
                2327.0 => -55.0,
                1484.0 => 35.0,
                568.0 => 125.0,
            ),
            calib!(Femur, "D": 1555.0,
                626.0 => -90.0,
                1455.0 => 0.0,
                2480.0 => 90.0, // 2535 original
            ),
            calib!(Coxa, "G": 1440.0,
                1073.0 => 40.0,
                1405.0 => 90.0,
                1760.0 => 140.0,
            ),
        ],
        //
        // Leg 6
        [
            calib!(Tibia, "V": 1884.0,
                2342.0 => -55.0,
                1345.0 => 35.0,
                662.0 => 125.0,
            ),
            calib!(Femur, "H": 1563.0,
                547.0 => -90.0,
                1558.0 => 0.0,
                2470.0 => 90.0,
            ),
            calib!(Coxa, "L": 1560.0,
                1220.0 => 40.0,
                1552.0 => 90.0,
                1881.0 => 140.0,
            ),
        ],
    ]
}

pub fn make_joints(
    servos: [[ServoIdx; NUM_SERVOS_PER_LEG]; NUM_LEGS],
    calibrations: [[CalData; NUM_SERVOS_PER_LEG]; NUM_LEGS],
    calibrating_calibrations: [[CalData; NUM_SERVOS_PER_LEG]; NUM_LEGS],
) -> [[Joint; NUM_SERVOS_PER_LEG]; NUM_LEGS] {
    [
        [
            Joint::new(
                servos[0][0],
                calibrations[0][0],
                calibrating_calibrations[0][0],
            ),
            Joint::new(
                servos[0][1],
                calibrations[0][1],
                calibrating_calibrations[0][1],
            ),
            Joint::new(
                servos[0][2],
                calibrations[0][2],
                calibrating_calibrations[0][2],
            ),
        ],
        [
            Joint::new(
                servos[1][0],
                calibrations[1][0],
                calibrating_calibrations[1][0],
            ),
            Joint::new(
                servos[1][1],
                calibrations[1][1],
                calibrating_calibrations[1][1],
            ),
            Joint::new(
                servos[1][2],
                calibrations[1][2],
                calibrating_calibrations[1][2],
            ),
        ],
        [
            Joint::new(
                servos[2][0],
                calibrations[2][0],
                calibrating_calibrations[2][0],
            ),
            Joint::new(
                servos[2][1],
                calibrations[2][1],
                calibrating_calibrations[2][1],
            ),
            Joint::new(
                servos[2][2],
                calibrations[2][2],
                calibrating_calibrations[2][2],
            ),
        ],
        [
            Joint::new(
                servos[3][0],
                calibrations[3][0],
                calibrating_calibrations[3][0],
            ),
            Joint::new(
                servos[3][1],
                calibrations[3][1],
                calibrating_calibrations[3][1],
            ),
            Joint::new(
                servos[3][2],
                calibrations[3][2],
                calibrating_calibrations[3][2],
            ),
        ],
        [
            Joint::new(
                servos[4][0],
                calibrations[4][0],
                calibrating_calibrations[4][0],
            ),
            Joint::new(
                servos[4][1],
                calibrations[4][1],
                calibrating_calibrations[4][1],
            ),
            Joint::new(
                servos[4][2],
                calibrations[4][2],
                calibrating_calibrations[4][2],
            ),
        ],
        [
            Joint::new(
                servos[5][0],
                calibrations[5][0],
                calibrating_calibrations[5][0],
            ),
            Joint::new(
                servos[5][1],
                calibrations[5][1],
                calibrating_calibrations[5][1],
            ),
            Joint::new(
                servos[5][2],
                calibrations[5][2],
                calibrating_calibrations[5][2],
            ),
        ],
    ]
}
