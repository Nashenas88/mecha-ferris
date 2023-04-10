use servo_pio::calibration::{AngularCalibration, Calibration, Point};

use crate::NUM_SERVOS;

macro_rules! calib {
    (
        $min_pulse:literal, $min_value:literal;
        $mid_pulse:literal, $mid_value:literal;
        $max_pulse:literal, $max_value:literal;
    ) => {
        Calibration::builder(AngularCalibration::new(
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
        .build()
    };
}

pub fn calibrations() -> [Calibration<AngularCalibration>; NUM_SERVOS] {
    [
        // Leg 1
        // Tibia
        calib!(
            558.0, -55.0;
            1382.0, 35.0;
            2325.0, 125.0;
        ),
        // Femur
        calib!(
            2353.0, -90.0;
            1375.0, 0.0;
            450.0, 90.0; // 4436 orig
        ),
        // Coxa
        calib!(
            1106.0, 40.0;
            1535.0, 90.0;
            1964.0, 140.0;
        ),
        //
        // Leg 2
        // Tibia
        calib!(
            571.0, -55.0;
            1475.0, 35.0;
            2366.0, 125.0;
        ),
        // Femur
        calib!(
            2436.0, -90.0;
            1533.0, 0.0;
            554.0, 90.0;
        ),
        // Coxa
        calib!(
            1008.0, 40.0;
            1397.0, 90.0;
            1783.0, 140.0;
        ),
        //
        // Leg 3
        // Tibia
        calib!(
            620.0, -55.0;
            1739.0, 35.0;
            2442.0, 125.0;
        ),
        // Femur
        calib!(
            2467.0, -90.0;
            1456.0, 0.0;
            525.0, 90.0;
        ),
        // Coxa
        calib!(
            1200.0, 40.0;
            1534.0, 90.0;
            1797.0, 140.0;
        ),
        // Leg 4
        // Tibia
        calib!(
            2250.0, -55.0;
            1362.0, 35.0;
            454.0, 125.0;
        ),
        // Femur
        calib!(
            523.0, -90.0;
            1449.0, 0.0;
            2399.0, 90.0;
        ),
        // Coxa
        calib!(
            1160.0, 40.0;
            1543.0, 90.0;
            1870.0, 140.0;
        ),
        //
        // Leg 5
        // Tibia
        calib!(
            2327.0, -55.0;
            1484.0, 35.0;
            568.0, 125.0;
        ),
        // Femur
        calib!(
            626.0, -90.0;
            1455.0, 0.0;
            2480.0, 90.0; // 2535 original
        ),
        // Coxa
        calib!(
            1073.0, 40.0;
            1405.0, 90.0;
            1760.0, 140.0;
        ),
        //
        // Leg 6
        // Tibia
        calib!(
            2342.0, -55.0;
            1345.0, 35.0;
            662.0, 125.0;
        ),
        // Femur
        calib!(
            547.0, -90.0;
            1558.0, 0.0;
            2470.0, 90.0;
        ),
        // Coxa
        calib!(
            1220.0, 40.0;
            1552.0, 90.0;
            1881.0, 140.0;
        ),
    ]
}
