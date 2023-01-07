use servo_pio::calibration::{AngularCalibration, Calibration, Point};

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

pub fn calibrations() -> [Calibration<AngularCalibration>; 9] {
    [
        // Leg 1
        // Tibia
        calib!(
            768.0, 0.0;
            1476.0, 67.5;
            2184.0, 135.0;
        ),
        // Femur
        calib!(
            2167.0, -90.0;
            1478.0, -22.5;
            788.0, 45.0;
        ),
        // Coxa
        calib!(
            2285.0, 20.0;
            1563.0, 90.0;
            846.0, 160.0;
        ),
        //
        // Leg 2
        // Tibia
        calib!(
            780.0, 0.0;
            1220.0, 67.5;
            2110.0, 157.5;
        ),
        // Femur
        calib!(
            2145.0, -90.0;
            1468.0, -22.5;
            791.0, 45.0;
        ),
        // Coxa
        // calib!(
        //     2423.0, 0.0;
        //     1538, 90.0;
        //     900.0, 180.0;
        // ),
        calib!(
            2283.0, 20.0;
            1551.0, 90.0;
            820.0, 170.0;
        ),
        //
        // Leg 3
        // Tibia
        calib!(
            783.0, 0.0;
            1459.0, 67.5;
            2135.0, 135.0;
        ),
        // Femur
        calib!(
            2240.0, -90.0;
            1553.0, -22.5;
            866.0, 45.0;
        ),
        // Coxa
        calib!(
            2091.0, 20.0;
            1403.0, 90.0;
            753.0, 170.0;
        ),
    ]
}
