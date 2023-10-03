use crate::calibrations::CalData;
use servo_pio::servo_cluster::ServoIdx;

#[derive(Clone, Copy)]
pub struct Joint {
    /// `ServoIdx` for the joint.
    servo: ServoIdx,
    /// Calibration for when the joint is running as part of the robot.
    cal: CalData,
    /// Calibration for when the joint is being calibrated.
    calibrating: CalData,
}

impl Joint {
    pub fn new(servo: ServoIdx, cal: CalData, calibrating: CalData) -> Self {
        Self {
            servo,
            cal,
            calibrating,
        }
    }

    pub fn servo(&self) -> ServoIdx {
        self.servo
    }

    pub fn cal(&self) -> &CalData {
        &self.cal
    }

    pub fn calibrating_cal(&self) -> &CalData {
        &self.calibrating
    }

    pub fn cal_mut(&mut self) -> &mut CalData {
        &mut self.cal
    }
}
