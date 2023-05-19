use servo_pio::servo_cluster::ServoIdx;

use crate::calibrations::CalData;

pub struct Joint {
    servo: ServoIdx,
    cal: CalData,
}

impl Joint {
    pub fn new(servo: ServoIdx, cal: CalData) -> Self {
        Self { servo, cal }
    }

    pub fn servo(&self) -> ServoIdx {
        self.servo
    }

    pub fn cal(&self) -> &CalData {
        &self.cal
    }

    pub fn cal_mut(&mut self) -> &mut CalData {
        &mut self.cal
    }
}
