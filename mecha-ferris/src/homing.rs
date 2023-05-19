use crate::{NUM_LEGS, NUM_SERVOS_PER_LEG};
use mecha_ferris::joint::Joint;
use pimoroni_servo2040::hal::pio::{PIOExt, StateMachineIndex};
use pimoroni_servo2040::NUM_SERVOS;
use servo_pio::calibration::AngularCalibration;
use servo_pio::servo_cluster::ServoCluster;

pub(crate) struct Homing {
    num_enabled: u8,
}

impl Homing {
    pub(crate) fn new() -> Self {
        Self { num_enabled: 0 }
    }

    pub(crate) fn update<P, SM>(
        &mut self,
        diff: u64,
        servo_cluster: &mut ServoCluster<{ NUM_SERVOS as usize }, P, SM, AngularCalibration>,
        joints: &[[Joint; NUM_SERVOS_PER_LEG]; NUM_LEGS],
    ) -> bool
    where
        P: PIOExt,
        SM: StateMachineIndex,
    {
        if self.num_enabled == NUM_SERVOS {
            false
        } else if diff > 500 {
            // Enable servo every 500ms.
            let leg = (self.num_enabled / 3) as usize;
            let joint = (self.num_enabled % 3) as usize;
            let joint = &joints[leg][joint];
            servo_cluster.set_pulse(joint.servo(), joint.cal().home_pulse, false);
            servo_cluster.set_enabled(joint.servo(), true, true);
            self.num_enabled += 1;
            true
        } else {
            false
        }
    }

    pub(crate) fn reset(&mut self) {
        self.num_enabled = 0;
    }
}
