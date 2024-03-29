use crate::{NUM_LEGS, NUM_SERVOS_PER_LEG};
use mecha_ferris::joint::Joint;
use pimoroni_servo2040::hal::pio::{PIOExt, StateMachineIndex};
use pimoroni_servo2040::NUM_SERVOS;
use servo_pio::calibration::AngularCalibration;
use servo_pio::servo_cluster::ServoCluster;

pub(crate) struct Homing {
    last_diff: u64,
    num_enabled: u8,
}

impl Homing {
    pub(crate) const fn new() -> Self {
        Self {
            last_diff: 0,
            num_enabled: 0,
        }
    }

    pub(crate) fn update<P, SM>(
        &mut self,
        mut diff: u64,
        servo_cluster: &mut ServoCluster<{ NUM_SERVOS as usize }, P, SM, AngularCalibration>,
        joints: &[[Joint; NUM_SERVOS_PER_LEG]; NUM_LEGS],
    ) -> bool
    where
        P: PIOExt,
        SM: StateMachineIndex,
    {
        diff += self.last_diff;
        if self.num_enabled < NUM_SERVOS && diff > 500 {
            // Enable servo every 500ms.
            let leg = (self.num_enabled / 3) as usize;
            let joint = (self.num_enabled % 3) as usize;
            let joint = &joints[leg][joint];
            servo_cluster.set_pulse(joint.servo(), joint.cal().home_pulse, false);
            servo_cluster.set_enabled(joint.servo(), true, true);
            self.num_enabled += 1;
            self.last_diff = 0;
            true
        } else {
            if self.num_enabled < NUM_SERVOS {
                self.last_diff = diff;
            }
            false
        }
    }

    pub(crate) fn reset(&mut self) {
        self.num_enabled = 0;
    }
}
