use super::ServoLimits;

pub(crate) struct Mg90d;

impl ServoLimits for Mg90d {
    const MIN_PULSE: u16 = 204; // 1ms pulse / (20ms/4096 prescale);
    const MAX_PULSE: u16 = 409; // 2ms pulse / (20ms/4096 prescale);
}
