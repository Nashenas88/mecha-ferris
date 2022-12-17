use super::ServoLimits;

pub(crate) struct HS311;

impl ServoLimits for HS311 {
    const MIN_PULSE: u16 = 184; // 0.9ms pulse / (20ms/4096 prescale);
    const MAX_PULSE: u16 = 430; // 2.1ms pulse / (20ms/4096 prescale);
}
