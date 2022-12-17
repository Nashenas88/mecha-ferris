use arduino_hal::I2c;
use core::marker::PhantomData;
use pwm_pca9685::{Channel, Pca9685};

mod hs311;
mod mg90d;

pub(crate) use hs311::HS311;
pub(crate) use mg90d::Mg90d;

/// The limits of the servo. Pulse values are in ticks. 
pub(crate) trait ServoLimits {
    const MIN_PULSE: u16;
    const MAX_PULSE: u16;
}

pub(crate) struct Servo<T> {
    channel: Channel,
    current: u16,
    _phantom: PhantomData<T>,
}

impl<T> Servo<T>
where
    T: ServoLimits,
{
    pub(crate) fn new(channel: Channel, pct: u8) -> Self {
        Self {
            channel,
            current: ((T::MAX_PULSE - T::MIN_PULSE) as u32 * pct as u32 * 41 >> 12) as u16 + T::MIN_PULSE,
            _phantom: PhantomData,
        }
    }

    /// Averages input between T::MAX_PULSE and T::MIN_PULSE. pct taken as integer between
    /// 0 and 100 inclusive.
    pub(crate) fn set_angle(&mut self, pct: u16) {
        debug_assert!(pct <= 100);
        self.current = ((T::MAX_PULSE - T::MIN_PULSE) as u32 * pct as u32 * 41 >> 12) as u16 + T::MIN_PULSE;
    }

    pub(crate) fn do_move(&self, pwm: &mut Pca9685<I2c>) {
        pwm.set_channel_off(self.channel, self.current).unwrap();
    }
}
