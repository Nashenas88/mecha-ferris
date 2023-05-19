use embedded_hal::digital::v2::InputPin;

pub struct Debouncer<P> {
    is_low: bool,
    ticks: u16,
    debounce_ticks: u16,
    pin: P,
}

impl<P> Debouncer<P> {
    pub fn new(debounce_ticks: u16, pin: P) -> Self {
        Self {
            is_low: false,
            ticks: 0,
            debounce_ticks,
            pin,
        }
    }

    pub fn is_low(&self) -> bool {
        self.is_low
    }
}

impl<P> Debouncer<P>
where
    P: InputPin,
    <P as InputPin>::Error: core::fmt::Debug,
{
    pub fn update(&mut self) -> bool {
        let pin_high = self.pin.is_low().unwrap();
        if self.is_low == pin_high {
            self.ticks = 0;
            return false;
        }

        self.ticks += 1;
        if self.ticks < self.debounce_ticks {
            return false;
        }

        self.is_low = pin_high;
        self.ticks = 0;
        true
    }
}
