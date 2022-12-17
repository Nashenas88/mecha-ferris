use core::hint::unreachable_unchecked;
use embedded_hal::digital::v2::InputPin;
use pimoroni_servo2040::hal::gpio::{
    BusKeepInput, FloatingInput, Pin, PinId, PullDownDisabled, PullDownInput, PullUpInput,
};

pub enum FlexibleInput<I>
where
    I: PinId,
{
    PullUpInput(Pin<I, PullUpInput>),
    PullDownInput(Pin<I, PullDownInput>),
    BusKeepInput(Pin<I, BusKeepInput>),
    FloatingInput(Pin<I, FloatingInput>),
    PullDownDisabled(Pin<I, PullDownDisabled>),
    Empty,
}

impl<I> FlexibleInput<I>
where
    I: PinId,
{
    pub(crate) fn into_pull_up_input(self) -> Self {
        match self {
            FlexibleInput::PullUpInput(_) => self,
            FlexibleInput::PullDownInput(pin) => {
                FlexibleInput::PullUpInput(pin.into_pull_up_input())
            }
            FlexibleInput::BusKeepInput(pin) => {
                FlexibleInput::PullUpInput(pin.into_pull_up_input())
            }
            FlexibleInput::FloatingInput(pin) => {
                FlexibleInput::PullUpInput(pin.into_pull_up_input())
            }
            FlexibleInput::PullDownDisabled(pin) => {
                FlexibleInput::PullUpInput(pin.into_pull_up_input())
            }
            FlexibleInput::Empty => {
                unreachable!()
            }
        }
    }

    pub(crate) fn into_pull_down_input(self) -> Self {
        match self {
            FlexibleInput::PullUpInput(pin) => {
                FlexibleInput::PullDownInput(pin.into_pull_down_input())
            }
            FlexibleInput::PullDownInput(_) => self,
            FlexibleInput::BusKeepInput(pin) => {
                FlexibleInput::PullDownInput(pin.into_pull_down_input())
            }
            FlexibleInput::FloatingInput(pin) => {
                FlexibleInput::PullDownInput(pin.into_pull_down_input())
            }
            FlexibleInput::PullDownDisabled(pin) => {
                FlexibleInput::PullDownInput(pin.into_pull_down_input())
            }
            FlexibleInput::Empty => {
                unreachable!()
            }
        }
    }

    pub(crate) fn into_bus_keep_input(self) -> Self {
        match self {
            FlexibleInput::PullUpInput(pin) => {
                FlexibleInput::BusKeepInput(pin.into_bus_keep_input())
            }
            FlexibleInput::PullDownInput(pin) => {
                FlexibleInput::BusKeepInput(pin.into_bus_keep_input())
            }
            FlexibleInput::BusKeepInput(_) => self,
            FlexibleInput::FloatingInput(pin) => {
                FlexibleInput::BusKeepInput(pin.into_bus_keep_input())
            }
            FlexibleInput::PullDownDisabled(pin) => {
                FlexibleInput::BusKeepInput(pin.into_bus_keep_input())
            }
            FlexibleInput::Empty => {
                unreachable!()
            }
        }
    }

    pub(crate) fn into_pull_down_disabled(self) -> Self {
        match self {
            FlexibleInput::PullUpInput(pin) => {
                FlexibleInput::PullDownDisabled(pin.into_pull_down_disabled())
            }
            FlexibleInput::PullDownInput(pin) => {
                FlexibleInput::PullDownDisabled(pin.into_pull_down_disabled())
            }
            FlexibleInput::BusKeepInput(pin) => {
                FlexibleInput::PullDownDisabled(pin.into_pull_down_disabled())
            }
            FlexibleInput::FloatingInput(pin) => {
                FlexibleInput::PullDownDisabled(pin.into_pull_down_disabled())
            }
            FlexibleInput::PullDownDisabled(_) => self,
            FlexibleInput::Empty => {
                unreachable!()
            }
        }
    }

    pub(crate) fn into_floating_input(self) -> Self {
        match self {
            FlexibleInput::PullUpInput(pin) => {
                FlexibleInput::FloatingInput(pin.into_floating_input())
            }
            FlexibleInput::PullDownInput(pin) => {
                FlexibleInput::FloatingInput(pin.into_floating_input())
            }
            FlexibleInput::BusKeepInput(pin) => {
                FlexibleInput::FloatingInput(pin.into_floating_input())
            }
            FlexibleInput::FloatingInput(_) => self,
            FlexibleInput::PullDownDisabled(pin) => {
                FlexibleInput::FloatingInput(pin.into_floating_input())
            }
            FlexibleInput::Empty => {
                unreachable!()
            }
        }
    }

    pub(crate) fn mut_floating_input(&mut self) -> &mut Pin<I, FloatingInput> {
        *self = match self.take() {
            FlexibleInput::PullUpInput(pin) => {
                FlexibleInput::FloatingInput(pin.into_floating_input())
            }
            FlexibleInput::PullDownInput(pin) => {
                FlexibleInput::FloatingInput(pin.into_floating_input())
            }
            FlexibleInput::BusKeepInput(pin) => {
                FlexibleInput::FloatingInput(pin.into_floating_input())
            }
            FlexibleInput::FloatingInput(pin) => FlexibleInput::FloatingInput(pin),
            FlexibleInput::PullDownDisabled(pin) => {
                FlexibleInput::FloatingInput(pin.into_floating_input())
            }
            FlexibleInput::Empty => {
                unreachable!()
            }
        };
        match self {
            FlexibleInput::FloatingInput(ref mut pin) => pin,
            // Safety: We just set the value above.
            _ => unsafe { unreachable_unchecked() },
        }
    }

    pub(crate) fn take(&mut self) -> Self {
        core::mem::replace(self, FlexibleInput::Empty)
    }
}

pub enum ReadError {
    Disabled,
}

impl<I> InputPin for FlexibleInput<I>
where
    I: PinId,
{
    type Error = ReadError;

    fn is_high(&self) -> Result<bool, Self::Error> {
        match self {
            FlexibleInput::PullUpInput(pin) => pin
                .is_high()
                // Safety: rp2040 <PullUpInput as InputPin>::Error is Infallible.
                .map_err(|_| unsafe { unreachable_unchecked() }),
            FlexibleInput::PullDownInput(pin) => pin
                .is_high()
                // Safety: rp2040 <PullDownInput as InputPin>::Error is Infallible.
                .map_err(|_| unsafe { unreachable_unchecked() }),
            FlexibleInput::BusKeepInput(pin) => pin
                .is_high()
                // Safety: rp2040 <BusKeepInput as InputPin>::Error is Infallible.
                .map_err(|_| unsafe { unreachable_unchecked() }),
            FlexibleInput::FloatingInput(pin) => pin
                .is_high()
                // Safety: rp2040 <FloatingInput as InputPin>::Error is Infallible.
                .map_err(|_| unsafe { unreachable_unchecked() }),
            FlexibleInput::PullDownDisabled(_) => Err(ReadError::Disabled),
            FlexibleInput::Empty => Err(ReadError::Disabled),
        }
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        match self {
            FlexibleInput::PullUpInput(pin) => pin
                .is_low()
                // Safety: rp2040 <PullUpInput as InputPin>::Error is Infallible.
                .map_err(|_| unsafe { unreachable_unchecked() }),
            FlexibleInput::PullDownInput(pin) => pin
                .is_low()
                // Safety: rp2040 <PullDownInput as InputPin>::Error is Infallible.
                .map_err(|_| unsafe { unreachable_unchecked() }),
            FlexibleInput::BusKeepInput(pin) => pin
                .is_low()
                // Safety: rp2040 <BusKeepInput as InputPin>::Error is Infallible.
                .map_err(|_| unsafe { unreachable_unchecked() }),
            FlexibleInput::FloatingInput(pin) => pin
                .is_low()
                // Safety: rp2040 <FloatingInput as InputPin>::Error is Infallible.
                .map_err(|_| unsafe { unreachable_unchecked() }),
            FlexibleInput::PullDownDisabled(_) => Err(ReadError::Disabled),
            FlexibleInput::Empty => Err(ReadError::Disabled),
        }
    }
}

impl<I> From<Pin<I, PullUpInput>> for FlexibleInput<I>
where
    I: PinId,
{
    fn from(pin: Pin<I, PullUpInput>) -> Self {
        FlexibleInput::PullUpInput(pin)
    }
}

impl<I> From<Pin<I, PullDownInput>> for FlexibleInput<I>
where
    I: PinId,
{
    fn from(pin: Pin<I, PullDownInput>) -> Self {
        FlexibleInput::PullDownInput(pin)
    }
}

impl<I> From<Pin<I, BusKeepInput>> for FlexibleInput<I>
where
    I: PinId,
{
    fn from(pin: Pin<I, BusKeepInput>) -> Self {
        FlexibleInput::BusKeepInput(pin)
    }
}

impl<I> From<Pin<I, FloatingInput>> for FlexibleInput<I>
where
    I: PinId,
{
    fn from(pin: Pin<I, FloatingInput>) -> Self {
        FlexibleInput::FloatingInput(pin)
    }
}

impl<I> From<Pin<I, PullDownDisabled>> for FlexibleInput<I>
where
    I: PinId,
{
    fn from(pin: Pin<I, PullDownDisabled>) -> Self {
        FlexibleInput::PullDownDisabled(pin)
    }
}
