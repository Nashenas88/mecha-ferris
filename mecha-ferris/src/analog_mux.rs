use crate::analog::Analog;
use crate::flexible_input::{FlexibleInput, ReadError};
use embedded_hal::digital::v2::{InputPin, OutputPin, PinState};
use embedded_hal::timer::CountDown as _;
use fugit::ExtU64;
use pimoroni_servo2040::hal::gpio::bank0::{Gpio22, Gpio24, Gpio25, Gpio29};
use pimoroni_servo2040::hal::gpio::PushPullOutput;
use pimoroni_servo2040::hal::gpio::{FloatingInput, Pin};
use pimoroni_servo2040::hal::timer::CountDown;
use pimoroni_servo2040::{
    CURRENT_SENSE_ADDR, SENSOR_1_ADDR, SENSOR_2_ADDR, SENSOR_3_ADDR, SENSOR_4_ADDR, SENSOR_5_ADDR,
    SENSOR_6_ADDR, VOLTAGE_SENSE_ADDR,
};

pub type AdcAddr0Pin = Pin<Gpio22, PushPullOutput>;
pub type AdcAddr1Pin = Pin<Gpio24, PushPullOutput>;
pub type AdcAddr2Pin = Pin<Gpio25, PushPullOutput>;
pub type SharedAdcPin = FlexibleInput<Gpio29>;
pub type FloatingSharedAdcPin = Pin<Gpio29, FloatingInput>;
const MAX_ADDRESS: u8 = 0b_0000_0111;

pub struct Sensor1;
pub struct Sensor2;
pub struct Sensor3;
pub struct Sensor4;
pub struct Sensor5;
pub struct Sensor6;
pub struct VoltageSensor;
pub struct CurrentSensor;

mod private {
    pub trait Sealed {}
}
use private::Sealed;

pub trait SensorAddress: Sealed {
    const ADDR: u8;
}

impl Sealed for Sensor1 {}
impl SensorAddress for Sensor1 {
    const ADDR: u8 = SENSOR_1_ADDR;
}
impl Sealed for Sensor2 {}
impl SensorAddress for Sensor2 {
    const ADDR: u8 = SENSOR_2_ADDR;
}
impl Sealed for Sensor3 {}
impl SensorAddress for Sensor3 {
    const ADDR: u8 = SENSOR_3_ADDR;
}
impl Sealed for Sensor4 {}
impl SensorAddress for Sensor4 {
    const ADDR: u8 = SENSOR_4_ADDR;
}
impl Sealed for Sensor5 {}
impl SensorAddress for Sensor5 {
    const ADDR: u8 = SENSOR_5_ADDR;
}
impl Sealed for Sensor6 {}
impl SensorAddress for Sensor6 {
    const ADDR: u8 = SENSOR_6_ADDR;
}
impl Sealed for VoltageSensor {}
impl SensorAddress for VoltageSensor {
    const ADDR: u8 = VOLTAGE_SENSE_ADDR;
}
impl Sealed for CurrentSensor {}
impl SensorAddress for CurrentSensor {
    const ADDR: u8 = CURRENT_SENSE_ADDR;
}

pub struct AnalogMux {
    addr0_pin: AdcAddr0Pin,
    addr1_pin: AdcAddr1Pin,
    addr2_pin: AdcAddr2Pin,
    muxed_pin: SharedAdcPin,
    pull_ups: u8,
    pull_downs: u8,
}

impl AnalogMux {
    pub fn new(
        addr0_pin: AdcAddr0Pin,
        addr1_pin: AdcAddr1Pin,
        addr2_pin: AdcAddr2Pin,
        muxed_pin: SharedAdcPin,
    ) -> Self {
        Self {
            addr0_pin,
            addr1_pin,
            addr2_pin,
            muxed_pin,
            pull_ups: 0,
            pull_downs: 0,
        }
    }

    fn select(&mut self, address: u8, count_down: &mut CountDown) {
        if address > MAX_ADDRESS {
            return;
        }
        let to_pull_up = self.pull_ups & (1 << address);
        let to_pull_down = self.pull_downs & (1 << address);
        if to_pull_up == 0 && to_pull_down == 0 {
            self.muxed_pin = self.muxed_pin.take().into_pull_down_disabled();
        }

        let _ = self
            .addr0_pin
            .set_state(PinState::from((address & 0b001) != 0));
        let _ = self
            .addr1_pin
            .set_state(PinState::from((address & 0b010) != 0));
        let _ = self
            .addr2_pin
            .set_state(PinState::from((address & 0b100) != 0));

        if to_pull_up != 0 && to_pull_down != 0 {
            self.muxed_pin = self.muxed_pin.take().into_bus_keep_input();
        } else if to_pull_up != 0 {
            self.muxed_pin = self.muxed_pin.take().into_pull_up_input();
        } else if to_pull_down != 0 {
            self.muxed_pin = self.muxed_pin.take().into_pull_down_input();
        } else {
            self.muxed_pin = self.muxed_pin.take().into_floating_input();
        }

        count_down.start(10.micros());
        let _ = nb::block!(count_down.wait());
    }

    pub fn configure_pulls(&mut self, address: u8, pullup: bool, pulldown: bool) {
        if address > MAX_ADDRESS {
            return;
        }

        if pullup {
            self.pull_ups |= 1 << address;
        } else {
            self.pull_ups &= !(1 << address);
        }

        if pulldown {
            self.pull_downs |= 1 << address;
        } else {
            self.pull_downs &= !(1 << address);
        }
    }

    #[inline]
    pub fn is_high(&self) -> Result<bool, ReadError> {
        self.muxed_pin.is_high()
    }

    #[inline]
    pub fn is_low(&self) -> Result<bool, ReadError> {
        self.muxed_pin.is_low()
    }

    pub fn reader<T>(&mut self, count_down: &mut CountDown) -> Analog<'_, T>
    where
        T: SensorAddress,
    {
        self.select(T::ADDR, count_down);
        Analog::new(self.muxed_pin.mut_floating_input())
    }
}
