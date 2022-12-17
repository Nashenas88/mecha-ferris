use core::marker::PhantomData;

use embedded_hal::adc::OneShot;
use pimoroni_servo2040::hal::gpio::bank0::Gpio29;
use pimoroni_servo2040::hal::gpio::{FloatingInput, Pin};
use pimoroni_servo2040::hal::Adc;
use pimoroni_servo2040::{CURRENT_GAIN, CURRENT_OFFSET};

use crate::analog_mux::{CurrentSensor, VoltageSensor};

type SharedAdcPin = Pin<Gpio29, FloatingInput>;

pub struct Analog<'a, T> {
    pin: &'a mut SharedAdcPin,
    _phantom: PhantomData<T>,
}

impl<'a, T> Analog<'a, T> {
    pub(crate) fn new(pin: &'a mut SharedAdcPin) -> Self {
        Self {
            pin,
            _phantom: PhantomData,
        }
    }

    /// Read the raw data for this sensor.
    pub fn read_raw(&mut self, adc: &mut Adc) -> u16 {
        <Adc as OneShot<Adc, u16, SharedAdcPin>>::read(adc, self.pin).unwrap()
    }
}

impl<'a> Analog<'a, VoltageSensor> {
    /// Returns the voltage across the shunt resistor.
    pub fn read_voltage(&mut self, adc: &mut Adc) -> f32 {
        let voltage =
            (<Adc as OneShot<Adc, f32, SharedAdcPin>>::read(adc, self.pin).unwrap() * 3.3 / 4096.0
                + CURRENT_OFFSET)
                / CURRENT_GAIN as f32;
        voltage.max(0.0)
    }
}

impl<'a> Analog<'a, CurrentSensor> {
    /// Returns the current measures across the shunt resistor.
    pub fn read_current(&mut self, adc: &mut Adc) -> f32 {
        // Safety: Analog has 1 field, and the const generic does not affect the fields.
        // Layout should be the same.
        unsafe { core::mem::transmute::<_, &mut Analog<'a, VoltageSensor>>(self) }.read_voltage(adc)
            / pimoroni_servo2040::SHUNT_RESISTOR
    }
}
