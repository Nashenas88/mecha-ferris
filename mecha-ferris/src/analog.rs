use core::marker::PhantomData;

use embedded_hal::adc::{Channel, OneShot};
use pimoroni_servo2040::hal::gpio::bank0::Gpio29;
use pimoroni_servo2040::hal::gpio::{FloatingInput, Pin};
use pimoroni_servo2040::hal::Adc;
use pimoroni_servo2040::{CURRENT_GAIN, CURRENT_OFFSET};
use servo_pio::calibration::map_float;

use crate::analog_mux::{CurrentSensor, VoltageSensor};

type SharedAdcPin = Pin<Gpio29, FloatingInput>;

#[repr(transparent)]
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
        let reading: f32 = adc.read(self.pin).unwrap();
        let voltage = (reading * 3.3 / 4096.0 + CURRENT_OFFSET) / CURRENT_GAIN as f32;
        voltage.max(0.0)
    }
}

impl<'a> Analog<'a, CurrentSensor> {
    /// Returns the current measures across the shunt resistor.
    pub fn read_current(&mut self, adc: &mut Adc) -> f32 {
        // Safety: Analog has 1 field, and the generic does not affect the fields.
        // Layout should be the same.
        unsafe { core::mem::transmute::<_, &mut Analog<'a, VoltageSensor>>(self) }.read_voltage(adc)
            / pimoroni_servo2040::SHUNT_RESISTOR
    }
}

pub fn read_external_current<P>(adc: &mut Adc, pin: &mut P) -> f32
where
    P: Channel<Adc, ID = u8>,
{
    let voltage: u16 = adc.read(pin).unwrap();
    let voltage = map_float(voltage as f32, 0.0, 1024.0, 0.0, 3.3);
    36.7 * voltage / 3.3 - 18.3
}
