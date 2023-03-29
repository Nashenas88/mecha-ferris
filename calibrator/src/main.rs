#![no_std]
#![no_main]

use calibrations::CalData;
use defmt::Format;
use defmt_rtt as _;
use embedded_graphics::geometry::Point as GeoPoint;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::{MonoFont, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::text::{Alignment, Baseline, Text, TextStyle, TextStyleBuilder};
use embedded_graphics::Drawable;
use embedded_hal::adc::OneShot;
use embedded_hal::timer::CountDown;
use fugit::{ExtU64, RateExtU32};
use micromath::F32Ext;
use panic_probe as _;
use pimoroni_servo2040::hal::clocks::SystemClock;
use pimoroni_servo2040::hal::dma::{Channel, ChannelIndex, DMAExt, CH0, CH1};
use pimoroni_servo2040::hal::gpio::{Error as GpioError, FunctionConfig, FunctionPio0};
use pimoroni_servo2040::hal::pio::{PIOExt, StateMachineIndex, UninitStateMachine, PIO, SM0};
use pimoroni_servo2040::hal::{self, pac, Clock};
use pimoroni_servo2040::pac::{interrupt, PIO0};
use servo_pio::calibration::{AngularCalibration, CalibrationData, Point};
use servo_pio::pwm_cluster::{dma_interrupt, GlobalState, GlobalStates, Handler};
use servo_pio::servo_cluster::{ServoCluster, ServoClusterBuilderError, ServoData};
use ssd1306::prelude::DisplayConfig;
use ssd1306::rotation::DisplayRotation;
use ssd1306::size::{DisplaySize, DisplaySize128x32};
use ssd1306::Ssd1306;

use crate::calibrations::map_float;

const NUM_SERVOS_PER_LEG: usize = 3;
const NUM_LEGS: usize = 6;
const NUM_SERVOS: usize = NUM_SERVOS_PER_LEG * NUM_LEGS;
const NUM_CHANNELS: usize = 2;
// Number of readings to average over to stabilize.
const SAMPLES: u8 = 16;

// Aliases to easily manage swapping out screens.
type ScreenSize = DisplaySize128x32;
const SCREEN_SIZE: ScreenSize = DisplaySize128x32;
// Text rendering properties.
const FONT: MonoFont = FONT_6X10;
// We want the text centered on the screen.
const TEXT_STYLE: TextStyle = TextStyleBuilder::new()
    .alignment(Alignment::Center)
    .baseline(Baseline::Middle)
    .build();
const POSITION: GeoPoint = GeoPoint {
    x: <ScreenSize as DisplaySize>::WIDTH as i32 / 2,
    y: <ScreenSize as DisplaySize>::HEIGHT as i32 / 2,
};

static mut STATE1: Option<GlobalState<CH0, CH1, PIO0, SM0>> = {
    const NONE_HACK: Option<GlobalState<CH0, CH1, PIO0, SM0>> = None;
    NONE_HACK
};
static mut GLOBALS: GlobalStates<NUM_CHANNELS> = {
    const NONE_HACK: Option<&'static mut dyn Handler> = None;
    GlobalStates {
        states: [NONE_HACK; NUM_CHANNELS],
    }
};

const UPDATE_MS: u64 = 50;

mod calibrations;
mod debouncer;

#[pimoroni_servo2040::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let sio = hal::Sio::new(pac.SIO);

    let clocks = hal::clocks::init_clocks_and_plls(
        pimoroni_servo2040::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = pimoroni_servo2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Update pins depending on your board/setup.
    let sda_pin = pins.sda;
    let scl_pin = pins.scl;
    let mut potentiometer_pin = pins.adc0.into_floating_input();

    // Setup i2c for display.
    let i2c1 = pimoroni_servo2040::hal::I2C::i2c0(
        pac.I2C0,
        sda_pin.into_mode(),
        scl_pin.into_mode(),
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
    );

    // Create the I2C display interface:
    let interface = ssd1306::I2CDisplayInterface::new(i2c1);
    let mut display = Ssd1306::new(interface, SCREEN_SIZE, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let character_style = MonoTextStyleBuilder::new()
        .font(&FONT)
        .text_color(BinaryColor::On)
        .build();

    // Setup adc for potentiometer.
    let mut adc = pimoroni_servo2040::hal::Adc::new(pac.ADC, &mut pac.RESETS);
    // Buffer for writing formatting data into. Used to display pulse readings.
    let mut buf = [0u8; 64];

    let mut calibrations = calibrations::calibrations();
    let servo_pins: [_; NUM_SERVOS] = [
        ServoData {
            pin: pins.servo1.into_mode::<FunctionPio0>().into(),
            calibration: calibrations[0][0].cal,
        },
        ServoData {
            pin: pins.servo2.into_mode::<FunctionPio0>().into(),
            calibration: calibrations[0][1].cal,
        },
        ServoData {
            pin: pins.servo3.into_mode::<FunctionPio0>().into(),
            calibration: calibrations[0][2].cal,
        },
        ServoData {
            pin: pins.servo4.into_mode::<FunctionPio0>().into(),
            calibration: calibrations[1][0].cal,
        },
        ServoData {
            pin: pins.servo5.into_mode::<FunctionPio0>().into(),
            calibration: calibrations[1][1].cal,
        },
        ServoData {
            pin: pins.servo6.into_mode::<FunctionPio0>().into(),
            calibration: calibrations[1][2].cal,
        },
        ServoData {
            pin: pins.servo7.into_mode::<FunctionPio0>().into(),
            calibration: calibrations[2][0].cal,
        },
        ServoData {
            pin: pins.servo8.into_mode::<FunctionPio0>().into(),
            calibration: calibrations[2][1].cal,
        },
        ServoData {
            pin: pins.servo9.into_mode::<FunctionPio0>().into(),
            calibration: calibrations[2][2].cal,
        },
        ServoData {
            pin: pins.servo10.into_mode::<FunctionPio0>().into(),
            calibration: calibrations[3][0].cal,
        },
        ServoData {
            pin: pins.servo11.into_mode::<FunctionPio0>().into(),
            calibration: calibrations[3][1].cal,
        },
        ServoData {
            pin: pins.servo12.into_mode::<FunctionPio0>().into(),
            calibration: calibrations[3][2].cal,
        },
        ServoData {
            pin: pins.servo13.into_mode::<FunctionPio0>().into(),
            calibration: calibrations[4][0].cal,
        },
        ServoData {
            pin: pins.servo14.into_mode::<FunctionPio0>().into(),
            calibration: calibrations[4][1].cal,
        },
        ServoData {
            pin: pins.servo15.into_mode::<FunctionPio0>().into(),
            calibration: calibrations[4][2].cal,
        },
        ServoData {
            pin: pins.servo16.into_mode::<FunctionPio0>().into(),
            calibration: calibrations[5][0].cal,
        },
        ServoData {
            pin: pins.servo17.into_mode::<FunctionPio0>().into(),
            calibration: calibrations[5][1].cal,
        },
        ServoData {
            pin: pins.servo18.into_mode::<FunctionPio0>().into(),
            calibration: calibrations[5][2].cal,
        },
    ];

    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let dma = pac.DMA.split(&mut pac.RESETS);

    // Configure the Timer peripheral in count-down mode.
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut count_down = timer.count_down();

    let mut servo_cluster = match build_servo_cluster(
        &mut pio0,
        sm0,
        (dma.ch0, dma.ch1),
        servo_pins,
        clocks.system_clock,
        unsafe { &mut STATE1 },
    ) {
        Ok(cluster) => cluster,
        Err(e) => {
            defmt::error!("Failed to build servo cluster: {:?}", e);
            #[allow(clippy::empty_loop)]
            loop {}
        }
    };

    // Unmask the DMA interrupt so the handler can start running. This can only
    // be done after the servo cluster has been built.
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::DMA_IRQ_0);
    }

    // We need to use the indices provided by the cluster because the servo pin
    // numbers do not line up with the indices in the clusters and PIO.
    #[rustfmt::skip]
    let [
        servo1, servo2, servo3,
        servo4, servo5, servo6,
        servo7, servo8, servo9,
        servo10, servo11, servo12,
        servo13, servo14, servo15,
        servo16, servo17, servo18
    ] = servo_cluster.servos();
    let servos = [
        [servo1, servo2, servo3],
        [servo4, servo5, servo6],
        [servo7, servo8, servo9],
        [servo10, servo11, servo12],
        [servo13, servo14, servo15],
        [servo16, servo17, servo18],
    ];

    let collect_pin = pins.user_sw.into_pull_up_input();
    let up_pin = pins.adc2.into_pull_up_input();
    let down_pin = pins.adc1.into_pull_up_input();
    let mut collect_debouncer = debouncer::Debouncer::new(5, collect_pin);
    let mut up_debouncer = debouncer::Debouncer::new(5, up_pin);
    let mut down_debouncer = debouncer::Debouncer::new(5, down_pin);

    display.clear();
    let text = Text::with_text_style("Press SW to start", POSITION, character_style, TEXT_STYLE);
    text.draw(&mut display).unwrap();
    display.flush().unwrap();

    defmt::info!("Waiting for start signal");

    // Loop until signal received for starting.
    loop {
        if collect_debouncer.update() && collect_debouncer.is_low() {
            break;
        }

        count_down.start(50.millis());
        let _ = nb::block!(count_down.wait());
    }
    defmt::info!("Signal received. Starting");
    display.clear();
    let text = Text::with_text_style("Homing...", POSITION, character_style, TEXT_STYLE);
    text.draw(&mut display).unwrap();
    display.flush().unwrap();

    // Move all legs to known home positions.
    for (leg, leg_cal) in servos.iter().zip(calibrations.iter()) {
        for (s, sc) in leg.iter().zip(leg_cal.iter()) {
            servo_cluster.set_pulse(*s, sc.home_pulse, false);
            servo_cluster.set_enabled(*s, true, true);

            count_down.start(500.millis());
            let _ = nb::block!(count_down.wait());
        }
    }
    servo_cluster.load();

    count_down.start(1.secs());
    let _ = nb::block!(count_down.wait());

    let mut leg_num = 0;
    let mut servo_num = 0;
    let mut angle_id = Angles::Home;
    let mut precision = PrecisionMode::Rough;
    let sensitivity_pin = pins.int_.into_pull_up_input();
    let mut sensitivity_debouncer = debouncer::Debouncer::new(5, sensitivity_pin);
    let mut sample_id = 0;
    let mut samples: [f32; SAMPLES as usize] = [0.0; SAMPLES as usize];
    let mut num_samples = 0;
    let mut target = 0.0;

    const MAX_READING: f32 = 4096.0;
    const DEVIATION: f32 = 200.0;
    const MIN: f32 = 350.0;
    const MAX: f32 = 2850.0;
    // Pause if we're about to home so we don't slam into anything.
    let mut home_pause = false;

    let mut now = timer.get_counter();
    loop {
        let last_update = now;
        now = timer.get_counter();
        let diff = (now - last_update).to_millis();
        let cal_data = &mut calibrations[leg_num][servo_num];
        let servo = servos[leg_num][servo_num];
        // Read the potentiometer value, averaging a few samples to get rid of noise.
        let reading: f32 = adc.read(&mut potentiometer_pin).unwrap();
        samples[sample_id] = map_float(
            reading / 2.0,
            0.0,
            MAX_READING / 2.0,
            if let PrecisionMode::Fine = precision {
                target - DEVIATION
            } else {
                MIN
            },
            if let PrecisionMode::Fine = precision {
                target + DEVIATION
            } else {
                MAX
            },
        );
        if num_samples < SAMPLES {
            num_samples += 1;
        }
        sample_id = (sample_id + 1) % SAMPLES as usize;
        let pulse = samples[..num_samples as usize].iter().sum::<f32>() / num_samples as f32;

        // If sensitivity button press, update servo precision.
        if sensitivity_debouncer.update() && sensitivity_debouncer.is_low() {
            defmt::info!("button pressed");
            precision = precision.next();
            match precision {
                PrecisionMode::Fine => target = pulse,
                PrecisionMode::Rough => {}
            }
        }

        // If collect button press, collect data and move to next target.
        if collect_debouncer.update() && collect_debouncer.is_low() {
            // If we're in home pause, just turn it off. Otherwise, collect the angle.
            if home_pause {
                home_pause = false;
                continue;
            } else {
                angle_id.set_pulse(cal_data, pulse);
                // Coxa and Femur servos share home and mid pulses.
                if Angles::Home == angle_id && (servo_num == 1 || servo_num == 2) {
                    Angles::Mid.set_pulse(cal_data, pulse);
                }
            }
            angle_id = angle_id.next();
            // Coxa and Femur servos share home and mid pulses, so this has already been collected.
            if Angles::Mid == angle_id && (servo_num == 1 || servo_num == 2) {
                angle_id = angle_id.next();
            }
            // If all angles collected, move to next servo.
            if let Angles::Done = angle_id {
                angle_id = Angles::Home;
                home_pause = true;
                // Move tibia up to avoid collisions.
                match servo_num {
                    0 => {
                        servo_cluster.set_pulse(servo, cal_data.cal.inner_mut().min_pulse(), true);
                    }
                    1 => {
                        servo_cluster.set_pulse(servo, cal_data.cal.inner_mut().max_pulse(), true);
                    }
                    2 => {
                        servo_cluster.set_pulse(servo, cal_data.cal.inner_mut().mid_pulse(), true);
                    }
                    _ => unreachable!(),
                }

                servo_num += 1;
                if servo_num == NUM_SERVOS_PER_LEG {
                    leg_num += 1;
                    servo_num = 0;
                    if leg_num == NUM_LEGS {
                        log_results(&calibrations);
                        break;
                    }
                }

                continue;
            }
        }

        let target_angle = angle_id.angle(cal_data);
        let servo_pulse = if Angles::Home == angle_id && home_pause {
            cal_data.home_pulse
        } else {
            pulse
        };
        servo_cluster.set_pulse(servo, servo_pulse, true);

        display.clear();
        let pulse = pulse.round() as u16;
        let s: &str = write_to::show(
            &mut buf,
            format_args!(
                "{} L{}J{} {:?}\n{:?} {}d: {}",
                precision.label(),
                leg_num,
                cal_data.name,
                angle_id,
                cal_data.typ,
                target_angle,
                pulse // "{} Pulse1: {:04}", precision.label(), pulse
            ),
        )
        .unwrap();
        let text = Text::with_text_style(s, POSITION, character_style, TEXT_STYLE);
        text.draw(&mut display).unwrap();
        display.flush().unwrap();

        count_down.start((UPDATE_MS - diff).min(0).millis());
        let _ = nb::block!(count_down.wait());
    }

    leg_num = 0;
    servo_num = 0;
    angle_id = Angles::Home;

    // Calibration done, display results
    loop {
        let last_update = now;
        now = timer.get_counter();
        let diff = (now - last_update).to_millis();

        if up_debouncer.update() && up_debouncer.is_low() {
            let (aid, wrapped) = angle_id.next_wrapping();
            angle_id = aid;
            if wrapped {
                servo_num += 1;
                if servo_num == 3 {
                    servo_num = 0;
                    leg_num += 1;
                    if leg_num == 6 {
                        leg_num = 0;
                    }
                }
            }
        } else if down_debouncer.update() && down_debouncer.is_low() {
            let (aid, wrapped) = angle_id.prev_wrapping();
            angle_id = aid;
            if wrapped {
                if servo_num == 0 {
                    servo_num = 2;
                    if leg_num == 0 {
                        leg_num = 5;
                    } else {
                        leg_num -= 1;
                    }
                } else {
                    servo_num -= 1;
                }
            }
        }

        let cal_data = &calibrations[leg_num][servo_num];
        let target_angle = angle_id.angle(cal_data);
        let pulse = angle_id.pulse(cal_data);
        display.clear();
        let pulse = pulse.round() as u16;
        let s: &str = write_to::show(
            &mut buf,
            format_args!(
                "{} L{}J{} {:?}\n{:?} {}d: {}",
                precision.label(),
                leg_num,
                cal_data.name,
                angle_id,
                cal_data.typ,
                target_angle,
                pulse // "{} Pulse1: {:04}", precision.label(), pulse
            ),
        )
        .unwrap();
        let text = Text::with_text_style(s, POSITION, character_style, TEXT_STYLE);
        text.draw(&mut display).unwrap();
        display.flush().unwrap();

        count_down.start((UPDATE_MS - diff).min(0).millis());
        let _ = nb::block!(count_down.wait());
    }
}

fn log_results(calibrations: &[[CalData; NUM_SERVOS_PER_LEG]; NUM_LEGS]) {
    defmt::info!("Calibration done:");
    for leg in calibrations {
        for servo in leg {
            defmt::info!("{} ({:?}): ", servo.name, servo.typ);
            defmt::info!(
                "Min: ({}, {})",
                servo.typ.min_angle(),
                servo.cal.value_to_pulse(servo.typ.min_angle()).pulse
            );
            defmt::info!(
                "Mid: ({}, {})",
                servo.typ.mid_angle(),
                servo.cal.value_to_pulse(servo.typ.mid_angle()).pulse
            );
            defmt::info!(
                "Max: ({}, {})",
                servo.typ.max_angle(),
                servo.cal.value_to_pulse(servo.typ.max_angle()).pulse
            );
            defmt::info!(
                "Home: ({}, {})",
                servo.typ.home_angle(),
                servo.cal.value_to_pulse(servo.typ.home_angle()).pulse
            );
        }
    }
}

#[derive(Format, Debug, PartialEq, Eq)]
enum Angles {
    Home,
    Min,
    Mid,
    Max,
    Done,
}

impl Angles {
    fn next(self) -> Self {
        match self {
            Angles::Home => Angles::Min,
            Angles::Min => Angles::Mid,
            Angles::Mid => Angles::Max,
            Angles::Max => Angles::Done,
            Angles::Done => Angles::Done,
        }
    }

    fn next_wrapping(self) -> (Self, bool) {
        match self {
            Angles::Home => (Angles::Min, false),
            Angles::Min => (Angles::Mid, false),
            Angles::Mid => (Angles::Max, false),
            Angles::Max => (Angles::Home, true),
            Angles::Done => unreachable!(),
        }
    }

    fn prev_wrapping(self) -> (Self, bool) {
        match self {
            Angles::Home => (Angles::Max, true),
            Angles::Min => (Angles::Home, false),
            Angles::Mid => (Angles::Min, false),
            Angles::Max => (Angles::Mid, false),
            Angles::Done => unreachable!(),
        }
    }

    fn angle(&self, cal: &CalData) -> f32 {
        match self {
            Angles::Home => cal.typ.home_angle(),
            Angles::Min => cal.typ.min_angle(),
            Angles::Mid => cal.typ.mid_angle(),
            Angles::Max => cal.typ.max_angle(),
            Angles::Done => unreachable!(),
        }
    }

    fn set_pulse(&self, cal: &mut CalData, pulse: f32) {
        match self {
            Angles::Home => cal.home_pulse = pulse,
            Angles::Min => cal.cal.inner_mut().set_min_pulse(pulse),
            Angles::Mid => cal.cal.inner_mut().set_mid_pulse(pulse),
            Angles::Max => cal.cal.inner_mut().set_max_pulse(pulse),
            Angles::Done => unreachable!(),
        }
    }

    fn pulse(&self, cal: &CalData) -> f32 {
        match self {
            Angles::Home => cal.home_pulse,
            Angles::Min => cal.cal.inner().min_pulse(),
            Angles::Mid => cal.cal.inner().mid_pulse(),
            Angles::Max => cal.cal.inner().max_pulse(),
            Angles::Done => unreachable!(),
        }
    }
}

enum PrecisionMode {
    Rough,
    Fine,
}

impl PrecisionMode {
    fn next(self) -> Self {
        match self {
            PrecisionMode::Rough => PrecisionMode::Fine,
            PrecisionMode::Fine => PrecisionMode::Rough,
        }
    }

    fn label(&self) -> &'static str {
        match self {
            PrecisionMode::Rough => "RF",
            PrecisionMode::Fine => "FN",
        }
    }
}

#[derive(Copy, Clone)]
struct Calibrating {
    min: Point,
    max: Point,
}

impl Default for Calibrating {
    fn default() -> Self {
        Self {
            min: Point {
                pulse: 100.0,
                value: 0.0,
            },
            max: Point {
                pulse: 1500.0,
                value: 90.0,
            },
        }
    }
}

struct CalibratingIter<'a> {
    idx: u8,
    c: &'a Calibrating,
}

impl<'a> Iterator for CalibratingIter<'a> {
    type Item = (Point, Point);
    fn next(&mut self) -> Option<Self::Item> {
        match self.idx {
            0 => Some((self.c.min, self.c.max)),
            _ => None,
        }
    }
}

impl CalibrationData for Calibrating {
    const LEN: usize = 2;

    type Iterator<'a> = CalibratingIter<'a> where Self: 'a;

    fn first(&self) -> Point {
        self.min
    }

    fn second(&self) -> Point {
        self.max
    }

    fn penultimate(&self) -> Point {
        self.min
    }

    fn last(&self) -> Point {
        self.max
    }

    fn windows(&self) -> Self::Iterator<'_> {
        CalibratingIter { idx: 0, c: self }
    }
}

#[derive(Format)]
enum BuildError {
    Gpio(GpioError),
    Build(ServoClusterBuilderError),
}

fn build_servo_cluster<C1, C2, P, SM>(
    pio: &mut PIO<P>,
    sm: UninitStateMachine<(P, SM)>,
    dma_channels: (Channel<C1>, Channel<C2>),
    servo_data: [ServoData<AngularCalibration>; NUM_SERVOS],
    system_clock: SystemClock,
    state: &'static mut Option<GlobalState<C1, C2, P, SM>>,
) -> Result<ServoCluster<NUM_SERVOS, P, SM, AngularCalibration>, BuildError>
where
    C1: ChannelIndex,
    C2: ChannelIndex,
    P: PIOExt + FunctionConfig,
    SM: StateMachineIndex,
{
    ServoCluster::<NUM_SERVOS, P, SM, AngularCalibration>::builder(pio, sm, dma_channels, unsafe {
        &mut GLOBALS
    })
    .pins_and_calibration(servo_data)
    .map_err(BuildError::Gpio)?
    .pwm_frequency(50.0)
    .build(&system_clock, state)
    .map_err(BuildError::Build)
}

#[interrupt]
fn DMA_IRQ_0() {
    critical_section::with(|_| {
        // Safety: we're within a critical section, so nothing else will modify global_state.
        dma_interrupt(unsafe { &mut GLOBALS });
    });
}

pub mod write_to {
    use core::cmp::min;
    use core::fmt;

    pub struct WriteTo<'a> {
        buffer: &'a mut [u8],
        // on write error (i.e. not enough space in buffer) this grows beyond
        // `buffer.len()`.
        used: usize,
    }

    impl<'a> WriteTo<'a> {
        pub fn new(buffer: &'a mut [u8]) -> Self {
            WriteTo { buffer, used: 0 }
        }

        pub fn as_str(self) -> Option<&'a str> {
            if self.used <= self.buffer.len() {
                // only successful concats of str - must be a valid str.
                use core::str::from_utf8_unchecked;
                Some(unsafe { from_utf8_unchecked(&self.buffer[..self.used]) })
            } else {
                None
            }
        }
    }

    impl<'a> fmt::Write for WriteTo<'a> {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            if self.used > self.buffer.len() {
                return Err(fmt::Error);
            }
            let remaining_buf = &mut self.buffer[self.used..];
            let raw_s = s.as_bytes();
            let write_num = min(raw_s.len(), remaining_buf.len());
            remaining_buf[..write_num].copy_from_slice(&raw_s[..write_num]);
            self.used += raw_s.len();
            if write_num < raw_s.len() {
                Err(fmt::Error)
            } else {
                Ok(())
            }
        }
    }

    pub fn show<'a>(buffer: &'a mut [u8], args: fmt::Arguments) -> Result<&'a str, fmt::Error> {
        let mut w = WriteTo::new(buffer);
        fmt::write(&mut w, args)?;
        w.as_str().ok_or(fmt::Error)
    }
}
