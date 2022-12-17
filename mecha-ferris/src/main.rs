#![no_std]
#![no_main]

use core::iter::once;
use defmt::Format;
use defmt_rtt as _;
use embedded_hal::timer::CountDown;
use fugit::ExtU64;
use kinematics::{ComplexField, Leg, Point3};
use mecha_ferris::analog_mux::{AnalogMux, CurrentSensor};
use mecha_ferris::flexible_input::FlexibleInput;
use panic_probe as _;
use pimoroni_servo2040::hal::clocks::SystemClock;
use pimoroni_servo2040::hal::dma::{Channel, ChannelIndex, DMAExt, CH0, CH1};
use pimoroni_servo2040::hal::gpio::{Error as GpioError, FunctionConfig, FunctionPio0};
use pimoroni_servo2040::hal::pio::{PIOExt, StateMachineIndex, UninitStateMachine, PIO, SM0};
use pimoroni_servo2040::hal::{self, pac, Clock};
use pimoroni_servo2040::pac::{interrupt, PIO0};
use servo_pio::calibration::{AngularCalibration, Calibration, CalibrationData, Point};
use servo_pio::pwm_cluster::{dma_interrupt, GlobalState, GlobalStates, Handler};
use servo_pio::servo_cluster::{
    ServoCluster, ServoClusterBuilder, ServoClusterBuilderError, ServoData,
};
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812Direct;

const LED_BRIGHTNESS: u8 = 16;
const NUM_SERVOS: usize = 3;
const NUM_CHANNELS: usize = 12;
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
const SAMPLES: usize = 10;

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

    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);
    let mut analog_mux = AnalogMux::new(
        pins.adc_addr_0.into_mode(),
        pins.adc_addr_1.into_mode(),
        pins.adc_addr_2.into_mode(),
        FlexibleInput::from(pins.shared_adc.into_floating_input()),
    );

    let servo_pins: [_; NUM_SERVOS] = [
        ServoData {
            pin: pins.servo3.into_mode::<FunctionPio0>().into(),
            calibration: Calibration::builder(AngularCalibration::new(
                Point {
                    pulse: 780.0,
                    value: 0.0,
                },
                Point {
                    pulse: 1220.0,
                    value: 67.5,
                },
                Point {
                    pulse: 2110.0,
                    value: 157.5,
                },
            ))
            .limit_lower()
            .limit_upper()
            .build(),
        },
        ServoData {
            pin: pins.servo4.into_mode::<FunctionPio0>().into(),
            calibration: Calibration::builder(AngularCalibration::new(
                Point {
                    pulse: 2275.0,
                    value: -67.5,
                },
                Point {
                    pulse: 1790.0,
                    value: 0.0,
                },
                Point {
                    pulse: 890.0,
                    value: 90.0,
                },
            ))
            .limit_lower()
            .limit_upper()
            .build(),
        },
        ServoData {
            pin: pins.servo5.into_mode::<FunctionPio0>().into(),
            calibration: Calibration::builder(AngularCalibration::new(
                Point {
                    pulse: 2423.0,
                    value: 0.0,
                },
                Point {
                    pulse: 1538.0,
                    value: 90.0,
                },
                Point {
                    pulse: 900.0,
                    value: 180.0,
                },
            ))
            .limit_lower()
            .limit_upper()
            .build(),
        },
    ];

    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    // Use a different pio for the leds because they run at a different
    // clock speed.
    let (mut pio1, sm10, _, _, _) = pac.PIO1.split(&mut pac.RESETS);
    let dma = pac.DMA.split(&mut pac.RESETS);

    // Configure the Timer peripheral in count-down mode.
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut count_down = timer.count_down();

    let mut ws = Ws2812Direct::new(
        pins.led_data.into_mode(),
        &mut pio1,
        sm10,
        clocks.peripheral_clock.freq(),
    );

    let mut leg: Leg = Leg::new();

    let mut servo_cluster = match build_servo_cluster(
        &mut pio0,
        sm0,
        (dma.ch0, dma.ch1),
        servo_pins,
        #[cfg(feature = "debug_pio")]
        pins.scl.into_mode::<FunctionPio0>().into(),
        clocks.system_clock,
        unsafe { &mut STATE1 },
    ) {
        Ok(cluster) => cluster,
        Err(e) => {
            defmt::error!("Failed to build servo cluster: {:?}", e);
            let _ = ws.write(brightness(
                once(RGB8 { r: 255, g: 0, b: 0 }),
                LED_BRIGHTNESS,
            ));
            #[allow(clippy::empty_loop)]
            loop {}
        }
    };

    // Unmask the DMA interrupt so the handler can start running. This can only
    // be done after the servo cluster has been built.
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::DMA_IRQ_0);
    }

    // let movement_delay = 10.millis();
    let sensor_delay = 1.millis();

    // We need to use the indices provided by the cluster because the servo pin
    // numbers do not line up with the indices in the clusters and PIO.
    let [servo1, servo2, servo3] = servo_cluster.servos();

    count_down.start(1.secs());
    let _ = nb::block!(count_down.wait());
    {
        let mut analog = analog_mux.reader::<CurrentSensor>(&mut count_down);
        let mut current = 0.0;
        for _ in 0..SAMPLES {
            current += analog.read_current(&mut adc);
            count_down.start(sensor_delay);
            let _ = nb::block!(count_down.wait());
        }
        current /= SAMPLES as f32;
        defmt::info!("servos pulling {}A", current);
    }

    let mut time = 0.0;
    let step_delay = 0.25;

    let mut now = timer.get_counter();

    loop {
        let last_update = now;
        now = timer.get_counter();
        time += (now - last_update).to_millis() as f32 / 1000.0;
        if time > step_delay {
            time = 0.0;
        }
        let circle_angle = time / step_delay * 2.0 * core::f32::consts::PI;
        let target = point_on_circle(0.0, 150.0, 25.0, circle_angle as f32);
        match leg.go_to(target) {
            Ok(()) => {
                let coxa_servo_angle = rad_to_deg(leg.coxa_servo_angle());
                let femur_servo_angle = rad_to_deg(leg.femur_servo_angle());
                let tibia_servo_angle = rad_to_deg(leg.tibia_servo_angle());
                servo_cluster.set_value(servo1, tibia_servo_angle, false);
                servo_cluster.set_value(servo2, femur_servo_angle, false);
                servo_cluster.set_value(servo3, coxa_servo_angle, false);
                servo_cluster.load();
                let coxa_color = RGB8 {
                    r: coxa_servo_angle as u8,
                    g: 0,
                    b: 0,
                };
                let femur_color = RGB8 {
                    r: 0,
                    g: (femur_servo_angle + 90.0) as u8,
                    b: 0,
                };
                let tibia_color = RGB8 {
                    r: 0,
                    g: 0,
                    b: tibia_servo_angle as u8,
                };
                let _ = ws.write(brightness(
                    [
                        coxa_color,
                        coxa_color,
                        femur_color,
                        femur_color,
                        tibia_color,
                        tibia_color,
                    ]
                    .into_iter(),
                    LED_BRIGHTNESS,
                ));
            }
            Err(e) => {
                defmt::error!("Unable to reach target: {}", e);
            }
        }
    }
}

fn point_on_circle(z: f32, x: f32, radius: f32, angle: f32) -> Point3<f32> {
    Point3::new(x + radius * angle.sin(), 0.0, z + radius * angle.cos())
}

fn rad_to_deg(rad: f32) -> f32 {
    rad / core::f32::consts::PI * 180.0
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
    #[cfg(feature = "debug_pio")] side_set_pin: DynPin,
    system_clock: SystemClock,
    state: &'static mut Option<GlobalState<C1, C2, P, SM>>,
) -> Result<ServoCluster<NUM_SERVOS, P, SM, AngularCalibration>, BuildError>
where
    C1: ChannelIndex,
    C2: ChannelIndex,
    P: PIOExt + FunctionConfig,
    SM: StateMachineIndex,
{
    #[allow(unused_mut)]
    let mut builder: ServoClusterBuilder<
        '_,
        AngularCalibration,
        C1,
        C2,
        P,
        SM,
        NUM_SERVOS,
        NUM_CHANNELS,
    > = ServoCluster::<NUM_SERVOS, P, SM, AngularCalibration>::builder(
        pio,
        sm,
        dma_channels,
        unsafe { &mut GLOBALS },
    )
    .pins_and_calibration(servo_data)
    .map_err(BuildError::Gpio)?;
    #[cfg(feature = "debug_pio")]
    {
        builder = builder
            .side_set_pin(side_set_pin)
            .map_err(BuildError::Gpio)?;
    }
    builder
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
