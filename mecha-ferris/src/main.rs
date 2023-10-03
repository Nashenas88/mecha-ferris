#![no_std]
#![no_main]

use communication::COMMS_ADDR;
use data::{calibrating_calibrations, make_joints};
use homing::Homing;
use mecha_ferris::analog::read_external_current;
use mecha_ferris::comms::CommsManager;
use mecha_ferris::debouncer::Debouncer;
use mecha_ferris::joint::Joint;
use mecha_ferris::{log, State};

use core::iter::once;
#[cfg(feature = "defmt")]
use defmt::Format;
#[cfg(feature = "defmt")]
use defmt_rtt as _;
use embedded_hal::timer::CountDown;
use fugit::ExtU64;
use kinematics::walking::{MechaLeg, VisitLeg, Walking};
use kinematics::{ComplexField, DefaultConsts, ExpensiveMath, LegError, Point3};
#[cfg(feature = "defmt")]
use mecha_ferris::analog_mux::{AnalogMux, CurrentSensor};
#[cfg(feature = "defmt")]
use mecha_ferris::flexible_input::FlexibleInput;
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use panic_probe as _;
use pimoroni_servo2040::hal::clocks::SystemClock;
use pimoroni_servo2040::hal::dma::{Channel, ChannelIndex, DMAExt, CH0, CH1};
use pimoroni_servo2040::hal::gpio::{Error as GpioError, FunctionConfig, FunctionPio0};
use pimoroni_servo2040::hal::pio::{PIOExt, StateMachineIndex, UninitStateMachine, PIO, SM0};
use pimoroni_servo2040::hal::rom_data::float_funcs;
use pimoroni_servo2040::hal::timer::Instant;
use pimoroni_servo2040::hal::{self, pac, Clock, Timer};
use pimoroni_servo2040::pac::{interrupt, PIO0};
use servo_pio::calibration::{AngularCalibration, CalibrationData, Point};
use servo_pio::pwm_cluster::{dma_interrupt, GlobalState, GlobalStates, Handler};
use servo_pio::servo_cluster::{
    ServoCluster, ServoClusterBuilder, ServoClusterBuilderError, ServoData,
};
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812Direct;

const LED_BRIGHTNESS: u8 = 8;
const NUM_SERVOS_PER_LEG: usize = 3;
const NUM_LEGS: usize = 6;
const NUM_SERVOS: usize = NUM_SERVOS_PER_LEG * NUM_LEGS;
const NUM_CHANNELS: usize = 2;

type RobotState = state::RobotState<Joint, NUM_SERVOS_PER_LEG, NUM_LEGS>;

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

#[cfg(feature = "defmt")]
const SAMPLES: usize = 10;
const UPDATE_MS: u64 = 50;

mod data;
mod homing;

struct RomFuncs;
impl ExpensiveMath<f32> for RomFuncs {
    #[inline(always)]
    fn atan2(l: f32, r: f32) -> f32 {
        float_funcs::fatan2(l, r)
    }

    fn acos(f: f32) -> f32 {
        // Handbook of Mathematical Functions
        // M. Abramowitz and I.A. Stegun, Ed.

        // Absolute error <= 6.7e-5
        let negate = (f < 0.0) as u8 as f32;
        let f = f32::abs(f);
        let ret = -0.0187293 * f
            + 0.0742610 * f
            + -0.2121144 * f
            + 1.5707288 * float_funcs::fsqrt(1.0 - f);
        negate * core::f32::consts::PI + ret + -2.0 * negate * ret
    }

    #[inline(always)]
    fn sin(f: f32) -> f32 {
        float_funcs::fsin(f)
    }

    #[inline(always)]
    fn cos(f: f32) -> f32 {
        float_funcs::fcos(f)
    }

    #[inline(always)]
    fn sincos(f: f32) -> (f32, f32) {
        (Self::sin(f), Self::cos(f))
        // float_funcs::fsincos(f)
    }

    #[inline(always)]
    fn sqrt(f: f32) -> f32 {
        float_funcs::fsqrt(f)
    }
}

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

    let calibrations = data::calibrations();
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

    let mut i2c_peripheral = hal::i2c::I2C::new_peripheral_event_iterator(
        pac.I2C0,
        pins.sda.into_mode(),
        pins.scl.into_mode(),
        &mut pac.RESETS,
        COMMS_ADDR,
    );

    let mut walking = Walking::<RomFuncs, DefaultConsts>::new(75.0);

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
            log::error!("Failed to build servo cluster: {:?}", e);
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

    // We need to use the indices provided by the cluster because the servo pin
    // numbers do not line up with the indices in the clusters and PIO.
    #[rustfmt::skip]
    let [
        servo1, servo2, servo3,
        servo4, servo5, servo6,
        servo7, servo8, servo9,
        servo10, servo11, servo12,
        servo13, servo14, servo15,
        servo16, servo17, servo18,
    ] = servo_cluster.servos();
    let servos1 = [
        [servo1, servo2, servo3],
        [servo4, servo5, servo6],
        [servo7, servo8, servo9],
        [servo10, servo11, servo12],
        [servo13, servo14, servo15],
        [servo16, servo17, servo18],
    ];
    let calibrating_calibrations = calibrating_calibrations();
    let mut joints = make_joints(servos1, calibrations, calibrating_calibrations);

    count_down.start(1.secs());
    let _ = nb::block!(count_down.wait());
    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);
    #[cfg(feature = "defmt")]
    {
        let sensor_delay = 1.millis();

        let mut analog_mux = AnalogMux::new(
            pins.adc_addr_0.into_mode(),
            pins.adc_addr_1.into_mode(),
            pins.adc_addr_2.into_mode(),
            FlexibleInput::from(pins.shared_adc.into_floating_input()),
        );

        let mut analog = analog_mux.reader::<CurrentSensor>(&mut count_down);
        let mut current = 0.0;
        for _ in 0..SAMPLES {
            current += analog.read_current(&mut adc);
            count_down.start(sensor_delay);
            let _ = nb::block!(count_down.wait());
        }
        current /= SAMPLES as f32;
        log::info!("servos pulling {}A", current);
    }

    let mut external_voltage_sense = pins.adc0.into_floating_input();

    let mut time = 0.0;
    let mut step_delay = walking.duration() as f32;
    let mut i2c_manager = CommsManager::new();
    let mut robot_state = RobotState::new();
    robot_state.body_translation.y = 170.0;
    robot_state.leg_radius = 250.0;
    robot_state.animation_factor = 2.0;
    let mut state = State::new(robot_state);

    let _ = ws.write(brightness(
        [black(), black(), black(), black(), black(), white()].into_iter(),
        LED_BRIGHTNESS,
    ));

    // Wait for button press to initialize servos.
    let user_sw = pins.user_sw.into_pull_up_input();
    let mut collect_debouncer = Debouncer::new(5, user_sw);
    loop {
        if collect_debouncer.update() && collect_debouncer.is_low() {
            break;
        }

        count_down.start(50.millis());
        let _ = nb::block!(count_down.wait());
    }

    let mut now = timer.get_counter();
    let _ = ws.write(brightness([purple(); 6].into_iter(), LED_BRIGHTNESS));
    let mut last_update = now;
    state.live_state.state_machine = state::StateMachine::Homing;
    let mut homing = Homing::new();

    loop {
        now = timer.get_counter();
        if i2c_manager.run_loop(&mut i2c_peripheral, &mut state) {
            let color = match state.live_state.state_machine {
                state::StateMachine::Paused => cyan(),
                state::StateMachine::Homing => {
                    last_update = now;
                    homing.reset();
                    yellow()
                }
                state::StateMachine::Calibrating => blue(),
                state::StateMachine::Looping | state::StateMachine::Exploring => {
                    last_update = now;
                    green()
                }
            };
            let _ = ws.write(brightness([color; 6].into_iter(), LED_BRIGHTNESS));
        }

        let diff = (now - last_update).to_millis();
        time += diff as f32 / 1000.0;

        let update = match state.live_state.state_machine {
            state::StateMachine::Paused => false,
            state::StateMachine::Homing => {
                let update = homing.update(diff, &mut servo_cluster, &joints);
                if update {
                    last_update = now;
                }
                update
            }
            state::StateMachine::Calibrating => {
                // last_update = now;
                // true
                state.update(diff as f32, &mut servo_cluster, &mut joints);
                false
            }
            state::StateMachine::Looping | state::StateMachine::Exploring => {
                last_update = now;
                if time > step_delay {
                    let (_, delay) = walking.next_animation();
                    time %= step_delay;
                    step_delay = delay as f32 * state.live_state.animation_factor;
                }
                let mut leg_visitor = LegVisitor {
                    servo_cluster: &mut servo_cluster,
                    joints: &joints,
                    position_start: Instant::from_ticks(0),
                    servo_start: Instant::from_ticks(0),
                    timer: &timer,
                    errors: [false; 6],
                };
                walking.walk(&state.live_state, &mut leg_visitor, time / step_delay);

                // Update led color based on whether any errors were encountered for that particular leg.
                let _ = ws.write(brightness(
                    leg_visitor
                        .errors
                        .into_iter()
                        .map(|e| if e { red() } else { green() }),
                    LED_BRIGHTNESS,
                ));
                state.update(diff as f32, &mut servo_cluster, &mut joints);
                true
            }
        };

        if update {
            servo_cluster.load();
        }

        #[cfg(feature = "debug-current")]
        {
            let current = read_external_current(&mut adc, &mut external_voltage_sense);
            log::info!("Current reading: {}", current);
        }

        count_down.start((UPDATE_MS - diff).min(0).millis());
        let _ = nb::block!(count_down.wait());
    }
}

fn red() -> RGB8 {
    RGB8 { r: 255, g: 0, b: 0 }
}

fn green() -> RGB8 {
    RGB8 { r: 0, g: 255, b: 0 }
}

fn blue() -> RGB8 {
    RGB8 { r: 0, g: 0, b: 255 }
}

fn yellow() -> RGB8 {
    RGB8 {
        r: 255,
        g: 255,
        b: 0,
    }
}

fn purple() -> RGB8 {
    RGB8 {
        r: 255,
        g: 0,
        b: 255,
    }
}

fn black() -> RGB8 {
    RGB8 { r: 0, g: 0, b: 0 }
}

fn cyan() -> RGB8 {
    RGB8 {
        r: 0,
        g: 255,
        b: 255,
    }
}

fn white() -> RGB8 {
    RGB8 {
        r: 255,
        g: 255,
        b: 255,
    }
}

struct LegVisitor<'a> {
    servo_cluster: &'a mut ServoCluster<NUM_SERVOS, PIO0, SM0, AngularCalibration>,
    joints: &'a [[Joint; NUM_SERVOS_PER_LEG]; NUM_LEGS],
    #[cfg_attr(not(feature = "debug-visitor"), allow(dead_code))]
    position_start: Instant,
    #[cfg_attr(not(feature = "debug-visitor"), allow(dead_code))]
    servo_start: Instant,
    #[cfg_attr(not(feature = "debug-visitor"), allow(dead_code))]
    timer: &'a Timer,
    errors: [bool; NUM_LEGS],
}

impl<'a> VisitLeg<f32, RomFuncs, DefaultConsts> for LegVisitor<'a> {
    fn before(&mut self, _: Point3<f32>, _: &MechaLeg<f32, RomFuncs, DefaultConsts>) {}

    #[cfg(feature = "debug-visitor")]
    fn on_error(&mut self, _: &MechaLeg<f32, RomFuncs, DefaultConsts>, e: LegError<f32>) {
        log::error!("Unable to reach target: {}", e);
    }

    #[cfg(not(feature = "debug-visitor"))]
    fn on_error(&mut self, _: &MechaLeg<f32, RomFuncs, DefaultConsts>, _: LegError<f32>) {}

    fn after(&mut self, _: Point3<f32>, leg: &MechaLeg<f32, RomFuncs, DefaultConsts>) {
        let coxa_servo_angle = rad_to_deg(leg.leg().coxa_servo_angle());
        let femur_servo_angle = rad_to_deg(leg.leg().femur_servo_angle());
        let tibia_servo_angle = rad_to_deg(leg.leg().tibia_servo_angle());
        // TODO removing this line causes faults to start triggering on the real hardware.
        // I wonder if removing this causes the LED's to be written to too quickly, or
        // if it's a problem with updating the servo data too quickly (maybe bug in DMA
        // mapping).
        // log::info!(
        //     "Moving to ({}, {}, {})",
        //     coxa_servo_angle,
        //     femur_servo_angle,
        //     tibia_servo_angle
        // );
        let idx = leg.idx();
        let [joint1, joint2, joint3] = &self.joints[idx as usize];
        self.servo_cluster
            .set_value(joint1.servo(), tibia_servo_angle, false);
        self.servo_cluster
            .set_value(joint2.servo(), femur_servo_angle, false);
        self.servo_cluster
            .set_value(joint3.servo(), coxa_servo_angle, false);
    }

    #[cfg(feature = "debug-visitor")]
    fn position_start(&mut self) {
        self.position_start = self.timer.get_counter();
    }

    #[cfg(feature = "debug-visitor")]
    fn position_end(&mut self) {
        let now = self.timer.get_counter();
        log::info!(
            "Calculating position took {}us",
            (now - self.position_start).to_micros() as f32
        )
    }

    #[cfg(feature = "debug-visitor")]
    fn servo_start(&mut self) {
        self.servo_start = self.timer.get_counter();
    }

    #[cfg(feature = "debug-visitor")]
    fn servo_end(&mut self) {
        let now = self.timer.get_counter();
        log::info!(
            "Calculating and moving legs took {}us",
            (now - self.servo_start).to_micros() as f32,
        )
    }

    #[cfg(not(feature = "debug-visitor"))]
    fn position_start(&mut self) {}

    #[cfg(not(feature = "debug-visitor"))]
    fn position_end(&mut self) {}

    #[cfg(not(feature = "debug-visitor"))]
    fn servo_start(&mut self) {}

    #[cfg(not(feature = "debug-visitor"))]
    fn servo_end(&mut self) {}
}

#[allow(dead_code)]
fn point_on_circle(z: f32, y: f32, x: f32, radius: f32, angle: f32) -> Point3<f32> {
    Point3::new(x + radius * angle.sin(), y, z + radius * angle.cos())
}

#[allow(dead_code)]
fn point_on_line(z: f32, y: f32, x: f32, length: f32, t: f32) -> Point3<f32> {
    Point3::new(x, y, z + length * t.cos())
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

#[cfg_attr(feature = "defmt", derive(Format))]
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
