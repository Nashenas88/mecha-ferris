#![no_std]
#![no_main]

use communication::COMMS_ADDR;
use core::iter::once;
use data::{calibrating_calibrations, make_joints};
#[cfg(feature = "defmt")]
use defmt::Format;
#[cfg(feature = "defmt")]
use defmt_rtt as _;
use embedded_hal::timer::CountDown;
use embedded_hal::watchdog::{Watchdog, WatchdogEnable};
use fugit::ExtU32;
use homing::Homing;
use kinematics::walking::{MechaLeg, VisitLeg, Walking};
use kinematics::{ComplexField, DefaultConsts, ExpensiveMath, LegError, Point3};
#[cfg(feature = "debug-current")]
use mecha_ferris::analog::read_external_current;
#[cfg(feature = "defmt")]
use mecha_ferris::analog_mux::{AnalogMux, CurrentSensor};
use mecha_ferris::comms::CommsManager;
use mecha_ferris::debouncer::Debouncer;
#[cfg(feature = "defmt")]
use mecha_ferris::flexible_input::FlexibleInput;
use mecha_ferris::joint::Joint;
use mecha_ferris::{log, State};
use once_cell::unsync::Lazy;
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use panic_probe as _;
#[cfg(feature = "debug-current")]
use pimoroni_servo2040::hal::adc::AdcPin;
use pimoroni_servo2040::hal::clocks::SystemClock;
use pimoroni_servo2040::hal::dma::{Channel, ChannelIndex, DMAExt, CH0, CH1};
use pimoroni_servo2040::hal::gpio::bank0::{Gpio18, Gpio20, Gpio21, Gpio26};
#[cfg(feature = "defmt")]
use pimoroni_servo2040::hal::gpio::FunctionSioOutput;
use pimoroni_servo2040::hal::gpio::{
    Function, FunctionI2C, FunctionPio0, FunctionPio1, Pin, PinId, PullNone, PullType, PullUp,
    ValidFunction,
};
use pimoroni_servo2040::hal::i2c::peripheral::I2CPeripheralEventIterator;
use pimoroni_servo2040::hal::pio::{PIOExt, StateMachineIndex, UninitStateMachine, PIO, SM0};
use pimoroni_servo2040::hal::rom_data::float_funcs;
use pimoroni_servo2040::hal::timer::{Alarm, Alarm0, Instant};
use pimoroni_servo2040::hal::{self, pac, Clock, Timer};
use pimoroni_servo2040::pac::{interrupt, I2C0, PIO0};
use servo_pio::calibration::{AngularCalibration, CalibrationData, Point};
use servo_pio::pwm_cluster::{dma_interrupt, DynPin, GlobalState, GlobalStates, Handler};
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
const UPDATE_MS: u32 = 50;

mod data;
mod homing;

static mut SERVO_CLUSTER: Option<ServoCluster<NUM_SERVOS, PIO0, SM0, AngularCalibration>> = None;

type I2CPeripheral = I2CPeripheralEventIterator<
    I2C0,
    (
        Pin<Gpio20, FunctionI2C, PullUp>,
        Pin<Gpio21, FunctionI2C, PullUp>,
    ),
>;

#[rtic::app(device = pimoroni_servo2040::hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod app {
    use pimoroni_servo2040::hal::i2c::I2CIrq;

    use super::*;

    #[shared]
    struct Shared {
        #[lock_free]
        state: State<NUM_SERVOS_PER_LEG, NUM_LEGS>,
    }

    #[local]
    struct Local {
        ws: Ws2812Direct<pac::PIO1, SM0, Pin<Gpio18, FunctionPio1, PullNone>>,
        i2c_peripheral: I2CPeripheral,
        alarm0: Alarm0,
        timer: Timer,
        watchdog: hal::Watchdog,
        last_update: Instant,
        #[cfg(feature = "debug-current")]
        adc: hal::Adc,
        #[cfg(feature = "debug-current")]
        external_voltage_sense:
            hal::adc::AdcPin<Pin<Gpio26, hal::gpio::FunctionSioInput, PullNone>>,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut watchdog = hal::Watchdog::new(c.device.WATCHDOG);
        let mut resets = c.device.RESETS;

        let clocks = hal::clocks::init_clocks_and_plls(
            pimoroni_servo2040::XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = hal::Sio::new(c.device.SIO);
        let pins = pimoroni_servo2040::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let calibrations = data::calibrations();
        let servo_pins: [_; NUM_SERVOS] = [
            ServoData {
                pin: reconfigure(pins.servo1),
                calibration: calibrations[0][0].cal,
            },
            ServoData {
                pin: reconfigure(pins.servo2),
                calibration: calibrations[0][1].cal,
            },
            ServoData {
                pin: reconfigure(pins.servo3),
                calibration: calibrations[0][2].cal,
            },
            ServoData {
                pin: reconfigure(pins.servo4),
                calibration: calibrations[1][0].cal,
            },
            ServoData {
                pin: reconfigure(pins.servo5),
                calibration: calibrations[1][1].cal,
            },
            ServoData {
                pin: reconfigure(pins.servo6),
                calibration: calibrations[1][2].cal,
            },
            ServoData {
                pin: reconfigure(pins.servo7),
                calibration: calibrations[2][0].cal,
            },
            ServoData {
                pin: reconfigure(pins.servo8),
                calibration: calibrations[2][1].cal,
            },
            ServoData {
                pin: reconfigure(pins.servo9),
                calibration: calibrations[2][2].cal,
            },
            ServoData {
                pin: reconfigure(pins.servo10),
                calibration: calibrations[3][0].cal,
            },
            ServoData {
                pin: reconfigure(pins.servo11),
                calibration: calibrations[3][1].cal,
            },
            ServoData {
                pin: reconfigure(pins.servo12),
                calibration: calibrations[3][2].cal,
            },
            ServoData {
                pin: reconfigure(pins.servo13),
                calibration: calibrations[4][0].cal,
            },
            ServoData {
                pin: reconfigure(pins.servo14),
                calibration: calibrations[4][1].cal,
            },
            ServoData {
                pin: reconfigure(pins.servo15),
                calibration: calibrations[4][2].cal,
            },
            ServoData {
                pin: reconfigure(pins.servo16),
                calibration: calibrations[5][0].cal,
            },
            ServoData {
                pin: reconfigure(pins.servo17),
                calibration: calibrations[5][1].cal,
            },
            ServoData {
                pin: reconfigure(pins.servo18),
                calibration: calibrations[5][2].cal,
            },
        ];

        let (mut pio0, sm0, _, _, _) = c.device.PIO0.split(&mut resets);
        // Use a different pio for the leds because they run at a different
        // clock speed.
        let (mut pio1, sm10, _, _, _) = c.device.PIO1.split(&mut resets);
        let dma = c.device.DMA.split(&mut resets);

        // Configure the Timer peripheral in count-down mode.
        let mut timer = hal::Timer::new(c.device.TIMER, &mut resets, &clocks);
        let mut count_down = timer.count_down();

        let mut ws = Ws2812Direct::new(
            pins.led_data.reconfigure::<FunctionPio1, PullNone>(),
            &mut pio1,
            sm10,
            clocks.peripheral_clock.freq(),
        );

        let i2c_peripheral = hal::i2c::I2C::new_peripheral_event_iterator_with_irq(
            c.device.I2C0,
            pins.sda.reconfigure::<FunctionI2C, PullUp>(),
            pins.scl.reconfigure::<FunctionI2C, PullUp>(),
            &mut resets,
            COMMS_ADDR,
            &[
                I2CIrq::StartDet,
                I2CIrq::RestartDet,
                I2CIrq::RdReq,
                I2CIrq::StopDet,
            ],
        );

        let servo_cluster = match build_servo_cluster(
            &mut pio0,
            sm0,
            (dma.ch0, dma.ch1),
            servo_pins,
            #[cfg(feature = "debug_pio")]
            pins.scl
                .reconfigure::<FunctionPio0, PullNone>()
                .into_dyn_pin(),
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

        count_down.start(1.secs());
        let _ = nb::block!(count_down.wait());
        #[cfg(any(feature = "defmt", feature = "debug-current"))]
        let mut adc = hal::Adc::new(c.device.ADC, &mut resets);
        #[cfg(feature = "defmt")]
        {
            let sensor_delay = 1.millis();

            let mut analog_mux = AnalogMux::new(
                pins.adc_addr_0.reconfigure::<FunctionSioOutput, PullNone>(),
                pins.adc_addr_1.reconfigure::<FunctionSioOutput, PullNone>(),
                pins.adc_addr_2.reconfigure::<FunctionSioOutput, PullNone>(),
                FlexibleInput::from(pins.shared_adc.into_floating_input()),
            );

            analog_mux.with_reader::<CurrentSensor, _>(
                &mut count_down,
                |count_down, mut analog| {
                    let mut current = 0.0;
                    for _ in 0..SAMPLES {
                        current += analog.read_current(&mut adc);
                        count_down.start(sensor_delay);
                        let _ = nb::block!(count_down.wait());
                    }
                    current /= SAMPLES as f32;
                    log::info!("servos pulling {}A", current);
                },
            );
        }

        #[cfg(feature = "debug-current")]
        let external_voltage_sense = AdcPin::new(pins.adc0.into_floating_input());

        let mut robot_state =
            RobotState::new(make_joints(servos1, calibrations, calibrating_calibrations));
        robot_state.body_translation.y = 170.0;
        robot_state.leg_radius = 250.0;
        robot_state.animation_factor = 2.0;
        let mut state = State::new(robot_state);

        let _ = ws.write(brightness(
            [black(), black(), black(), black(), black(), white()].into_iter(),
            LED_BRIGHTNESS,
        ));

        log::info!("Waiting for button press...");
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

        log::info!("Button pressed. Starting event loop");
        let _ = ws.write(brightness([purple(); 6].into_iter(), LED_BRIGHTNESS));
        state.live_state.state_machine = state::StateMachine::Homing;

        // The alarm used to trigger the matrix scan.
        let mut alarm0 = timer.alarm_0().unwrap();
        alarm0.enable_interrupt();
        // Allow the event loop to start immediately after init is complete.
        rtic::pend(pac::Interrupt::TIMER_IRQ_0);

        // Do not enable watchdog when debugging.
        #[cfg(not(feature = "debug"))]
        {
            // Start watchdog and feed it with the lowest priority task at 50khz
            watchdog.start(50_000.micros());
        }

        unsafe {
            SERVO_CLUSTER = Some(servo_cluster);
        }

        (
            Shared { state },
            Local {
                ws,
                i2c_peripheral,
                alarm0,
                timer,
                watchdog,
                last_update: timer.get_counter(),
                #[cfg(feature = "debug-current")]
                adc,
                #[cfg(feature = "debug-current")]
                external_voltage_sense,
            },
            init::Monotonics(),
        )
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(
        binds = I2C0_IRQ,
        shared = [state],
        local = [
            i2c_peripheral,
            i2c_manager: CommsManager<NUM_SERVOS_PER_LEG, NUM_LEGS> = CommsManager::new(),
        ],
        priority = 1,
    )]
    fn i2c(c: i2c::Context) {
        let i2c::LocalResources {
            i2c_peripheral,
            i2c_manager,
            ..
        } = c.local;
        let state = c.shared.state;
        let _updated = i2c_manager.run_loop(i2c_peripheral, state);
    }

    #[task(
        binds = TIMER_IRQ_0,
        shared = [state],
        local = [
            alarm0,
            timer,
            watchdog,
            adc,
            external_voltage_sense,
            last_update,
            ws,
            homing: Homing = Homing::new(),
            walking: Lazy<Walking<RomFuncs, DefaultConsts>> = Lazy::new(|| Walking::new(75.0)),
            delay: Option<f32> = None,
            time: f32 = 0.0,
        ],
        priority = 1,
    )]
    fn kinematics(c: kinematics::Context) {
        let state = c.shared.state;
        let kinematics::LocalResources {
            alarm0,
            timer,
            watchdog,
            last_update,
            ws,
            homing,
            walking,
            #[cfg(feature = "debug-current")]
            adc,
            #[cfg(feature = "debug-current")]
            external_voltage_sense,
            time,
            delay,
            ..
        } = c.local;
        alarm0.clear_interrupt();
        let _ = alarm0.schedule(UPDATE_MS.millis());

        let servo_cluster = unsafe { SERVO_CLUSTER.as_mut().unwrap() };
        let mut step_delay = delay.unwrap_or_else(|| walking.duration() as f32);

        let start = timer.get_counter();
        let diff = (start - *last_update).to_millis();
        *last_update = start;

        // TODO add animated transition between states.
        let update = {
            if state.from_previous_state.is_some() {
                state.from_previous_state = None;
                let color = match state.live_state.state_machine {
                    state::StateMachine::Paused => cyan(),
                    state::StateMachine::Homing => {
                        homing.reset();
                        yellow()
                    }
                    state::StateMachine::Calibrating => blue(),
                    state::StateMachine::Looping => red(),
                    state::StateMachine::Exploring => green(),
                };
                // Update LEDs to show current state.
                let _ = ws.write(brightness([color; 6].into_iter(), LED_BRIGHTNESS));
            }

            match state.live_state.state_machine {
                state::StateMachine::Paused => false,
                state::StateMachine::Homing => {
                    homing.update(diff, servo_cluster, &state.live_state.joints)
                }
                state::StateMachine::Calibrating => {
                    *last_update = start;
                    state.update(diff as f32, servo_cluster);
                    true
                }
                state::StateMachine::Looping | state::StateMachine::Exploring => {
                    *last_update = start;
                    *time += diff as f32 / 1000.0;
                    if *time > step_delay {
                        let (_, delay) = walking.next_animation();
                        *time %= step_delay;
                        step_delay = delay as f32 * state.live_state.animation_factor;
                    }
                    let mut leg_visitor = LegVisitor {
                        servo_cluster,
                        joints: &state.live_state.joints,
                        position_start: Instant::from_ticks(0),
                        servo_start: Instant::from_ticks(0),
                        timer,
                        errors: [false; NUM_LEGS],
                    };
                    walking.walk(&state.live_state, &mut leg_visitor, *time / step_delay);
                    state.update(diff as f32, servo_cluster);
                    true
                }
            }
        };

        *delay = Some(step_delay);

        if update {
            servo_cluster.load();
        }

        #[cfg(feature = "debug-current")]
        {
            let current = read_external_current(adc, external_voltage_sense);
            log::info!("Current reading: {}", current);
        }

        #[cfg(not(feature = "debug"))]
        {
            watchdog.feed();
        }
    }
}

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
        // (Self::sin(f), Self::cos(f))
        let res = float_funcs::fsincos(f);
        (res.sin, res.cos)
    }

    #[inline(always)]
    fn sqrt(f: f32) -> f32 {
        float_funcs::fsqrt(f)
    }
}

/// Reconfigure servo pins so they can be used by the servo cluster.
fn reconfigure<I, F, P>(pin: Pin<I, F, P>) -> DynPin<FunctionPio0>
where
    I: PinId + ValidFunction<FunctionPio0>,
    F: Function,
    P: PullType,
{
    pin.reconfigure::<FunctionPio0, PullNone>().into_dyn_pin()
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
    Build(ServoClusterBuilderError),
}

fn build_servo_cluster<C1, C2, P, SM>(
    pio: &mut PIO<P>,
    sm: UninitStateMachine<(P, SM)>,
    dma_channels: (Channel<C1>, Channel<C2>),
    servo_data: [ServoData<AngularCalibration, FunctionPio0>; NUM_SERVOS],
    #[cfg(feature = "debug_pio")] side_set_pin: DynPin,
    system_clock: SystemClock,
    state: &'static mut Option<GlobalState<C1, C2, P, SM>>,
) -> Result<ServoCluster<NUM_SERVOS, P, SM, AngularCalibration>, BuildError>
where
    C1: ChannelIndex,
    C2: ChannelIndex,
    P: PIOExt<PinFunction = FunctionPio0>,
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
        FunctionPio0,
        NUM_SERVOS,
        NUM_CHANNELS,
    > = ServoCluster::<NUM_SERVOS, P, SM, AngularCalibration>::builder(
        pio,
        sm,
        dma_channels,
        unsafe { &mut GLOBALS },
    )
    .pins_and_calibration(servo_data);
    #[cfg(feature = "debug_pio")]
    {
        builder = builder.side_set_pin(side_set_pin);
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
