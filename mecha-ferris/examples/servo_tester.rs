//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use core::iter::once;
use defmt::Format;
use defmt_rtt as _;
use embedded_hal::adc::OneShot;
use embedded_hal::timer::CountDown;
use fugit::ExtU64;
use panic_probe as _;
use pimoroni_servo2040::hal::clocks::SystemClock;
use pimoroni_servo2040::hal::dma::{Channel, ChannelIndex, DMAExt, CH0, CH1};
use pimoroni_servo2040::hal::gpio::{DynPin, Error as GpioError, FunctionConfig, FunctionPio0};
use pimoroni_servo2040::hal::pio::{PIOExt, StateMachineIndex, UninitStateMachine, PIO, SM0};
use pimoroni_servo2040::hal::{self, pac, Clock};
use pimoroni_servo2040::pac::{interrupt, PIO0};
use servo_pio::calibration::{map_float, AngularCalibration, Calibration};
use servo_pio::pwm_cluster::{dma_interrupt, GlobalState, GlobalStates, Handler};
use servo_pio::servo_cluster::{ServoCluster, ServoClusterBuilder, ServoClusterBuilderError};
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812Direct;

const LED_BRIGHTNESS: u8 = 16;
const NUM_SERVOS: usize = 1;
const NUM_CHANNELS: usize = 1;
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
    let servo_pins: [_; NUM_SERVOS] = [pins.servo4.into_mode::<FunctionPio0>().into()];
    let mut pot_pin = pins.adc0.into_floating_input();

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

    const MIN_PULSE: f32 = 1000.0;
    const MAX_PULSE: f32 = 3000.0;
    const INITIAL_PULSE: f32 = 2000.0;
    let movement_delay = 10.millis();

    // We need to use the indices provided by the cluster because the servo pin
    // numbers do not line up with the indices in the clusters and PIO.
    let [servo1] = servo_cluster.servos();

    servo_cluster.set_pulse(servo1, INITIAL_PULSE, true);

    loop {
        let pot: u16 = adc.read(&mut pot_pin).unwrap();
        defmt::info!("pot value: {}", pot);
        let new_pulse = map_float(pot as f32, 0.0, 4096.0, MIN_PULSE, MAX_PULSE);
        defmt::info!("pulse value: {}", new_pulse);
        servo_cluster.set_pulse(servo1, new_pulse, true);
        count_down.start(movement_delay);
        let _ = nb::block!(count_down.wait());
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
    servo_pins: [DynPin; NUM_SERVOS],
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
    ServoCluster::builder(pio, sm, dma_channels, unsafe { &mut GLOBALS })
        .pins_and_calibration(servo_pins)
        .map_err(BuildError::Gpio)?
        .calibrations([Some(Calibration::new())])
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
