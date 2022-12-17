#![no_std]
#![no_main]

use arduino_hal::I2c;
use panic_halt as _;
use pwm_pca9685::{Address, Channel, Pca9685};
use servo::{Mg90d, Servo};

use crate::servo::{ServoLimits, HS311};

mod servo;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        100_000,
    );

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    // We use 0x40 as an address as that is the first default address.
    let mut pwm = Pca9685::new(i2c, Address::from(0x40)).unwrap();
    // This results in 50Hz.
    // 122 * (4096*20000)= x
    pwm.set_prescale(121).unwrap();
    pwm.enable().unwrap();
    // Turn all channels on with a setting of "0".
    pwm.set_channel_on(Channel::All, 0).unwrap();

    // let mut led = pins.d13.into_output();

    let mut legs = [
        Leg::new(Servo::new(Channel::C0, 50), Servo::new(Channel::C1, 50)),
        Leg::new(Servo::new(Channel::C2, 50), Servo::new(Channel::C3, 50)),
        Leg::new(Servo::new(Channel::C4, 50), Servo::new(Channel::C5, 50)),
        Leg::new(Servo::new(Channel::C6, 50), Servo::new(Channel::C7, 50)),
        Leg::new(Servo::new(Channel::C8, 50), Servo::new(Channel::C9, 50)),
        Leg::new(Servo::new(Channel::C10, 50), Servo::new(Channel::C11, 50)),
    ];

    let mut claw = Servo::<HS311>::new(Channel::C12, 0);

    for leg in &mut legs {
        leg.shoulder.do_move(&mut pwm);
        leg.elbow.do_move(&mut pwm);
    }
    claw.do_move(&mut pwm);

    let mut a_anim = WalkingAnimation::new(WalkingPhase::LiftAndForward);
    let mut b_anim = WalkingAnimation::new(WalkingPhase::HoldDownAndBackward);

    // Hold middle position for 10 second before beginning walk.
    arduino_hal::delay_ms(2_000);
    loop {
        // led.toggle();
        arduino_hal::delay_ms(1);
        // led.toggle();
        // arduino_hal::delay_ms(500);
        // let mut updated = false;
        // for leg in &mut legs {
        //     updated = leg.shoulder.update();
        //     leg.elbow.update();
        // }

        // if !updated {
        //     for leg in &mut legs {
        //         leg.shoulder.flip_direction();
        //         leg.elbow.flip_direction();
        //         leg.shoulder.update();
        //         leg.elbow.update();
        //     }
        // }

        // if !claw.update() {
        //     claw.flip_direction();
        //     claw.update();
        // }

        // for leg in &legs {
        //     leg.shoulder.do_move(&mut pwm);
        //     leg.elbow.do_move(&mut pwm);
        // }
        // claw.do_move(&mut pwm);

        a_anim.do_move(&mut serial, &mut legs, &mut pwm);
    }
}

struct WalkingAnimation {
    phase: WalkingPhase,
    counter: u16,
}

impl WalkingAnimation {
    fn new(phase: WalkingPhase) -> Self {
        Self { phase, counter: 0 }
    }

    fn do_move<S>(&mut self, serial: &mut S, legs: &mut [Leg; 6], pwm: &mut Pca9685<I2c>)
    where
        S: ufmt::uWrite,
    {
        const RES2: u16 = RESOLUTION as u16 >> 1;
        if self.counter == RESOLUTION as u16 {
            self.counter = 0;
            self.switch_phase();
        } else {
            if self.counter == RES2 {
                self.switch_phase();
            }
        }

        fn clamp_pct(pct: u16) -> u16 {
            if pct > 90 {
                90
            } else if pct < 10 {
                10
            } else {
                pct
            }
        }

        let (a_shoulder_pct, a_elbow_pct, b_shoulder_pct, b_elbow_pct) = match self.phase {
            WalkingPhase::LiftAndForward => {
                // *7>>10 ~= /150
                let pct = clamp_pct((self.counter as u32 * 100 * 7 >> 10) as u16);
                (
                    pct as u16,
                    100 - SIN_LOOKUP_TABLE[self.counter as usize * 2],
                    100 - pct as u16,
                    100,
                )
            }
            WalkingPhase::HoldDownAndBackward => {
                let counter = self.counter - RES2;
                // *7>>10 ~= /150
                let pct = clamp_pct((counter as u32 * 100 * 7 >> 10) as u16);
                (
                    100 - pct as u16,
                    100,
                    pct as u16,
                    100 - SIN_LOOKUP_TABLE[counter as usize * 2],
                )
            }
        };

        fn move_leg(shoulder_pct: u16, elbow_pct: u16, leg: &mut Leg, pwm: &mut Pca9685<I2c>) {
            leg.shoulder.set_angle(shoulder_pct);
            leg.elbow.set_angle(elbow_pct);
            leg.shoulder.do_move(pwm);
            leg.elbow.do_move(pwm);
        }

        // let _ = ufmt::uwriteln!(
        //     serial,
        //     "{}: Shoulder {}, elbow {} = {} -> {} -> {} -> {}",
        //     self.counter,
        //     (a_shoulder_pct) as u32,
        //     (a_elbow_pct) as u32,
        //     (Mg90d::MAX_PULSE - Mg90d::MIN_PULSE) as u32,
        //     (Mg90d::MAX_PULSE - Mg90d::MIN_PULSE) as u32 * a_elbow_pct as u32,
        //     ((Mg90d::MAX_PULSE - Mg90d::MIN_PULSE) as u32 * a_elbow_pct as u32) * 41 >> 12,
        //     ((Mg90d::MAX_PULSE - Mg90d::MIN_PULSE) as u32 * a_elbow_pct as u32 * 41 >> 12) as u16
        //         + Mg90d::MIN_PULSE
        // );
        move_leg(a_shoulder_pct, a_elbow_pct, &mut legs[0], pwm);
        move_leg(a_shoulder_pct, a_elbow_pct, &mut legs[2], pwm);
        move_leg(a_shoulder_pct, a_elbow_pct, &mut legs[4], pwm);
        move_leg(b_shoulder_pct, b_elbow_pct, &mut legs[1], pwm);
        move_leg(b_shoulder_pct, b_elbow_pct, &mut legs[3], pwm);
        move_leg(b_shoulder_pct, b_elbow_pct, &mut legs[5], pwm);

        self.counter += 1;
    }

    fn switch_phase(&mut self) {
        self.phase = match self.phase {
            WalkingPhase::LiftAndForward => WalkingPhase::HoldDownAndBackward,
            WalkingPhase::HoldDownAndBackward => WalkingPhase::LiftAndForward,
        };
    }
}

const RESOLUTION: usize = 300;
// Made in the rust playground with:

// let mut arr = [0; 300];
// for (i, v) in arr.iter_mut().enumerate() {
//     *v = (((i as f32 / 300.0) * std::f32::consts::PI).sin() * 100.0) as u16;
// }
const SIN_LOOKUP_TABLE: [u16; RESOLUTION] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
    50, 50, 51, 52, 53, 54, 55, 56, 57, 57, 58, 59, 60, 61, 62, 62, 63, 64, 65, 66, 66, 67, 68, 69,
    69, 70, 71, 72, 72, 73, 74, 75, 75, 76, 77, 77, 78, 79, 79, 80, 80, 81, 82, 82, 83, 83, 84, 84,
    85, 86, 86, 87, 87, 88, 88, 89, 89, 90, 90, 90, 91, 91, 92, 92, 92, 93, 93, 94, 94, 94, 95, 95,
    95, 96, 96, 96, 96, 97, 97, 97, 97, 98, 98, 98, 98, 98, 98, 99, 99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 100, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98,
    98, 97, 97, 97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 92, 92, 92, 91, 91, 90, 90,
    90, 89, 89, 88, 88, 87, 87, 86, 86, 85, 84, 84, 83, 83, 82, 82, 81, 80, 80, 79, 79, 78, 77, 77,
    76, 75, 75, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67, 66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58,
    57, 57, 56, 55, 54, 53, 52, 51, 50, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36,
    35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12,
    11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1,
];

enum WalkingPhase {
    LiftAndForward,
    HoldDownAndBackward,
}

#[allow(dead_code)]
struct Leg {
    shoulder: Servo<Mg90d>,
    elbow: Servo<Mg90d>,
}

impl Leg {
    fn new(shoulder: Servo<Mg90d>, elbow: Servo<Mg90d>) -> Self {
        Self { shoulder, elbow }
    }
}

// impl Direction {
//     fn flip(&mut self) {
//         *self = match self {
//             Direction::Forward => Direction::Backward,
//             Direction::Backward => Direction::Forward,
//         };
//     }
// }
