use crate::{log, Anim, CalibState, State};
use communication::{I2cRequest, I2cResponse};
use core::marker::PhantomData;
use heapless::Vec;
use pimoroni_servo2040::hal::i2c::peripheral::{I2CEvent, I2CPeripheralEventIterator};
use pimoroni_servo2040::pac::I2C0;
use pimoroni_servo2040::{Gp20I2C0Sda, Gp21I2C0Scl};

type I2C = I2CPeripheralEventIterator<I2C0, (Gp20I2C0Sda, Gp21I2C0Scl)>;

const BUF_LEN: usize = 32;
pub struct CommsManager<const NUM_SERVOS_PER_LEG: usize, const NUM_LEGS: usize> {
    buf: [u8; BUF_LEN],
    written: usize,
    read: usize,
    size_in: Option<usize>,
    size_out: Option<usize>,
    phantom_: PhantomData<[[(); NUM_SERVOS_PER_LEG]; NUM_LEGS]>,
}

impl<const NUM_SERVOS_PER_LEG: usize, const NUM_LEGS: usize> Default
    for CommsManager<NUM_SERVOS_PER_LEG, NUM_LEGS>
{
    fn default() -> Self {
        Self::new()
    }
}

impl<const NUM_SERVOS_PER_LEG: usize, const NUM_LEGS: usize>
    CommsManager<NUM_SERVOS_PER_LEG, NUM_LEGS>
{
    pub const fn new() -> Self {
        Self {
            buf: [0; BUF_LEN],
            written: 0,
            read: 0,
            size_in: None,
            size_out: None,
            phantom_: PhantomData,
        }
    }

    pub fn process_i2c_read(&mut self, i2c: &mut I2C, size_out: usize) {
        log::info!(
            "Starting read with written: {}, size_out: {}",
            self.written,
            size_out
        );
        if self.written >= size_out {
            log::info!("Writing dummy byte");
            // write one byte at a time to satisfy the controller.
            i2c.write(&[0]);
            return;
        }

        // only send the size of the first byte. The rest will come with a second
        // read request.
        if self.written == 0 {
            self.written += i2c.write(&self.buf[..1]);
        } else {
            while self.written < size_out {
                self.written += i2c.write(&self.buf[self.written..size_out]);
            }
        }
    }

    pub fn process_i2c_write(
        &mut self,
        i2c: &mut I2C,
        state: &mut State<NUM_SERVOS_PER_LEG, NUM_LEGS>,
    ) -> bool {
        let size_in = match self.size_in {
            Some(size_in) => size_in,
            None => {
                let mut size_in = 0;
                let _ = i2c.read(core::slice::from_mut(&mut size_in));
                let size_in = size_in as usize;
                if size_in > BUF_LEN {
                    log::panic!("Request too large: {}", size_in);
                }

                self.size_in = Some(size_in);
                size_in
            }
        };
        if self.read >= size_in {
            // read one byte at a time to satisfy the controller.
            i2c.read(&mut [0]);
            return false;
        }

        while self.read < size_in {
            self.read += i2c.read(&mut self.buf[self.read..size_in]);
        }

        // Clear the state now that we can process the request.
        self.read = 0;
        self.size_in = None;
        let req = postcard::from_bytes::<I2cRequest>(&self.buf[..size_in])
            .unwrap_or_else(|e| log::panic!("Unable to deserialize data: {:?}", e));
        self.process_request(state, req)
    }

    pub fn process_request(
        &mut self,
        state: &mut State<NUM_SERVOS_PER_LEG, NUM_LEGS>,
        req: I2cRequest,
    ) -> bool {
        match req {
            I2cRequest::GetState
            | I2cRequest::GetAnimationFactor
            | I2cRequest::GetAngularVelocity
            | I2cRequest::GetMotionVector
            | I2cRequest::GetBodyTranslation
            | I2cRequest::GetBodyRotation
            | I2cRequest::GetLegRadius
            | I2cRequest::GetBatteryLevel
            | I2cRequest::GetCalibration { .. } => {
                self.process_read(state, req);
                false
            }
            I2cRequest::ChangeState(_)
            | I2cRequest::SetAnimationFactor(_)
            | I2cRequest::SetAngularVelocity(_)
            | I2cRequest::SetMotionVector(_)
            | I2cRequest::SetBodyTranslation(_)
            | I2cRequest::SetBodyRotation(_)
            | I2cRequest::SetLegRadius(_)
            | I2cRequest::SetBatteryUpdateInterval(_)
            | I2cRequest::SetCalibration { .. } => {
                self.process_write(state, req);
                true
            }
        }
    }

    pub fn process_read(&mut self, state: &State<NUM_SERVOS_PER_LEG, NUM_LEGS>, req: I2cRequest) {
        let vec: Vec<u8, BUF_LEN> = match req {
            I2cRequest::GetState => {
                postcard::to_vec(&I2cResponse::GetState(state.live_state.state_machine)).unwrap()
            }
            I2cRequest::GetAnimationFactor => postcard::to_vec(&I2cResponse::GetAnimationFactor(
                state.live_state.animation_factor,
            ))
            .unwrap(),
            I2cRequest::GetAngularVelocity => postcard::to_vec(&I2cResponse::GetAngularVelocity(
                state.live_state.angular_velocity,
            ))
            .unwrap(),
            I2cRequest::GetMotionVector => postcard::to_vec(&I2cResponse::GetMotionVector(
                state.live_state.motion_vector,
            ))
            .unwrap(),
            I2cRequest::GetBodyTranslation => postcard::to_vec(&I2cResponse::GetBodyTranslation(
                state.live_state.body_translation,
            ))
            .unwrap(),
            I2cRequest::GetBodyRotation => postcard::to_vec(&I2cResponse::GetBodyRotation(
                state.live_state.body_rotation,
            ))
            .unwrap(),
            I2cRequest::GetLegRadius => {
                postcard::to_vec(&I2cResponse::GetLegRadius(state.live_state.leg_radius)).unwrap()
            }
            I2cRequest::GetBatteryLevel => postcard::to_vec(&I2cResponse::GetBatteryLevel(
                state.live_state.battery_level,
            ))
            .unwrap(),
            I2cRequest::GetCalibration { leg, joint, kind } => {
                let pulse = state
                    .live_state
                    .joint(leg, joint)
                    .map(|j| {
                        let calib = j.cal();
                        match kind {
                            0 => calib.home_pulse,
                            1 => calib.cal.inner().min_pulse(),
                            2 => calib.cal.inner().mid_pulse(),
                            3 => calib.cal.inner().max_pulse(),
                            _ => -2.0,
                        }
                    })
                    .unwrap_or(-1.0);
                postcard::to_vec(&I2cResponse::GetCalibration {
                    leg,
                    joint,
                    kind,
                    pulse,
                })
                .unwrap()
            }
            I2cRequest::ChangeState(_)
            | I2cRequest::SetAnimationFactor(_)
            | I2cRequest::SetAngularVelocity(_)
            | I2cRequest::SetMotionVector(_)
            | I2cRequest::SetBodyTranslation(_)
            | I2cRequest::SetBodyRotation(_)
            | I2cRequest::SetLegRadius(_)
            | I2cRequest::SetBatteryUpdateInterval(_)
            | I2cRequest::SetCalibration { .. } => {
                log::panic!("Unreachable");
            }
        };

        let size_out = vec.len() + 1;
        if size_out > BUF_LEN {
            log::panic!("Response too large: {}: for req {:?}", size_out, req);
        }
        self.size_out = Some(size_out);
        self.buf[0] = vec.len() as u8;
        self.buf[1..size_out].copy_from_slice(vec.as_slice());
    }

    /// Returns true if the state was updated.
    pub fn process_write(
        &mut self,
        state: &mut State<NUM_SERVOS_PER_LEG, NUM_LEGS>,
        req: I2cRequest,
    ) {
        match req {
            I2cRequest::ChangeState(sm) => {
                if sm != state.live_state.state_machine {
                    state.from_previous_state = Some(state.live_state.state_machine);
                    state.live_state.state_machine = sm;
                }
            }
            I2cRequest::SetAnimationFactor(v) => {
                state.live_state.animation_factor = v;
            }
            I2cRequest::SetAngularVelocity(v) => {
                state.live_state.angular_velocity = v;
            }
            I2cRequest::SetMotionVector(v) => {
                state.live_state.motion_vector = v;
            }
            I2cRequest::SetBodyTranslation(v) => {
                state.translation_anim = Some(Anim::new(state.live_state.body_translation, v));
            }
            I2cRequest::SetBodyRotation(v) => {
                state.rotation_anim = Some(Anim::new(state.live_state.body_rotation, v));
            }
            I2cRequest::SetLegRadius(v) => {
                state.leg_radius_anim = Some(Anim::new(state.live_state.leg_radius, v));
            }
            I2cRequest::SetBatteryUpdateInterval(v) => {
                state.live_state.battery_update_interval_ms = v;
            }
            I2cRequest::SetCalibration {
                leg,
                joint,
                kind,
                pulse,
            } => {
                if let Some(cal_kind) = match kind {
                    0 => Some(crate::CalKind::Min),
                    1 => Some(crate::CalKind::Mid),
                    2 => Some(crate::CalKind::Max),
                    3 => Some(crate::CalKind::Home),
                    _ => {
                        log::warn!("Unknown cal kind \"{}\"", kind,);
                        None
                    }
                } {
                    let start_pulse = state
                        .live_state
                        .joint(leg, joint)
                        .map(|j| {
                            let cal = j.cal();
                            match kind {
                                0 => cal.cal.inner().min_pulse(),
                                1 => cal.cal.inner().mid_pulse(),
                                2 => cal.cal.inner().max_pulse(),
                                3 => cal.home_pulse,
                                _ => unreachable!(),
                            }
                        })
                        .unwrap_or(500.0);
                    let calib_state = CalibState {
                        leg: leg as usize,
                        joint: joint as usize,
                        cal_kind,
                        anim: Anim::new(start_pulse, pulse),
                    };
                    state.calib_update = Some(calib_state);
                }
            }
            I2cRequest::GetState
            | I2cRequest::GetAnimationFactor
            | I2cRequest::GetAngularVelocity
            | I2cRequest::GetMotionVector
            | I2cRequest::GetBodyTranslation
            | I2cRequest::GetBodyRotation
            | I2cRequest::GetLegRadius
            | I2cRequest::GetBatteryLevel
            | I2cRequest::GetCalibration { .. } => {
                log::panic!("Should not get read in write");
            }
        }
    }

    pub fn run_loop(
        &mut self,
        i2c: &mut I2C,
        state: &mut State<NUM_SERVOS_PER_LEG, NUM_LEGS>,
    ) -> bool {
        let mut updated = false;
        while let Some(event) = i2c.next() {
            match event {
                I2CEvent::Start => {
                    log::info!("Start");
                }
                I2CEvent::Restart => {
                    log::info!("Restart");
                }
                I2CEvent::TransferRead => {
                    // Clear data used for TransferWrite
                    self.size_in = None;
                    self.read = 0;
                    log::info!("TransferRead");
                    if let Some(size_out) = self.size_out {
                        self.process_i2c_read(i2c, size_out);
                    } else {
                        i2c.write(&[0]);
                    }
                }
                I2CEvent::TransferWrite => {
                    // Clear data used for TransferRead
                    log::info!("TransferWrite");
                    self.size_out = None;
                    self.written = 0;
                    updated |= self.process_i2c_write(i2c, state)
                }
                I2CEvent::Stop => {
                    log::info!("Stop: clearing data");
                }
            }
        }
        log::info!("No more events");
        updated
    }
}
