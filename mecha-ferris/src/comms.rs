use crate::{log, Anim, CalibState, State};
use communication::{Deserialize, I2cRequestField, I2cRequestOp, Serialize};
use core::marker::PhantomData;
use core::task::Poll;
use pimoroni_servo2040::hal::i2c::peripheral::{I2CEvent, I2CPeripheralEventIterator};
use pimoroni_servo2040::pac::I2C0;
use pimoroni_servo2040::{Gp20I2C0Sda, Gp21I2C0Scl};

type I2C = I2CPeripheralEventIterator<I2C0, (Gp20I2C0Sda, Gp21I2C0Scl)>;

macro_rules! on_ready {
    ($call:expr, $assign:expr, $block:block) => {
        if let Poll::Ready(res) = $call {
            match res {
                Ok(v) => {
                    $block
                    $assign = v
                },
                Err(e) => log::warn!("Unable to deserialize data: {:?}", e),
            }
        }
    };
}

pub struct CommsManager<const NUM_SERVOS_PER_LEG: usize, const NUM_LEGS: usize> {
    i2c_req: Option<I2cRequestOp>,
    buf: [u8; 16],
    written: usize,
    read: usize,
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
    pub fn new() -> Self {
        Self {
            i2c_req: None,
            buf: [0; 16],
            written: 0,
            read: 0,
            phantom_: PhantomData,
        }
    }

    fn write(&mut self, i2c: &mut I2C, data: impl Serialize) {
        if self.written == 0 {
            let size = data.serialize(&mut self.buf);
            self.buf[size..].fill(0);
        }
        self.written += i2c.write(&self.buf[self.written..])
    }

    fn read<T>(&mut self, i2c: &mut I2C) -> Poll<Result<T, <T as Deserialize>::Error>>
    where
        T: for<'a> Deserialize<Input<'a> = &'a [u8]>,
    {
        self.read += i2c.read(&mut self.buf[self.read..]);
        if self.read == self.buf.len() {
            <T as Deserialize>::deserialize(&mut self.buf[1..])
        } else {
            Poll::Pending
        }
    }

    pub fn process_read_op(
        &mut self,
        i2c: &mut I2C,
        state: &State<NUM_SERVOS_PER_LEG, NUM_LEGS>,
        op: I2cRequestOp,
    ) {
        // Finish consuming the buffer.
        while self.read < self.buf.len() {
            self.read += i2c.read(&mut self.buf[self.read..]);
        }
        match op {
            I2cRequestOp::Get(field) => match field {
                I2cRequestField::AnimationFactor => {
                    self.write(i2c, state.live_state.animation_factor)
                }
                I2cRequestField::AngularVelocity => {
                    self.write(i2c, state.live_state.angular_velocity)
                }
                I2cRequestField::MotionVector => self.write(i2c, state.live_state.motion_vector),
                I2cRequestField::BodyTranslation => {
                    self.write(i2c, state.live_state.body_translation)
                }
                I2cRequestField::BodyRotation => self.write(i2c, state.live_state.body_rotation),
                I2cRequestField::LegRadius => self.write(i2c, state.live_state.leg_radius),
                I2cRequestField::Calibration { leg, joint, kind } => {
                    let data = {
                        state
                            .live_state
                            .joints
                            .map(|j| {
                                let calib = j[leg as usize][joint as usize].cal();
                                match kind {
                                    0 => calib.home_pulse,
                                    1 => calib.cal.inner().min_pulse(),
                                    2 => calib.cal.inner().mid_pulse(),
                                    3 => calib.cal.inner().max_pulse(),
                                    _ => -1.0,
                                }
                            })
                            .unwrap_or(-1.0)
                    };
                    self.write(i2c, data)
                }
            },
            I2cRequestOp::GetBatteryLevel => self.write(i2c, state.live_state.battery_level),
            _ => log::warn!("unexpected op on read request {}", op),
        }
    }

    pub fn process_write_op(
        &mut self,
        i2c: &mut I2C,
        state: &mut State<NUM_SERVOS_PER_LEG, NUM_LEGS>,
    ) -> Poll<bool> {
        self.read = i2c.read(&mut self.buf);
        match <I2cRequestOp as Deserialize>::deserialize(&self.buf[..self.read]) {
            Poll::Ready(Ok(op)) => {
                self.i2c_req = Some(op);
                if self.read > 1 {
                    return Poll::Ready(self.process_op(i2c, state, op));
                }
            }
            Poll::Ready(Err(e)) => log::warn!("Unable to deserialize data: {:?}", e),
            Poll::Pending => return Poll::Pending,
        }
        Poll::Ready(false)
    }

    /// Returns true if the state was updated.
    pub fn process_op(
        &mut self,
        i2c: &mut I2C,
        state: &mut State<NUM_SERVOS_PER_LEG, NUM_LEGS>,
        op: I2cRequestOp,
    ) -> bool {
        match op {
            I2cRequestOp::Get(_) | I2cRequestOp::GetBatteryLevel => {
                self.process_read_op(i2c, state, op);
                false
            }
            I2cRequestOp::Set(_)
            | I2cRequestOp::ChangeState
            | I2cRequestOp::SetBatteryUpdateInterval => self.process_write(i2c, state, op),
        }
    }

    /// Returns true if the state was updated.
    pub fn process_write(
        &mut self,
        i2c: &mut I2C,
        state: &mut State<NUM_SERVOS_PER_LEG, NUM_LEGS>,
        op: I2cRequestOp,
    ) -> bool {
        let mut updated = false;
        match op {
            I2cRequestOp::ChangeState => {
                on_ready!(self.read(i2c), state.live_state.state_machine, {
                    updated = true;
                })
            }
            I2cRequestOp::Set(field) => match field {
                I2cRequestField::AnimationFactor => {
                    on_ready!(self.read(i2c), state.live_state.animation_factor, {
                        updated = true;
                    });
                    if updated {
                        // Make sure the animation factor is not too low.
                        state.live_state.animation_factor =
                            state.live_state.animation_factor.max(0.1);
                    }
                }
                I2cRequestField::AngularVelocity => {
                    on_ready!(self.read(i2c), state.live_state.angular_velocity, {
                        updated = true;
                    })
                }
                I2cRequestField::MotionVector => {
                    on_ready!(self.read(i2c), state.live_state.motion_vector, {
                        updated = true;
                    })
                }
                I2cRequestField::BodyTranslation => {
                    if let Poll::Ready(res) = self.read(i2c) {
                        match res {
                            Ok(v) => {
                                state.translation_anim =
                                    Some(Anim::new(state.live_state.body_translation, v));
                                updated = true;
                            }
                            Err(e) => log::warn!("Unable to deserialize data: {:?}", e),
                        }
                    }
                }
                I2cRequestField::BodyRotation => {
                    if let Poll::Ready(res) = self.read(i2c) {
                        match res {
                            Ok(v) => {
                                state.rotation_anim =
                                    Some(Anim::new(state.live_state.body_rotation, v));
                                updated = true;
                            }
                            Err(e) => log::warn!("Unable to deserialize data: {:?}", e),
                        }
                    }
                }
                I2cRequestField::LegRadius => {
                    if let Poll::Ready(res) = self.read(i2c) {
                        match res {
                            Ok(v) => {
                                state.leg_radius_anim =
                                    Some(Anim::new(state.live_state.leg_radius, v));
                                updated = true;
                            }
                            Err(e) => log::warn!("Unable to deserialize data: {:?}", e),
                        }
                    }
                }
                I2cRequestField::Calibration { leg, joint, kind } => {
                    if let Poll::Ready(res) = self.read(i2c) {
                        match res {
                            Ok(v) => {
                                state.calib_update = Some(CalibState {
                                    leg: leg as usize,
                                    joint: joint as usize,
                                    cal_kind: match kind {
                                        0 => crate::CalKind::Min,
                                        1 => crate::CalKind::Mid,
                                        2 => crate::CalKind::Max,
                                        3 => crate::CalKind::Home,
                                        // TODO FIXME
                                        _ => unreachable!(),
                                    },
                                    anim: Anim::new(0.0, v),
                                });
                                updated = true;
                            }
                            Err(e) => log::warn!("Unable to deserialize data: {:?}", e),
                        }
                    }
                }
            },
            I2cRequestOp::SetBatteryUpdateInterval => {
                on_ready!(
                    self.read(i2c),
                    state.live_state.battery_update_interval_ms,
                    {
                        updated = true;
                    }
                )
            }
            _ => log::warn!("unexpected op on write request {}", op),
        }
        updated
    }

    pub fn run_loop(
        &mut self,
        i2c: &mut I2C,
        state: &mut State<NUM_SERVOS_PER_LEG, NUM_LEGS>,
    ) -> bool {
        let mut updated = false;
        while let Some(event) = i2c.next() {
            match event {
                I2CEvent::Start | I2CEvent::Restart => {}
                I2CEvent::TransferRead => match self.i2c_req {
                    Some(op) => self.process_read_op(i2c, state, op),
                    None => log::warn!("missing request on read"),
                },
                I2CEvent::TransferWrite => match self.i2c_req {
                    Some(op) => updated |= self.process_op(i2c, state, op),
                    None => {
                        if let Poll::Ready(u) = self.process_write_op(i2c, state) {
                            updated |= u;
                        }
                    }
                },
                I2CEvent::Stop => {
                    self.i2c_req = None;
                    self.written = 0;
                    self.read = 0;
                    break;
                }
            }
        }
        updated
    }
}
