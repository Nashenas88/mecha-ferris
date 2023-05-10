use crate::log;
use communication::{Deserialize, I2cRequestField, I2cRequestOp, Serialize};
use core::task::Poll;
use pimoroni_servo2040::hal::i2c::peripheral::{I2CEvent, I2CPeripheralEventIterator};
use pimoroni_servo2040::pac::I2C0;
use pimoroni_servo2040::{Gp20I2C0Sda, Gp21I2C0Scl};

use state::RobotState;

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

pub struct CommsManager {
    i2c_req: Option<I2cRequestOp>,
    buf: [u8; 16],
    written: usize,
    read: usize,
}

impl Default for CommsManager {
    fn default() -> Self {
        Self::new()
    }
}

impl CommsManager {
    pub fn new() -> Self {
        Self {
            i2c_req: None,
            buf: [0; 16],
            written: 0,
            read: 0,
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
            Poll::Ready(<T as Deserialize>::deserialize(&mut self.buf[1..]))
        } else {
            Poll::Pending
        }
    }

    pub fn process_read_op(&mut self, i2c: &mut I2C, state: &RobotState, op: I2cRequestOp) {
        match op {
            I2cRequestOp::Get(field) => match field {
                I2cRequestField::Speed => self.write(i2c, state.speed),
                I2cRequestField::AngularVelocity => self.write(i2c, state.angular_velocity),
                I2cRequestField::MoveVector => self.write(i2c, state.motion_vector),
                I2cRequestField::BodyTranslation => self.write(i2c, state.body_translation),
                I2cRequestField::BodyRotation => self.write(i2c, state.body_rotation),
                I2cRequestField::LegRadius => self.write(i2c, state.leg_radius),
            },
            I2cRequestOp::GetBatteryLevel => self.write(i2c, state.battery_level),
            _ => log::warn!("unexpected op on read request {}", op),
        }
    }

    pub fn process_write_op(&mut self, i2c: &mut I2C, state: &mut RobotState) -> bool {
        self.read = i2c.read(&mut self.buf);
        match <I2cRequestOp as Deserialize>::deserialize(&self.buf[..self.read]) {
            Ok(op) => {
                self.i2c_req = Some(op);
                if self.read > 1 {
                    return self.process_write(i2c, state, op);
                }
            }
            Err(e) => log::warn!("Unable to deserialize data: {:?}", e),
        }
        false
    }

    pub fn process_write(
        &mut self,
        i2c: &mut I2C,
        state: &mut RobotState,
        op: I2cRequestOp,
    ) -> bool {
        let mut updated = false;
        match op {
            I2cRequestOp::ChangeState => {
                on_ready!(self.read(i2c), state.state_machine, {
                    updated = true;
                })
            }
            I2cRequestOp::Set(field) => match field {
                I2cRequestField::Speed => {
                    on_ready!(self.read(i2c), state.speed, {
                        updated = true;
                    })
                }
                I2cRequestField::AngularVelocity => {
                    on_ready!(self.read(i2c), state.angular_velocity, {
                        updated = true;
                    })
                }
                I2cRequestField::MoveVector => {
                    on_ready!(self.read(i2c), state.motion_vector, {
                        updated = true;
                    })
                }
                I2cRequestField::BodyTranslation => {
                    on_ready!(self.read(i2c), state.body_translation, {
                        updated = true;
                    })
                }
                I2cRequestField::BodyRotation => {
                    on_ready!(self.read(i2c), state.body_rotation, {
                        updated = true;
                    })
                }
                I2cRequestField::LegRadius => {
                    on_ready!(self.read(i2c), state.leg_radius, {
                        updated = true;
                    })
                }
            },
            I2cRequestOp::SetBatteryUpdateInterval => {
                on_ready!(self.read(i2c), state.battery_update_interval_ms, {
                    updated = true;
                })
            }
            _ => log::warn!("unexpected op on write request {}", op),
        }
        updated
    }

    pub fn run_loop(&mut self, i2c: &mut I2C, state: &mut RobotState) -> bool {
        let mut updated = false;
        while let Some(event) = i2c.next() {
            match event {
                I2CEvent::Start | I2CEvent::Restart => {}
                I2CEvent::TransferRead => match self.i2c_req {
                    Some(op) => self.process_read_op(i2c, state, op),
                    None => log::warn!("missing request on read"),
                },
                I2CEvent::TransferWrite => {
                    updated |= match self.i2c_req {
                        Some(op) => self.process_write(i2c, state, op),
                        None => self.process_write_op(i2c, state),
                    }
                }
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
