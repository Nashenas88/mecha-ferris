#![no_std]

pub trait TestableFixedGattValue {
    const SIZE: usize;
    fn from_gatt(data: &[u8]) -> Self;
    fn to_gatt(&self) -> &[u8];
}

pub mod log;
pub mod wrappers;

#[derive(Copy, Clone, Debug)]
#[repr(C)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CalibrationIndex {
    pub leg: u8,
    pub joint: u8,
    pub kind: u8,
}

impl TestableFixedGattValue for CalibrationIndex {
    const SIZE: usize = core::mem::size_of::<CalibrationIndex>();

    fn from_gatt(data: &[u8]) -> Self {
        Self {
            leg: data[0],
            joint: data[1],
            kind: data[2],
        }
    }

    fn to_gatt(&self) -> &[u8] {
        // Safety: struct has C layout, alignment of 1 due to only u8 fields.
        unsafe {
            core::slice::from_raw_parts(
                self as *const _ as *const u8,
                core::mem::size_of::<CalibrationIndex>(),
            )
        }
    }
}

#[derive(Copy, Clone, Debug)]
#[repr(C, packed(4))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CalibrationDatum {
    value: f32,
    index: CalibrationIndex,
    _packed: u8,
}

impl CalibrationDatum {
    pub fn new(value: f32, index: CalibrationIndex) -> Self {
        Self {
            value,
            index,
            _packed: 0,
        }
    }

    pub fn value(&self) -> f32 {
        self.value
    }

    pub fn index(&self) -> CalibrationIndex {
        self.index
    }
}

const _: () = assert!(core::mem::size_of::<CalibrationDatum>() == 8);
const _: () = assert!(core::mem::align_of::<CalibrationDatum>() == 4);

impl TestableFixedGattValue for CalibrationDatum {
    const SIZE: usize = core::mem::size_of::<CalibrationDatum>();

    fn from_gatt(data: &[u8]) -> Self {
        Self {
            value: f32::from_gatt(&data[0..4]),
            index: CalibrationIndex::from_gatt(&data[4..7]),
            _packed: 0,
        }
    }

    fn to_gatt(&self) -> &[u8] {
        // TODO(paul) the slice needs to be aligned...
        unsafe {
            core::slice::from_raw_parts(
                self as *const _ as *const u8,
                core::mem::size_of::<CalibrationDatum>(),
            )
        }
    }
}

#[derive(Copy, Clone, Debug)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SetRes {
    Ok = 0,
    InvalidIndex = 1,
    InvalidValue = 2,
    InvalidKind = 3,
    InvalidLeg = 4,
    InvalidJoint = 5,
}

#[derive(Copy, Clone, Debug)]
#[repr(C, packed(4))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SetResult {
    pub index: CalibrationIndex,
    pub result: SetRes,
}

impl TestableFixedGattValue for SetResult {
    const SIZE: usize = core::mem::size_of::<SetResult>();

    fn from_gatt(data: &[u8]) -> Self {
        Self {
            index: CalibrationIndex::from_gatt(&data[0..3]),
            result: match data[3] {
                0 => SetRes::Ok,
                1 => SetRes::InvalidIndex,
                2 => SetRes::InvalidValue,
                3 => SetRes::InvalidKind,
                4 => SetRes::InvalidLeg,
                5 => SetRes::InvalidJoint,
                _ => unreachable!(),
            },
        }
    }

    fn to_gatt(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                self as *const _ as *const u8,
                core::mem::size_of::<SetResult>(),
            )
        }
    }
}
