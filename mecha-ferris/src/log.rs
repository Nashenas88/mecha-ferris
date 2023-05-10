#[cfg(feature = "defmt")]
pub use defmt::{error, info, panic, trace, warn};

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! error {
    ($($arg:tt)*) => {{}};
}

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! info {
    ($($arg:tt)*) => {{}};
}

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! panic_ {
    ($($arg:tt)*) => {core::panic!($($arg)*)};
}

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! trace {
    ($($arg:tt)*) => {{}};
}

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! warn_ {
    ($($arg:tt)*) => {{}};
}

#[cfg(not(feature = "defmt"))]
pub use {error, info, panic_ as panic, trace, warn_ as warn};
