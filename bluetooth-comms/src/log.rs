#[cfg(feature = "defmt")]
pub use defmt::{error, info, panic, trace, unwrap, warn};

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! error {
    ($($arg:expr),*$(,)?) => {{$(let _ = &$arg;)*}};
}

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! info {
    ($($arg:expr),*$(,)?) => {{$(let _ = &$arg;)*}};
}

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! panic_ {
    ($($arg:tt)*) => {core::panic!($($arg)*)};
}

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! trace {
    ($($arg:expr),*$(,)?) => {{$(let _ = &$arg;)*}};
}

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! warn_ {
    ($($arg:expr),*$(,)?) => {{$(let _ = &$arg;)*}};
}

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! unwrap_ {
    ($arg:expr) => {
        $arg.unwrap()
    };
}

#[cfg(not(feature = "defmt"))]
pub use {error, info, panic_ as panic, trace, unwrap_ as unwrap, warn_ as warn};
