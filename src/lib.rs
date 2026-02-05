#![no_std]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]
#![deny(clippy::panic)]
#![deny(unused_must_use)]

mod pid_controller;

pub use pid_controller::{PidConstants, PidController, PidError};
