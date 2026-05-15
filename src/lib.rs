#![doc = include_str!("../README.md")]
#![no_std]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]
#![deny(clippy::panic)]
#![deny(unused_must_use)]
#![warn(unused_results)]
#![warn(clippy::pedantic)]
#![warn(clippy::doc_paragraphs_missing_punctuation)]
#![allow(clippy::return_self_not_must_use)]

mod pidsk_adjust;
mod pidsk_controller;

pub use pidsk_adjust::UpdatePidController;
pub use pidsk_controller::PidController;
pub use pidsk_controller::{Pid, Pidf32, Pidf64};
pub use pidsk_controller::{PidErrorf32, PidErrorf64, PidErrors};
pub use pidsk_controller::{PidGains, PidGainsf32, PidGainsf64};
