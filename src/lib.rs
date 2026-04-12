#![doc = include_str!("../README.md")]
#![no_std]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]
#![deny(clippy::panic)]
#![deny(unused_must_use)]
#![warn(unused_results)]
#![warn(clippy::pedantic)]
#![warn(clippy::doc_paragraphs_missing_punctuation)]
#![allow(clippy::inline_always)]
#![allow(clippy::return_self_not_must_use)]

pub mod pidsk_controller;

pub use pidsk_controller::{PidConstants, PidController, PidError};
