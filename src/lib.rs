#![no_std]

//! # Rust Pico 2 Discovery Utility Library
//!
//! Provides shared utilities for examples in this repository.

/// Helper struct for formatting floating-point numbers in `defmt` logs.
///
/// `defmt` performs deferred formatting on the host, so it doesn't natively
/// support precision control (like `.2f`) on the device. This utility ensures
/// exactly 2 decimal places by converting the float to a scaled integer pair
/// before logging.
///
/// ## Usage:
/// ```rust
/// use rust_rpico2_discovery::Fmt;
/// defmt::info!("Value: {}", Fmt(1.2345)); // Outputs: "Value: 1.23"
/// ```
pub struct Fmt(pub f32);

impl defmt::Format for Fmt {
    fn format(&self, f: defmt::Formatter) {
        // Multiplier for 2 decimal places
        const PRECISION: f32 = 100.0;

        let scaled = (self.0 * PRECISION) as i32;
        let int = scaled / 100;
        let frac = (scaled % 100).abs();

        // Handle negative sign correctly for values between -1.0 and 0.0
        if scaled < 0 && int == 0 {
            defmt::write!(f, "-0.{:02}", frac);
        } else {
            defmt::write!(f, "{}.{:02}", int, frac);
        }
    }
}
