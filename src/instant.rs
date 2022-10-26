//! This module fits the requirement of `rtic-monotonic`, but has uses beyond that.

use core::{
    self,
    cmp::{Ord, Ordering, PartialOrd},
    ops::{Add, Sub},
    time::Duration,
};

/// A time instant, from the start of a timer, for use with `rtic-monotonic`. Currently only
/// has microsecond precision.
#[derive(Eq, PartialEq, PartialOrd, Copy, Clone, Default)]
pub struct Instant {
    /// Total count, in microseconds.
    /// todo: Do you need ns resolution?
    pub count_us: i64, // todo: u64 or i64
}

impl Instant {
    /// The time, in seconds.
    pub fn as_seconds(&self) -> f32 {
        self.count_us as f32 / 1_000_000.
    }

    /// The time, in milliseconds.
    pub fn as_ms(&self) -> f32 {
        self.count_us as f32 / 1_000.
    }

    /// The time, in microseconds
    pub fn as_us(&self) -> f32 {
        self.count_us as f32
    }
}

impl Ord for Instant {
    fn cmp(&self, other: &Self) -> Ordering {
        self.count_us.cmp(&other.count_us)
    }
}

impl Add<Duration> for Instant {
    type Output = Self;

    fn add(self, rhs: Duration) -> Self::Output {
        Self {
            count_us: self.count_us + rhs.as_micros() as i64,
        }
    }
}

impl Sub<Duration> for Instant {
    type Output = Self;

    fn sub(self, rhs: Duration) -> Self::Output {
        Self {
            count_us: self.count_us - rhs.as_micros() as i64,
        }
    }
}

impl Sub<Self> for Instant {
    type Output = Duration;

    fn sub(self, rhs: Self) -> Self::Output {
        // todo: Handle negative overflow!
        Duration::from_micros((self.count_us - rhs.count_us) as u64)
    }
}
