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
    // pub count_us: i64, // todo: u64 or i64? x128?
    pub count_ns: i64, // todo: u64 or i64? x128?
}

impl Instant {
    /// The time, in seconds.
    pub fn as_secs(&self) -> f32 {
        // self.count_us as f32 / 1_000_000.
        self.count_ns as f32 / 1_000_000_000.
    }

    /// The time, in milliseconds.
    pub fn as_ms(&self) -> f32 {
        // self.count_us as f32 / 1_000.
        self.count_ns as f32 / 1_000_000.
    }

    /// The time, in microseconds
    pub fn as_us(&self) -> f32 {
        // self.count_us as f32
        self.count_ns as f32 / 1_000.
    }

    /// The time, in nanoseconds
    pub fn as_ns(&self) -> f32 {
        self.count_ns as f32
    }
}

impl Ord for Instant {
    fn cmp(&self, other: &Self) -> Ordering {
        // self.count_us.cmp(&other.count_us)
        self.count_ns.cmp(&other.count_ns)
    }
}

impl Add<Duration> for Instant {
    type Output = Self;

    fn add(self, rhs: Duration) -> Self::Output {
        Self {
            // count_us: self.count_us + rhs.as_micros() as i64,
            count_ns: self.count_ns + rhs.as_nanos() as i64,
        }
    }
}

impl Sub<Duration> for Instant {
    type Output = Self;

    fn sub(self, rhs: Duration) -> Self::Output {
        Self {
            // count_us: self.count_us - rhs.as_micros() as i64,
            count_ns: self.count_ns - rhs.as_nanos() as i64,
        }
    }
}

impl Sub<Self> for Instant {
    type Output = Duration;

    fn sub(self, rhs: Self) -> Self::Output {
        // todo: Handle negative overflow!
        // Duration::from_micros((self.count_us - rhs.count_us) as u64)
        Duration::from_nanos((self.count_ns - rhs.count_ns) as u64)
    }
}
