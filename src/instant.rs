//! This module fits the requirement of `rtic-monotonic`, but has uses beyond that.

use core::{
    self,
    cmp::{Ord, Ordering, PartialOrd},
    ops::{Add, Sub},
    time::Duration,
};

/// A time instant, from the start of a timer, with nanosecond precision. Has  methods similar
/// to that found in `core::Duration`.
#[derive(Eq, PartialEq, PartialOrd, Copy, Clone, Default)]
pub struct Instant {
    /// Total count, in microseconds. We use a signed integer for use with Durations.
    count_ns: i128,
    // todo: Count ticks instead?
}

/// An instant. Designed to be, in its most basic sense, similar to `std::time::Instant`. Created
/// from the `now()` method on a `Timer`. Can be compared to create a `core::Duration`. Standalone
/// methods on this struct are similar to those on `Duration`, but return timestamps since the timer start.
impl Instant {
    pub(crate) fn new(count_ns: i128) -> Self {
        Self { count_ns }
    }

    /// The time, in seconds.
    pub fn as_secs(&self) -> f32 {
        self.count_ns as f32 / 1_000_000_000.
    }

    /// The time, in milliseconds.
    pub fn as_millis(&self) -> u32 {
        (self.count_ns / 1_000_000) as u32
    }

    /// The time, in milliseconds as an f32.
    pub fn as_millis_f32(&self) -> f32 {
        self.count_ns as f32 / 1_000_000.
    }

    /// The time, in microseconds
    pub fn as_micros(&self) -> u64 {
        (self.count_ns / 1_000) as u64
    }

    /// The time, in nanoseconds
    pub fn as_nanos(&self) -> u128 {
        self.count_ns as u128
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
            count_ns: self.count_ns + rhs.as_nanos() as i128,
        }
    }
}

impl Sub<Duration> for Instant {
    type Output = Self;

    fn sub(self, rhs: Duration) -> Self::Output {
        Self {
            count_ns: self.count_ns - rhs.as_nanos() as i128,
        }
    }
}

impl Sub<Self> for Instant {
    type Output = Duration;

    fn sub(self, rhs: Self) -> Self::Output {
        // todo: Handle negative overflow.
        Duration::from_nanos((self.count_ns - rhs.count_ns) as u64)
    }
}
