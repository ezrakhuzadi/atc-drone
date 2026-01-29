//! Simple exponential backoff helper with jitter.
//!
//! Intended for background loops that call external dependencies (Blender/DSS/etc.)
//! so outages don't cause tight retry loops and log storms.

use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

#[derive(Debug, Clone)]
pub struct Backoff {
    base: Duration,
    max: Duration,
    current: Duration,
    next_attempt_at: Instant,
    jitter_ratio: f64,
}

impl Backoff {
    pub fn new(base: Duration, max: Duration) -> Self {
        let base = base.max(Duration::from_millis(1));
        let max = max.max(base);
        Self {
            base,
            max,
            current: base,
            next_attempt_at: Instant::now(),
            jitter_ratio: 0.2,
        }
    }

    pub fn ready(&self) -> bool {
        Instant::now() >= self.next_attempt_at
    }

    pub fn reset(&mut self) {
        self.current = self.base;
        self.next_attempt_at = Instant::now();
    }

    pub fn fail(&mut self) -> Duration {
        self.current = self.current.saturating_mul(2).min(self.max);
        let delay = add_jitter(self.current, self.jitter_ratio);
        self.next_attempt_at = Instant::now() + delay;
        delay
    }
}

fn add_jitter(delay: Duration, ratio: f64) -> Duration {
    if !(0.0..=1.0).contains(&ratio) {
        return delay;
    }

    let delay_ms = delay.as_millis();
    if delay_ms == 0 {
        return delay;
    }

    let jitter_ms_max = ((delay_ms as f64) * ratio) as u128;
    if jitter_ms_max == 0 {
        return delay;
    }

    let now_nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.subsec_nanos() as u64)
        .unwrap_or(0);
    let jitter_ms = (now_nanos as u128) % (jitter_ms_max + 1);
    delay + Duration::from_millis(jitter_ms as u64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_backoff_is_ready() {
        let backoff = Backoff::new(Duration::from_millis(10), Duration::from_secs(1));
        assert!(backoff.ready());
    }

    #[test]
    fn fail_makes_not_ready_until_reset() {
        let mut backoff = Backoff::new(Duration::from_millis(100), Duration::from_secs(1));
        assert!(backoff.ready());

        let delay = backoff.fail();
        assert!(delay >= Duration::from_millis(200));
        assert!(!backoff.ready());

        backoff.reset();
        assert!(backoff.ready());
    }

    #[test]
    fn fail_saturates_at_max() {
        let mut backoff = Backoff::new(Duration::from_millis(10), Duration::from_millis(20));

        let delay1 = backoff.fail();
        assert!(delay1 >= Duration::from_millis(20));
        assert!(delay1 <= Duration::from_millis(24));

        let delay2 = backoff.fail();
        assert!(delay2 >= Duration::from_millis(20));
        assert!(delay2 <= Duration::from_millis(24));
    }
}
