/// Atomic latest-value slot for intra-process "mailbox" communication.
///
/// Producer atomically swaps in a new value; consumers always read the latest.
/// Uses Arc + RwLock for shared immutable snapshots.
/// All lock acquisitions handle poisoning gracefully (no panics).
use std::sync::{Arc, RwLock};
use std::time::Instant;

struct SlotInner<T> {
    value: Option<T>,
    updated_at: Option<Instant>,
}

pub struct AtomicSlot<T> {
    inner: Arc<RwLock<SlotInner<T>>>,
}

impl<T: Clone> AtomicSlot<T> {
    pub fn new() -> Self {
        Self {
            inner: Arc::new(RwLock::new(SlotInner {
                value: None,
                updated_at: None,
            })),
        }
    }

    /// Store a new value (producer side).
    pub fn store(&self, value: T) {
        if let Ok(mut inner) = self.inner.write() {
            inner.value = Some(value);
            inner.updated_at = Some(Instant::now());
        }
        // If lock is poisoned, silently skip (degraded but no panic)
    }

    /// Load the latest value (consumer side). Returns None if never written or lock poisoned.
    pub fn load(&self) -> Option<T> {
        self.inner.read().ok()?.value.clone()
    }

    /// Load value with its age in seconds. Returns (None, f64::INFINITY) if unset or lock poisoned.
    pub fn load_with_age(&self) -> (Option<T>, f64) {
        let inner = match self.inner.read() {
            Ok(guard) => guard,
            Err(_) => return (None, f64::INFINITY),
        };
        match (&inner.value, inner.updated_at) {
            (Some(val), Some(ts)) => (Some(val.clone()), ts.elapsed().as_secs_f64()),
            _ => (None, f64::INFINITY),
        }
    }

    /// Check if a value has been stored.
    pub fn has_value(&self) -> bool {
        self.inner.read().ok().map_or(false, |g| g.value.is_some())
    }

    /// Age of the stored value in seconds. Returns INFINITY if unset or lock poisoned.
    pub fn age_secs(&self) -> f64 {
        let inner = match self.inner.read() {
            Ok(guard) => guard,
            Err(_) => return f64::INFINITY,
        };
        match inner.updated_at {
            Some(ts) => ts.elapsed().as_secs_f64(),
            None => f64::INFINITY,
        }
    }
}

impl<T: Clone> Clone for AtomicSlot<T> {
    fn clone(&self) -> Self {
        Self {
            inner: Arc::clone(&self.inner),
        }
    }
}

impl<T: Clone> Default for AtomicSlot<T> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_store_load() {
        let slot = AtomicSlot::new();
        assert!(slot.load().is_none());
        assert!(!slot.has_value());

        slot.store(42);
        assert_eq!(slot.load(), Some(42));
        assert!(slot.has_value());

        slot.store(99);
        assert_eq!(slot.load(), Some(99));
    }

    #[test]
    fn test_age() {
        let slot = AtomicSlot::new();
        assert!(slot.age_secs().is_infinite());

        slot.store(1);
        std::thread::sleep(std::time::Duration::from_millis(10));
        assert!(slot.age_secs() < 1.0);
        assert!(slot.age_secs() > 0.0);
    }
}
