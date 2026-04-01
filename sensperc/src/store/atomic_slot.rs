/// Atomic latest-value slot for intra-process "mailbox" communication.
///
/// Producer atomically swaps in a new value; consumers always read the latest.
/// Uses Arc + RwLock for shared immutable snapshots.
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
        let mut inner = self.inner.write().unwrap();
        inner.value = Some(value);
        inner.updated_at = Some(Instant::now());
    }

    /// Load the latest value (consumer side). Returns None if never written.
    pub fn load(&self) -> Option<T> {
        let inner = self.inner.read().unwrap();
        inner.value.clone()
    }

    /// Load value with its age in seconds. Returns (None, f64::INFINITY) if unset.
    pub fn load_with_age(&self) -> (Option<T>, f64) {
        let inner = self.inner.read().unwrap();
        match (&inner.value, inner.updated_at) {
            (Some(val), Some(ts)) => (Some(val.clone()), ts.elapsed().as_secs_f64()),
            _ => (None, f64::INFINITY),
        }
    }

    /// Check if a value has been stored.
    pub fn has_value(&self) -> bool {
        self.inner.read().unwrap().value.is_some()
    }

    /// Age of the stored value in seconds. Returns INFINITY if unset.
    pub fn age_secs(&self) -> f64 {
        let inner = self.inner.read().unwrap();
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
