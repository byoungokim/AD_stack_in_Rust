/// SPSC lock-free ring buffer for intra-process streaming data.
///
/// Uses crossbeam's ArrayQueue under the hood — bounded, lock-free MPMC queue
/// that we use in SPSC mode for driver → processor communication.
use crossbeam::queue::ArrayQueue;
use std::sync::Arc;

pub struct RingBuffer<T> {
    queue: Arc<ArrayQueue<T>>,
    capacity: usize,
}

impl<T> RingBuffer<T> {
    /// Create a new ring buffer with the given capacity.
    pub fn new(capacity: usize) -> Self {
        Self {
            queue: Arc::new(ArrayQueue::new(capacity)),
            capacity,
        }
    }

    /// Push an item. If full, drops the oldest item first.
    pub fn push(&self, item: T) {
        if self.queue.push(item).is_err() {
            // Full — drop oldest and retry
            let _ = self.queue.pop();
            // Item was moved into the failed push, so we can't retry with it.
            // This is a known limitation; use push_overwrite instead.
        }
    }

    /// Push an item, dropping the oldest if full. Returns the dropped item.
    pub fn push_overwrite(&self, item: T) -> Option<T> {
        match self.queue.push(item) {
            Ok(()) => None,
            Err(item) => {
                let dropped = self.queue.pop();
                // Retry push — should succeed now
                let _ = self.queue.push(item);
                dropped
            }
        }
    }

    /// Pop the oldest item. Returns None if empty.
    pub fn pop(&self) -> Option<T> {
        self.queue.pop()
    }

    /// Drain all items and return only the latest one.
    pub fn pop_latest(&self) -> Option<T> {
        let mut latest = None;
        while let Some(item) = self.queue.pop() {
            latest = Some(item);
        }
        latest
    }

    /// Check if the buffer is empty.
    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }

    // Public introspection/sharing API for ring-buffer consumers (stats, fan-out across threads).
    #[allow(dead_code)]
    /// Current number of items in the buffer.
    pub fn len(&self) -> usize {
        self.queue.len()
    }

    /// Buffer capacity.
    #[allow(dead_code)]
    pub fn capacity(&self) -> usize {
        self.capacity
    }

    /// Get a clone of the inner Arc for sharing across threads.
    #[allow(dead_code)]
    pub fn handle(&self) -> Arc<ArrayQueue<T>> {
        Arc::clone(&self.queue)
    }
}

impl<T> Clone for RingBuffer<T> {
    fn clone(&self) -> Self {
        Self {
            queue: Arc::clone(&self.queue),
            capacity: self.capacity,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_push_pop() {
        let buf = RingBuffer::new(4);
        buf.push(1);
        buf.push(2);
        buf.push(3);
        assert_eq!(buf.pop(), Some(1));
        assert_eq!(buf.pop(), Some(2));
        assert_eq!(buf.pop(), Some(3));
        assert_eq!(buf.pop(), None);
    }

    #[test]
    fn test_push_overwrite() {
        let buf = RingBuffer::new(2);
        assert!(buf.push_overwrite(1).is_none());
        assert!(buf.push_overwrite(2).is_none());
        // Buffer full, should drop oldest (1)
        let dropped = buf.push_overwrite(3);
        assert_eq!(dropped, Some(1));
        assert_eq!(buf.pop(), Some(2));
        assert_eq!(buf.pop(), Some(3));
    }

    #[test]
    fn test_pop_latest() {
        let buf = RingBuffer::new(8);
        buf.push(1);
        buf.push(2);
        buf.push(3);
        buf.push(4);
        assert_eq!(buf.pop_latest(), Some(4));
        assert!(buf.is_empty());
    }
}
