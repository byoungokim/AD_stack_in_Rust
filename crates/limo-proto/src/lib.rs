/// Limo Drive shared Protobuf message definitions.
///
/// All inter-process messages are defined in `proto/*.proto` and compiled
/// via prost. This crate is the single source of truth for message types
/// used across all processes.
pub mod limo {
    include!(concat!(env!("OUT_DIR"), "/limo.rs"));
}

// Re-export commonly used types at the crate root for convenience.
pub use limo::*;
