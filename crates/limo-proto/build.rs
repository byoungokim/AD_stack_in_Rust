use std::io::Result;

fn main() -> Result<()> {
    // Without this, edits to ../../proto/*.proto (outside the crate dir)
    // never trigger regeneration and stale message structs linger.
    println!("cargo:rerun-if-changed=../../proto");
    prost_build::compile_protos(
        &[
            "../../proto/common.proto",
            "../../proto/sensor.proto",
            "../../proto/perception.proto",
            "../../proto/planning.proto",
            "../../proto/control.proto",
            "../../proto/world_state.proto",
            "../../proto/system.proto",
            "../../proto/sim.proto",
            "../../proto/scenario.proto",
            "../../proto/visualization.proto",
        ],
        &["../../proto/"],
    )?;
    Ok(())
}
