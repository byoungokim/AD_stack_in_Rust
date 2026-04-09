use std::io::Result;

fn main() -> Result<()> {
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
