use std::io::Result;

fn main() -> Result<()> {
    // Compile protobuf definitions
    prost_build::compile_protos(
        &[
            "../proto/common.proto",
            "../proto/sensor.proto",
            "../proto/perception.proto",
            "../proto/world_state.proto",
            "../proto/control.proto",
            "../proto/planning.proto",
            "../proto/system.proto",
        ],
        &["../proto/"],
    )?;
    Ok(())
}
