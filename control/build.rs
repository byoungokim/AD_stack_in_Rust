use std::io::Result;

fn main() -> Result<()> {
    prost_build::compile_protos(
        &[
            "../proto/common.proto",
            "../proto/control.proto",
            "../proto/planning.proto",
            "../proto/system.proto",
        ],
        &["../proto/"],
    )?;
    Ok(())
}
