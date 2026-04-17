/// Image preprocessing for YOLO inference.
///
/// Converts raw camera frames to the tensor format expected by YOLO:
/// - Resize to model input size (e.g., 640x640)
/// - Normalize pixel values to [0, 1]
/// - Convert HWC → CHW layout
/// - BGR → RGB color conversion

/// Preprocess a raw image for YOLO inference.
///
/// Input: raw BGR8 image bytes (H*W*3)
/// Output: CHW float32 tensor as Vec<f32> (3 * target_h * target_w)
pub fn preprocess_image(
    data: &[u8],
    src_width: u32,
    src_height: u32,
    target_size: u32,
) -> Vec<f32> {
    let tw = target_size as usize;
    let th = target_size as usize;
    let sw = src_width as usize;
    let sh = src_height as usize;

    let mut output = vec![0.0f32; 3 * th * tw];

    // Simple bilinear resize + normalize + HWC→CHW + BGR→RGB
    for ty in 0..th {
        for tx in 0..tw {
            // Map target pixel to source pixel
            let sx = (tx as f64 * sw as f64 / tw as f64).min((sw - 1) as f64) as usize;
            let sy = (ty as f64 * sh as f64 / th as f64).min((sh - 1) as f64) as usize;

            let src_idx = (sy * sw + sx) * 3;
            if src_idx + 2 < data.len() {
                let b = data[src_idx] as f32 / 255.0;
                let g = data[src_idx + 1] as f32 / 255.0;
                let r = data[src_idx + 2] as f32 / 255.0;

                // CHW layout: [R plane][G plane][B plane]
                output[0 * th * tw + ty * tw + tx] = r; // R channel
                output[1 * th * tw + ty * tw + tx] = g; // G channel
                output[2 * th * tw + ty * tw + tx] = b; // B channel
            }
        }
    }

    output
}

/// Compute scale factors for mapping detections back to original image.
pub fn compute_scale(src_width: u32, src_height: u32, target_size: u32) -> (f32, f32) {
    (
        src_width as f32 / target_size as f32,
        src_height as f32 / target_size as f32,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_preprocess_size() {
        let data = vec![128u8; 640 * 480 * 3]; // 640x480 BGR
        let tensor = preprocess_image(&data, 640, 480, 320);
        assert_eq!(tensor.len(), 3 * 320 * 320);
    }

    #[test]
    fn test_preprocess_normalization() {
        // All white pixels (255, 255, 255)
        let data = vec![255u8; 4 * 4 * 3];
        let tensor = preprocess_image(&data, 4, 4, 4);
        // Should be ~1.0 after normalization
        assert!((tensor[0] - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_scale_factors() {
        let (sx, sy) = compute_scale(640, 480, 320);
        assert!((sx - 2.0).abs() < 0.01);
        assert!((sy - 1.5).abs() < 0.01);
    }
}
