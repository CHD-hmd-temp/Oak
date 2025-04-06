#![allow(dead_code)]
pub fn mid360_to_bevy(
    x: f32,
    y: f32,
    z: f32,
) -> nalgebra::Point3<f32> {
    return nalgebra::Point3::new(
        -y,
        z,
        -x,
    );
}

pub fn transform_to_normal_position(
    x: f32,
    y: f32,
    z: f32,
) -> nalgebra::Point3<f32> {
    // 第一步：绕 X 轴旋转 180 度（倒置）
    let inverted = nalgebra::Point3::new(x, -y, -z);
    
    // 第二步：绕 Y 轴旋转 -30 度（修正 x 轴倾斜）
    rotate_point_around_y_axis(&inverted, -0.0_f32.to_radians())
}

fn rotate_point_around_y_axis(
    point: &nalgebra::Point3<f32>,
    angle: f32,
) -> nalgebra::Point3<f32> {
    let cos_angle = angle.cos();
    let sin_angle = angle.sin();

    let x = point.x * cos_angle + point.z * sin_angle;
    let z = -point.x * sin_angle + point.z * cos_angle;
    let y = point.y;

    nalgebra::Point3::new(x, y, z)
}
