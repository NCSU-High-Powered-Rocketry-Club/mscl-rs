#[derive(Debug)]
pub struct RawDataPacket {
    pub timestamp: u128,
    pub invalid_fields: Option<String>,
    pub scaled_accel: Option<[f32; 3]>,
    pub scaled_gyro: Option<[f32; 3]>,
    pub delta_vel: Option<[f32; 3]>,
    pub delta_theta: Option<[f32; 3]>,
    pub scaled_ambient_pressure: Option<f32>,
}

#[derive(Debug)]
pub struct EstimatedDataPacket {
    pub timestamp: u128,
    pub invalid_fields: Option<String>,
    pub est_pressure_alt: Option<f32>,
    pub est_orient_quaternion: Option<[f32; 4]>,
    pub est_attitude_uncert_quaternion: Option<[f32; 4]>,
    pub est_angular_rate: Option<[f32; 3]>,
    pub est_compensated_accel: Option<[f32; 3]>,
    pub est_linear_accel: Option<[f32; 3]>,
    pub est_gravity_vector: Option<[f32; 3]>,
}

#[derive(Debug)]
pub enum ImuPacket {
    Raw(RawDataPacket),
    Estimated(EstimatedDataPacket),
}
