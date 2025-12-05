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

impl Default for EstimatedDataPacket {
    fn default() -> Self {
        EstimatedDataPacket {
            timestamp: 0,
            invalid_fields: None,
            est_pressure_alt: None,
            est_orient_quaternion: None,
            est_attitude_uncert_quaternion: None,
            est_angular_rate: None,
            est_compensated_accel: None,
            est_linear_accel: None,
            est_gravity_vector: None,
        }
    }
}

impl Default for RawDataPacket {
    fn default() -> Self {
        RawDataPacket {
            timestamp: 0,
            invalid_fields: None,
            scaled_accel: None,
            scaled_gyro: None,
            delta_vel: None,
            delta_theta: None,
            scaled_ambient_pressure: None,
        }
    }
}

#[derive(Debug)]
pub enum ImuPacket {
    Raw(RawDataPacket),
    Estimated(EstimatedDataPacket),
}
