use pyo3::prelude::*;

#[derive(Debug, Default)]
pub struct RawDataPacket {
    pub timestamp: u128,
    pub invalid_fields: Option<String>,
    pub scaled_accel: Option<[f32; 3]>,
    pub scaled_gyro: Option<[f32; 3]>,
    pub delta_vel: Option<[f32; 3]>,
    pub delta_theta: Option<[f32; 3]>,
    pub scaled_ambient_pressure: Option<f32>,
}

#[derive(Debug, Default)]
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

#[pyclass(frozen, freelist = 20, get_all)]
#[derive(Debug, Clone)]
pub struct IMUPacket {
    pub packet_type: String,
    pub timestamp: u128,
    pub invalid_fields: Option<String>,
    pub scaled_accel: Option<[f32; 3]>,
    pub scaled_gyro: Option<[f32; 3]>,
    pub delta_vel: Option<[f32; 3]>,
    pub delta_theta: Option<[f32; 3]>,
    pub scaled_ambient_pressure: Option<f32>,
    pub est_pressure_alt: Option<f32>,
    pub est_orient_quaternion: Option<[f32; 4]>,
    pub est_attitude_uncert_quaternion: Option<[f32; 4]>,
    pub est_angular_rate: Option<[f32; 3]>,
    pub est_compensated_accel: Option<[f32; 3]>,
    pub est_linear_accel: Option<[f32; 3]>,
    pub est_gravity_vector: Option<[f32; 3]>,
}

impl From<ImuPacket> for IMUPacket {
    fn from(packet: ImuPacket) -> Self {
        match packet {
            ImuPacket::Raw(r) => IMUPacket {
                packet_type: "raw".to_string(),
                timestamp: r.timestamp,
                invalid_fields: r.invalid_fields,
                scaled_accel: r.scaled_accel,
                scaled_gyro: r.scaled_gyro,
                delta_vel: r.delta_vel,
                delta_theta: r.delta_theta,
                scaled_ambient_pressure: r.scaled_ambient_pressure,
                est_pressure_alt: None,
                est_orient_quaternion: None,
                est_attitude_uncert_quaternion: None,
                est_angular_rate: None,
                est_compensated_accel: None,
                est_linear_accel: None,
                est_gravity_vector: None,
            },
            ImuPacket::Estimated(e) => IMUPacket {
                packet_type: "estimated".to_string(),
                timestamp: e.timestamp,
                invalid_fields: e.invalid_fields,
                scaled_accel: None,
                scaled_gyro: None,
                delta_vel: None,
                delta_theta: None,
                scaled_ambient_pressure: None,
                est_pressure_alt: e.est_pressure_alt,
                est_orient_quaternion: e.est_orient_quaternion,
                est_attitude_uncert_quaternion: e.est_attitude_uncert_quaternion,
                est_angular_rate: e.est_angular_rate,
                est_compensated_accel: e.est_compensated_accel,
                est_linear_accel: e.est_linear_accel,
                est_gravity_vector: e.est_gravity_vector,
            },
        }
    }
}
