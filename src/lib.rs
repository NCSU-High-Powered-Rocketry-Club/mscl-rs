use pyo3::prelude::*;

mod parser;
mod structs;

use parser::SerialParser;
use structs::ImuPacket;

#[pyclass(frozen)]
#[derive(Debug, Clone)]
pub struct IMUPacket {
    #[pyo3(get)]
    pub packet_type: String,
    #[pyo3(get)]
    pub timestamp: u128,
    #[pyo3(get)]
    pub invalid_fields: Option<String>,
    #[pyo3(get)]
    pub scaled_accel: Option<[f32; 3]>,
    #[pyo3(get)]
    pub scaled_gyro: Option<[f32; 3]>,
    #[pyo3(get)]
    pub delta_vel: Option<[f32; 3]>,
    #[pyo3(get)]
    pub delta_theta: Option<[f32; 3]>,
    #[pyo3(get)]
    pub scaled_ambient_pressure: Option<f32>,
    #[pyo3(get)]
    pub est_pressure_alt: Option<f32>,
    #[pyo3(get)]
    pub est_orient_quaternion: Option<[f32; 4]>,
    #[pyo3(get)]
    pub est_attitude_uncert_quaternion: Option<[f32; 4]>,
    #[pyo3(get)]
    pub est_angular_rate: Option<[f32; 3]>,
    #[pyo3(get)]
    pub est_compensated_accel: Option<[f32; 3]>,
    #[pyo3(get)]
    pub est_linear_accel: Option<[f32; 3]>,
    #[pyo3(get)]
    pub est_gravity_vector: Option<[f32; 3]>,
}

#[pyclass(unsendable)]
struct PySerialParser {
    inner: SerialParser,
}

#[pymethods]
impl PySerialParser {
    #[new]
    fn new(port: String, baudrate: u32, timeout: f64) -> PyResult<Self> {
        let timeout_duration = std::time::Duration::from_secs_f64(timeout);
        let inner = SerialParser::new(&port, baudrate, timeout_duration)
            .map_err(|e| pyo3::exceptions::PyIOError::new_err(e.to_string()))?;
        Ok(PySerialParser { inner })
    }

    fn start(&mut self) {
        self.inner.start();
    }

    fn get_data_packets(&mut self) -> PyResult<Vec<IMUPacket>> {
        let packets = self.inner.get_all_packets();
        let mut py_packets = Vec::with_capacity(packets.len());

        for packet in packets {
            let imu_packet = match packet {
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
            };
            py_packets.push(imu_packet);
        }

        Ok(py_packets)
    }
}

#[pymodule]
fn mscl_rs(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PySerialParser>()?;
    m.add_class::<IMUPacket>()?;
    Ok(())
}
