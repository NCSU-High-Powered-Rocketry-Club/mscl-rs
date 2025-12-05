use pyo3::prelude::*;

mod parser;
mod protocol;
mod structs;

use parser::MsclParser;
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

#[pyclass(unsendable)]
struct SerialParser {
    inner: MsclParser,
}

#[pymethods]
impl SerialParser {
    #[new]
    #[pyo3(signature=(port, baudrate=None, timeout=None))]
    fn new(port: String, baudrate: Option<u32>, timeout: Option<f64>) -> PyResult<Self> {
        let baudrate = baudrate.unwrap_or(115200);
        let timeout = timeout.unwrap_or(0.0);
        let inner = MsclParser::new_serial(&port, baudrate, timeout)
            .map_err(|e| pyo3::exceptions::PyIOError::new_err(e.to_string()))?;
        Ok(SerialParser { inner })
    }

    fn start(&mut self) {
        self.inner.start();
    }

    fn get_data_packets(&mut self) -> PyResult<Vec<IMUPacket>> {
        let packets = self.inner.get_all_packets();
        Ok(packets.into_iter().map(IMUPacket::from).collect())
    }
}

#[pyclass(unsendable)]
struct MockParser {
    inner: MsclParser,
}

#[pymethods]
impl MockParser {
    #[new]
    fn new(path: String) -> PyResult<Self> {
        let inner = MsclParser::new_mock(&path)
            .map_err(|e| pyo3::exceptions::PyIOError::new_err(e.to_string()))?;
        Ok(MockParser { inner })
    }

    fn start(&mut self) {
        self.inner.start();
    }

    fn get_data_packets(&mut self) -> PyResult<Vec<IMUPacket>> {
        let packets = self.inner.get_all_packets();
        Ok(packets.into_iter().map(IMUPacket::from).collect())
    }
}

#[pymodule]
fn mscl_rs(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<SerialParser>()?;
    m.add_class::<MockParser>()?;
    m.add_class::<IMUPacket>()?;
    Ok(())
}
