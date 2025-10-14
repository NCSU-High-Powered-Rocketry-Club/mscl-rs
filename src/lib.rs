use pyo3::prelude::*;
use pyo3::types::PyTuple;
use pyo3::IntoPyObjectExt;
use std::sync::Mutex;

mod parser;
mod structs;

use parser::SerialParser;
use structs::ImuPacket;

#[pyclass]
struct PySerialParser {
    inner: Mutex<SerialParser>,
}

#[pymethods]
impl PySerialParser {
    #[new]
    fn new(port: String, baudrate: u32, timeout: f64) -> PyResult<Self> {
        let timeout_duration = std::time::Duration::from_secs_f64(timeout);
        let inner = SerialParser::new(&port, baudrate, timeout_duration)
            .map_err(|e| pyo3::exceptions::PyIOError::new_err(e.to_string()))?;
        Ok(PySerialParser {
            inner: Mutex::new(inner),
        })
    }

    fn get_data_packets(slf: &Bound<'_, Self>) -> PyResult<Vec<Py<PyAny>>> {
        let py = slf.py();
        let binding = slf.borrow();
        let mut guard = binding.inner.lock().unwrap();
        let packets = guard.get_all_packets();
        drop(guard);

        let mut py_packets = Vec::with_capacity(packets.len());

        for packet in packets {
            let py_packet = match packet {
                ImuPacket::Raw(r) => {
                    let mut elements: Vec<Py<PyAny>> = Vec::new();
                    
                    // Add packet type
                    elements.push("raw".into_py_any(py).unwrap());
                    
                    // Add timestamp
                    elements.push(r.timestamp.into_py_any(py).unwrap());
                    
                    // Add invalid_fields
                    elements.push(r.invalid_fields.into_py_any(py).unwrap());
                    
                    // Add accel components (or None for each if missing)
                    if let Some([x, y, z]) = r.scaled_accel {
                        elements.push(x.into_py_any(py).unwrap());
                        elements.push(y.into_py_any(py).unwrap());
                        elements.push(z.into_py_any(py).unwrap());
                    } else {
                        elements.push(py.None());
                        elements.push(py.None());
                        elements.push(py.None());
                    }
                    
                    // Add gyro components (or None for each if missing)
                    if let Some([x, y, z]) = r.scaled_gyro {
                        elements.push(x.into_py_any(py).unwrap());
                        elements.push(y.into_py_any(py).unwrap());
                        elements.push(z.into_py_any(py).unwrap());
                    } else {
                        elements.push(py.None());
                        elements.push(py.None());
                        elements.push(py.None());
                    }
                    
                    // Add delta_vel components (or None for each if missing)
                    if let Some([x, y, z]) = r.delta_vel {
                        elements.push(x.into_py_any(py).unwrap());
                        elements.push(y.into_py_any(py).unwrap());
                        elements.push(z.into_py_any(py).unwrap());
                    } else {
                        elements.push(py.None());
                        elements.push(py.None());
                        elements.push(py.None());
                    }
                    
                    // Add delta_theta components (or None for each if missing)
                    if let Some([x, y, z]) = r.delta_theta {
                        elements.push(x.into_py_any(py).unwrap());
                        elements.push(y.into_py_any(py).unwrap());
                        elements.push(z.into_py_any(py).unwrap());
                    } else {
                        elements.push(py.None());
                        elements.push(py.None());
                        elements.push(py.None());
                    }
                    
                    // Add pressure
                    elements.push(r.scaled_ambient_pressure.into_py_any(py).unwrap());

                    PyTuple::new(py, elements)
                        .unwrap()
                        .into_any()
                        .unbind()
                }
                ImuPacket::Estimated(e) => {
                    let mut elements: Vec<Py<PyAny>> = Vec::new();
                    
                    // Add packet type
                    elements.push("estimated".into_py_any(py).unwrap());
                    
                    // Add timestamp
                    elements.push(e.timestamp.into_py_any(py).unwrap());
                    
                    // Add invalid_fields
                    elements.push(e.invalid_fields.into_py_any(py).unwrap());
                    
                    // Add pressure_alt
                    elements.push(e.est_pressure_alt.into_py_any(py).unwrap());
                    
                    // Add orient_quat components (or None for each if missing)
                    if let Some([w, x, y, z]) = e.est_orient_quaternion {
                        elements.push(w.into_py_any(py).unwrap());
                        elements.push(x.into_py_any(py).unwrap());
                        elements.push(y.into_py_any(py).unwrap());
                        elements.push(z.into_py_any(py).unwrap());
                    } else {
                        elements.push(py.None());
                        elements.push(py.None());
                        elements.push(py.None());
                        elements.push(py.None());
                    }
                    
                    // Add attitude_uncert_quat components (or None for each if missing)
                    if let Some([w, x, y, z]) = e.est_attitude_uncert_quaternion {
                        elements.push(w.into_py_any(py).unwrap());
                        elements.push(x.into_py_any(py).unwrap());
                        elements.push(y.into_py_any(py).unwrap());
                        elements.push(z.into_py_any(py).unwrap());
                    } else {
                        elements.push(py.None());
                        elements.push(py.None());
                        elements.push(py.None());
                        elements.push(py.None());
                    }
                    
                    // Add angular_rate components (or None for each if missing)
                    if let Some([x, y, z]) = e.est_angular_rate {
                        elements.push(x.into_py_any(py).unwrap());
                        elements.push(y.into_py_any(py).unwrap());
                        elements.push(z.into_py_any(py).unwrap());
                    } else {
                        elements.push(py.None());
                        elements.push(py.None());
                        elements.push(py.None());
                    }
                    
                    // Add compensated_accel components (or None for each if missing)
                    if let Some([x, y, z]) = e.est_compensated_accel {
                        elements.push(x.into_py_any(py).unwrap());
                        elements.push(y.into_py_any(py).unwrap());
                        elements.push(z.into_py_any(py).unwrap());
                    } else {
                        elements.push(py.None());
                        elements.push(py.None());
                        elements.push(py.None());
                    }
                    
                    // Add linear_accel components (or None for each if missing)
                    if let Some([x, y, z]) = e.est_linear_accel {
                        elements.push(x.into_py_any(py).unwrap());
                        elements.push(y.into_py_any(py).unwrap());
                        elements.push(z.into_py_any(py).unwrap());
                    } else {
                        elements.push(py.None());
                        elements.push(py.None());
                        elements.push(py.None());
                    }
                    
                    // Add gravity_vector components (or None for each if missing)
                    if let Some([x, y, z]) = e.est_gravity_vector {
                        elements.push(x.into_py_any(py).unwrap());
                        elements.push(y.into_py_any(py).unwrap());
                        elements.push(z.into_py_any(py).unwrap());
                    } else {
                        elements.push(py.None());
                        elements.push(py.None());
                        elements.push(py.None());
                    }

                    PyTuple::new(py, elements)
                        .unwrap()
                        .into_any()
                        .unbind()
                }
            };
            py_packets.push(py_packet);
        }

        Ok(py_packets)
    }
}

#[pymodule]
fn mscl_rs(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PySerialParser>()?;
    Ok(())
}
