use std::path::PathBuf;

use pyo3::prelude::*;

pub mod parser;
mod protocol;
pub mod structs;

use parser::MsclParser;
use structs::IMUPacket;

macro_rules! impl_parser {
    ($struct_name:ident, $new_method:item) => {
        #[pymethods]
        impl $struct_name {
            $new_method

            fn start(&mut self) {
                self.inner.start();
            }

            fn stop(&mut self) {
                self.inner.stop();
            }

            #[pyo3(signature = (block=false))]
            fn get_data_packets(&mut self, block: bool) -> PyResult<Vec<IMUPacket>> {
                if let Some(err_msg) = self.inner.check_error() {
                    return Err(pyo3::exceptions::PyIOError::new_err(err_msg));
                }
                let timeout = if block {
                    Some(std::time::Duration::from_secs_f64(self.timeout))
                } else {
                    None
                };

                // Get all packets, and return early if there's an error
                let packets = self.inner.get_packets(timeout)
                             .map_err(|e| pyo3::exceptions::PyIOError::new_err(e.to_string()))?;
                Ok(packets)
            }

            fn is_running(&self) -> bool {
                self.inner.is_running()
            }

            fn __enter__(slf: Bound<'_, Self>) -> PyResult<Bound<'_, Self>> {
                slf.borrow_mut().start();
                Ok(slf)
            }

            fn __exit__(
                slf: Bound<'_, Self>,
                _exc_type: Option<Bound<'_, PyAny>>,
                _exc_value: Option<Bound<'_, PyAny>>,
                _traceback: Option<Bound<'_, PyAny>>,
            ) -> PyResult<()> {
                slf.borrow_mut().stop();
                Ok(())
            }
        }
    };
}

#[pyclass(unsendable)]
struct SerialParser {
    inner: MsclParser,
    timeout: f64,
}

impl_parser!(
    SerialParser,
    #[new]
    #[pyo3(signature=(port, baudrate=None, timeout=0.1))]
    fn new(port: PathBuf, baudrate: Option<u32>, timeout: Option<f64>) -> PyResult<Self> {
        let baudrate = baudrate.unwrap_or(115200);
        let timeout_val = timeout.unwrap_or(0.0);
        let inner = MsclParser::new_serial(&port, baudrate, timeout_val)
            .map_err(|e| pyo3::exceptions::PyIOError::new_err(e.to_string()))?;
        Ok(SerialParser { inner, timeout: timeout_val })
    }
);

#[pyclass(unsendable)]
struct MockParser {
    inner: MsclParser,
    timeout: f64,
}

impl_parser!(
    MockParser,
    #[new]
    #[pyo3(signature=(path, timeout=0.1))]
    fn new(path: PathBuf, timeout: Option<f64>) -> PyResult<Self> {
        let inner = MsclParser::new_mock(&path)
            .map_err(|e| pyo3::exceptions::PyIOError::new_err(e.to_string()))?;
        Ok(MockParser { inner, timeout: timeout.unwrap_or(0.0) })
    }
);

#[pymodule(gil_used = false)]
fn mscl_rs(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<SerialParser>()?;
    m.add_class::<MockParser>()?;
    m.add_class::<IMUPacket>()?;
    m.add("VERSION", env!("CARGO_PKG_VERSION"))?;
    m.add("RELEASE_BUILD", cfg!(not(debug_assertions)))?;
    Ok(())
}
