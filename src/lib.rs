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

            fn get_data_packets(&mut self) -> PyResult<Vec<IMUPacket>> {
                if let Some(err_msg) = self.inner.check_error() {
                    return Err(pyo3::exceptions::PyIOError::new_err(err_msg));
                }
                Ok(self.inner.get_all_packets())
            }

            fn is_running(&self) -> bool {
                self.inner.is_running()
            }
        }
    };
}

#[pyclass(unsendable)]
struct SerialParser {
    inner: MsclParser,
}

impl_parser!(
    SerialParser,
    #[new]
    #[pyo3(signature=(port, baudrate=None, timeout=None))]
    fn new(port: String, baudrate: Option<u32>, timeout: Option<f64>) -> PyResult<Self> {
        let baudrate = baudrate.unwrap_or(115200);
        let timeout = timeout.unwrap_or(0.0);
        let inner = MsclParser::new_serial(&port, baudrate, timeout)
            .map_err(|e| pyo3::exceptions::PyIOError::new_err(e.to_string()))?;
        Ok(SerialParser { inner })
    }
);

#[pyclass(unsendable)]
struct MockParser {
    inner: MsclParser,
}

impl_parser!(
    MockParser,
    #[new]
    fn new(path: String) -> PyResult<Self> {
        let inner = MsclParser::new_mock(&path)
            .map_err(|e| pyo3::exceptions::PyIOError::new_err(e.to_string()))?;
        Ok(MockParser { inner })
    }
);

#[pymodule(gil_used = false)]
fn mscl_rs(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<SerialParser>()?;
    m.add_class::<MockParser>()?;
    m.add_class::<IMUPacket>()?;
    Ok(())
}
