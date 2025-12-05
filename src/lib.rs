use pyo3::prelude::*;

mod parser;
mod protocol;
mod structs;

use parser::MsclParser;
use structs::IMUPacket;

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

    fn stop(&mut self) {
        self.inner.stop();
    }

    fn get_data_packets(&mut self) -> PyResult<Vec<IMUPacket>> {
        if let Some(err_msg) = self.inner.check_error() {
            return Err(pyo3::exceptions::PyIOError::new_err(err_msg));
        }
        Ok(self.inner.get_all_packets())
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

    fn stop(&mut self) {
        self.inner.stop();
    }

    fn get_data_packets(&mut self) -> PyResult<Vec<IMUPacket>> {
        if let Some(err_msg) = self.inner.check_error() {
            return Err(pyo3::exceptions::PyIOError::new_err(err_msg));
        }
        Ok(self.inner.get_all_packets())
    }
}

#[pymodule(gil_used = false)]
fn mscl_rs(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<SerialParser>()?;
    m.add_class::<MockParser>()?;
    m.add_class::<IMUPacket>()?;
    Ok(())
}
