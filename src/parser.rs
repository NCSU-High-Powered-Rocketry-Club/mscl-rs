use std::fs::File;
use std::io::{self, Read};
use std::path::PathBuf;
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender, channel};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::thread;
use std::time::Duration;

use crate::protocol::{decode_packet, fletcher_checksum};
use crate::structs::IMUPacket;

pub struct MsclParser {
    receiver: Receiver<IMUPacket>,
    error_receiver: Receiver<String>,
    running: Arc<AtomicBool>,
    join_handle: Option<thread::JoinHandle<Box<dyn Read + Send>>>,
    sender: Sender<IMUPacket>,
    error_sender: Sender<String>,
    source: Option<Box<dyn Read + Send>>,
}

impl MsclParser {
    pub fn new_serial(port: &PathBuf, baudrate: u32, timeout: f64) -> io::Result<Self> {
        let ser = serialport::new(port.to_str().unwrap(), baudrate)
            .data_bits(serialport::DataBits::Eight)
            .flow_control(serialport::FlowControl::None)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .timeout(Duration::from_millis((timeout * 1000.0) as u64))
            .open_native()
            .map_err(io::Error::other)?;

        Ok(Self::new(Box::new(ser)))
    }

    pub fn new_mock(path: &PathBuf) -> io::Result<Self> {
        let file = File::open(path)?;
        Ok(Self::new(Box::new(file)))
    }

    fn new(source: Box<dyn Read + Send>) -> Self {
        let (sender, receiver) = channel();
        let (error_sender, error_receiver) = channel();
        let running = Arc::new(AtomicBool::new(false));

        Self {
            receiver,
            error_receiver,
            running,
            join_handle: None,
            sender,
            error_sender,
            source: Some(source),
        }
    }

    pub fn start(&mut self) {
        if self.join_handle.is_some() {
            return;
        }

        let mut source = match self.source.take() {
            Some(s) => s,
            None => return,
        };

        self.running.store(true, Ordering::Relaxed);
        let running_clone = self.running.clone();
        let sender = self.sender.clone();
        let error_sender = self.error_sender.clone();

        let join_handle = thread::spawn(move || {
            let mut buffer: Vec<u8> = Vec::with_capacity(4096);

            while running_clone.load(Ordering::Relaxed) {
                let current_len = buffer.len();
                buffer.resize(current_len + 2048, 0);

                match source.read(&mut buffer[current_len..]) {
                    Ok(n) if n > 0 => {
                        buffer.truncate(current_len + n);
                        process_buffer(&mut buffer, &sender);
                    }
                    Ok(_) => {
                        running_clone.store(false, Ordering::Relaxed);
                        break; // EOF
                    }
                    Err(e) => {
                        buffer.truncate(current_len);
                        let _ = error_sender.send(e.to_string());
                        running_clone.store(false, Ordering::Relaxed);
                        break;
                    }
                }
            }
            source
        });
        self.join_handle = Some(join_handle);
    }

    pub fn stop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        if let Some(handle) = self.join_handle.take() {
            if let Ok(source) = handle.join() {
                self.source = Some(source);
            }
        }
    }

    pub fn get_packets(&mut self, timeout: Option<Duration>) -> Result<Vec<IMUPacket>, RecvTimeoutError> {
        let mut packets = Vec::new();

        // If blocking, wait for at most one packet. The next loop will drain any others.
        if let Some(duration) = timeout {
            let pkt = self.receiver.recv_timeout(duration)?;
            packets.push(pkt);
        }

        while let Ok(pkt) = self.receiver.try_recv() {
            packets.push(pkt);
        }
        Ok(packets)
    }

    pub fn get_all_packets(&mut self) -> Result<Vec<IMUPacket>, RecvTimeoutError> {
        self.get_packets(None)
    }

    pub fn check_error(&self) -> Option<String> {
        self.error_receiver.try_recv().ok()
    }

    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::Relaxed)
    }
}

fn process_buffer(buffer: &mut Vec<u8>, sender: &Sender<IMUPacket>) {
    let mut i = 0;
    while i < buffer.len() {
        // Look for start of packet: 0x75, 0x65
        if buffer[i] != 0x75 {
            i += 1;
            continue;
        }

        if i + 1 >= buffer.len() {
            break; // Need more data
        }

        if buffer[i + 1] != 0x65 {
            i += 1;
            continue;
        }

        // Found potential start, check length
        if i + 4 > buffer.len() {
            break; // Need more data for header
        }

        let len = buffer[i + 3] as usize;
        let total_len = 4 + len + 2; // Header(4) + Payload(len) + Checksum(2)

        if i + total_len > buffer.len() {
            break; // Need more data for full packet
        }

        let pkt_bytes = &buffer[i..i + total_len];

        // Verify checksum
        let (cka, ckb) = fletcher_checksum(&pkt_bytes[..total_len - 2]);
        if cka != pkt_bytes[total_len - 2] || ckb != pkt_bytes[total_len - 1] {
            // Invalid checksum, skip this start byte and try again
            i += 1;
            continue;
        }

        // Valid packet found, decode it
        if let Some(pkt) = decode_packet(pkt_bytes[2], &pkt_bytes[4..4 + len])
            && sender.send(IMUPacket::from(pkt)).is_err()
        {
            return;
        }
        // Move past this packet
        i += total_len;
    }

    // Remove processed bytes
    buffer.drain(0..i);
}

impl Drop for MsclParser {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        if let Some(handle) = self.join_handle.take() {
            let _ = handle.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn test_mock_parser() {
        let path = "test_packet_temp.bin";
        let mut file = std::fs::File::create(path).unwrap();
        // Valid packet
        let pkt = [
            0x75, 0x65, 0x80, 0x0E, 0x0E, 0x04, 0x3F, 0x80, 0x00, 0x00, 0x3F, 0x80, 0x00, 0x00,
            0x3F, 0x80, 0x00, 0x00, 0xB7, 0x21,
        ];
        file.write_all(&pkt).unwrap();
        // Invalid packet (wrong checksum)
        let mut bad_pkt = pkt;
        bad_pkt[19] = 0x22;
        file.write_all(&bad_pkt).unwrap();
        // Valid packet again
        file.write_all(&pkt).unwrap();
        drop(file);

        let mut parser = MsclParser::new_mock(&PathBuf::from(path)).unwrap();
        parser.start();
        thread::sleep(Duration::from_millis(100));
        let packets = parser.get_all_packets().unwrap();

        // Cleanup
        let _ = std::fs::remove_file(path);

        assert_eq!(packets.len(), 2);
    }

    #[test]
    fn test_restart_parser() {
        let path = "test_restart.bin";
        let mut file = std::fs::File::create(path).unwrap();
        // Valid packet
        let pkt = [
            0x75, 0x65, 0x80, 0x0E, 0x0E, 0x04, 0x3F, 0x80, 0x00, 0x00, 0x3F, 0x80, 0x00, 0x00,
            0x3F, 0x80, 0x00, 0x00, 0xB7, 0x21,
        ];
        file.write_all(&pkt).unwrap();
        file.write_all(&pkt).unwrap();
        drop(file);

        let mut parser = MsclParser::new_mock(&PathBuf::from(path)).unwrap();

        // First run
        parser.start();
        thread::sleep(Duration::from_millis(50));
        parser.stop();
        let packets1 = parser.get_all_packets().unwrap();

        // Second run
        parser.start();
        thread::sleep(Duration::from_millis(50));
        parser.stop();
        let packets2 = parser.get_all_packets().unwrap();

        // Cleanup
        let _ = std::fs::remove_file(path);

        assert_eq!(packets1.len() + packets2.len(), 2);
    }

    #[test]
    fn test_stop_without_start() {
        let path = "test_stop_no_start.bin";
        let _ = std::fs::File::create(path).unwrap();
        let mut parser = MsclParser::new_mock(&PathBuf::from(path)).unwrap();

        // Should not panic
        parser.stop();

        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn test_blocking_parser() {
        let path = "test_blocking.bin";
        let mut file = std::fs::File::create(path).unwrap();
        // Valid packet
        let pkt = [
            0x75, 0x65, 0x80, 0x0E, 0x0E, 0x04, 0x3F, 0x80, 0x00, 0x00, 0x3F, 0x80, 0x00, 0x00,
            0x3F, 0x80, 0x00, 0x00, 0xB7, 0x21,
        ];
        file.write_all(&pkt).unwrap();
        drop(file);

        let mut parser = MsclParser::new_mock(&PathBuf::from(path)).unwrap();
        parser.start();

        // Should return immediately if data is ready, or wait if not.
        let packets = parser.get_packets(Some(Duration::from_millis(100))).unwrap();

        // Cleanup
        let _ = std::fs::remove_file(path);

        assert_eq!(packets.len(), 1);
    }
}
