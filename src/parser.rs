use std::fs::File;
use std::io::{self, Read};
use std::sync::mpsc::{Receiver, channel};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::thread;
use std::time::Duration;

use crate::structs::ImuPacket;
use crate::protocol::{fletcher_checksum, decode_packet};

pub struct MsclParser {
    receiver: Receiver<ImuPacket>,
    running: Arc<AtomicBool>,
    join_handle: Option<thread::JoinHandle<()>>,
    sender: Option<std::sync::mpsc::Sender<ImuPacket>>,
    source: Option<Box<dyn Read + Send>>,
}

impl MsclParser {
    pub fn new_serial(port: &str, baudrate: u32, timeout: f64) -> io::Result<Self> {
        let ser = serialport::new(port, baudrate)
            .data_bits(serialport::DataBits::Eight)
            .flow_control(serialport::FlowControl::None)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .timeout(Duration::from_millis((timeout * 1000.0) as u64))
            .open_native()
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

        let (sender, receiver) = channel();
        let running = Arc::new(AtomicBool::new(true));

        Ok(Self {
            receiver,
            running,
            join_handle: None,
            sender: Some(sender),
            source: Some(Box::new(ser)),
        })
    }

    pub fn new_mock(path: &str) -> io::Result<Self> {
        let file = File::open(path)?;
        let (sender, receiver) = channel();
        let running = Arc::new(AtomicBool::new(true));

        Ok(Self {
            receiver,
            running,
            join_handle: None,
            sender: Some(sender),
            source: Some(Box::new(file)),
        })
    }

    pub fn start(&mut self) {
        if self.join_handle.is_some() {
            return;
        }

        if let (Some(mut source), Some(sender)) = (self.source.take(), self.sender.take()) {
            let running_clone = self.running.clone();

            let join_handle = thread::spawn(move || {
                let mut buffer: Vec<u8> = Vec::with_capacity(4096);
                let mut buf = [0u8; 2048];

                while running_clone.load(Ordering::Relaxed) {
                    match source.read(&mut buf) {
                        Ok(n) if n > 0 => {
                            buffer.extend_from_slice(&buf[..n]);
                            process_buffer(&mut buffer, &sender);
                        }
                        Ok(_) => break, // EOF
                        Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {}
                        Err(_) => {
                            println!("Read error, stopping parser thread");
                            break;
                        }
                    }
                }
            });
            self.join_handle = Some(join_handle);
        }
    }

    pub fn get_all_packets(&mut self) -> Vec<ImuPacket> {
        let mut packets = Vec::new();
        while let Ok(pkt) = self.receiver.try_recv() {
            packets.push(pkt);
        }
        packets
    }
}

fn process_buffer(buffer: &mut Vec<u8>, sender: &std::sync::mpsc::Sender<ImuPacket>) {
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
        if let Some(pkt) = decode_packet(pkt_bytes[2], &pkt_bytes[4..4 + len]) {
            if sender.send(pkt).is_err() {
                return; // Receiver closed
            }
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
mod parser_tests;
