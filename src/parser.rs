use serialport::{DataBits, FlowControl, Parity, StopBits};
use std::io::{self, Read};
use std::sync::mpsc::{Receiver, channel};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crate::structs::{EstimatedDataPacket, ImuPacket, RawDataPacket};

pub(crate) fn fletcher_checksum(data: &[u8]) -> (u8, u8) {
    let (mut a, mut b) = (0u8, 0u8);
    for &x in data {
        a = a.wrapping_add(x);
        b = b.wrapping_add(a);
    }
    (a, b)
}

fn read_f32(d: &[u8]) -> f32 {
    f32::from_be_bytes(d.try_into().unwrap())
}
fn read_f64(d: &[u8]) -> f64 {
    f64::from_be_bytes(d.try_into().unwrap())
}
fn read_u16(d: &[u8]) -> u16 {
    u16::from_be_bytes(d.try_into().unwrap())
}

pub fn decode_packet(desc_set: u8, payload: &[u8]) -> Option<ImuPacket> {
    let timestamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let mut invalid = Vec::new();
    let mut i = 0;

    match desc_set {
        0x80 => {
            let mut pkt = RawDataPacket {
                timestamp,
                invalid_fields: None,
                scaled_accel: None,
                scaled_gyro: None,
                delta_vel: None,
                delta_theta: None,
                scaled_ambient_pressure: None,
            };
            while i < payload.len() {
                let len = payload[i] as usize;
                if len < 2 || i + len > payload.len() {
                    break;
                }
                let desc = payload[i + 1];
                let data = &payload[i + 2..i + len];

                match desc {
                    0x04 if data.len() == 12 => {
                        pkt.scaled_accel = Some([
                            read_f32(&data[0..4]),
                            read_f32(&data[4..8]),
                            read_f32(&data[8..12]),
                        ])
                    }
                    0x05 if data.len() == 12 => {
                        pkt.scaled_gyro = Some([
                            read_f32(&data[0..4]),
                            read_f32(&data[4..8]),
                            read_f32(&data[8..12]),
                        ])
                    }
                    0x07 if data.len() == 12 => {
                        pkt.delta_theta = Some([
                            read_f32(&data[0..4]),
                            read_f32(&data[4..8]),
                            read_f32(&data[8..12]),
                        ])
                    }
                    0x08 if data.len() == 12 => {
                        pkt.delta_vel = Some([
                            read_f32(&data[0..4]),
                            read_f32(&data[4..8]),
                            read_f32(&data[8..12]),
                        ])
                    }
                    0x17 if data.len() == 4 => pkt.scaled_ambient_pressure = Some(read_f32(data)),
                    0x12 if data.len() == 12 => {
                        pkt.timestamp = (read_f64(&data[0..8]) * 1e9) as u128
                    }
                    _ => {}
                }
                i += len;
            }
            if !invalid.is_empty() {
                pkt.invalid_fields = Some(invalid.join(","));
            }
            Some(ImuPacket::Raw(pkt))
        }
        0x82 => {
            let mut pkt = EstimatedDataPacket {
                timestamp,
                invalid_fields: None,
                est_pressure_alt: None,
                est_orient_quaternion: None,
                est_attitude_uncert_quaternion: None,
                est_angular_rate: None,
                est_compensated_accel: None,
                est_linear_accel: None,
                est_gravity_vector: None,
            };
            while i < payload.len() {
                let len = payload[i] as usize;
                if len < 2 || i + len > payload.len() {
                    break;
                }
                let desc = payload[i + 1];
                let data = &payload[i + 2..i + len];

                macro_rules! est_field {
                    ($field:ident, $name:expr, $len:expr, $parse:expr) => {
                        if data.len() == $len {
                            let flags = read_u16(&data[$len - 2..]);
                            if flags & 1 != 0 {
                                pkt.$field = Some($parse);
                            } else {
                                invalid.push($name.to_string());
                            }
                        }
                    };
                }

                match desc {
                    0x21 => {
                        est_field!(est_pressure_alt, "estPressureAlt", 6, read_f32(&data[0..4]))
                    }
                    0x03 => est_field!(
                        est_orient_quaternion,
                        "estOrientQuaternion",
                        18,
                        [
                            read_f32(&data[0..4]),
                            read_f32(&data[4..8]),
                            read_f32(&data[8..12]),
                            read_f32(&data[12..16])
                        ]
                    ),
                    0x12 => est_field!(
                        est_attitude_uncert_quaternion,
                        "estAttitudeUncertQuaternion",
                        18,
                        [
                            read_f32(&data[0..4]),
                            read_f32(&data[4..8]),
                            read_f32(&data[8..12]),
                            read_f32(&data[12..16])
                        ]
                    ),
                    0x0E => est_field!(
                        est_angular_rate,
                        "estAngularRate",
                        14,
                        [
                            read_f32(&data[0..4]),
                            read_f32(&data[4..8]),
                            read_f32(&data[8..12])
                        ]
                    ),
                    0x1C => est_field!(
                        est_compensated_accel,
                        "estCompensatedAccel",
                        14,
                        [
                            read_f32(&data[0..4]),
                            read_f32(&data[4..8]),
                            read_f32(&data[8..12])
                        ]
                    ),
                    0x0D => est_field!(
                        est_linear_accel,
                        "estLinearAccel",
                        14,
                        [
                            read_f32(&data[0..4]),
                            read_f32(&data[4..8]),
                            read_f32(&data[8..12])
                        ]
                    ),
                    0x13 => est_field!(
                        est_gravity_vector,
                        "estGravityVector",
                        14,
                        [
                            read_f32(&data[0..4]),
                            read_f32(&data[4..8]),
                            read_f32(&data[8..12])
                        ]
                    ),
                    0x11 if data.len() == 12 => {
                        pkt.timestamp = (read_f64(&data[0..8]) * 1e9) as u128
                    }
                    _ => {}
                }
                i += len;
            }
            if !invalid.is_empty() {
                pkt.invalid_fields = Some(invalid.join(","));
            }
            Some(ImuPacket::Estimated(pkt))
        }
        _ => None,
    }
}

pub struct SerialParser {
    receiver: Receiver<ImuPacket>,
    running: Arc<AtomicBool>,
    join_handle: Option<thread::JoinHandle<()>>,
    sender: Option<std::sync::mpsc::Sender<ImuPacket>>,
    serial_port: Option<Box<dyn serialport::SerialPort>>,
}

impl SerialParser {
    pub fn new(port: &str, baudrate: u32, _timeout: Duration) -> io::Result<Self> {
        let ser = serialport::new(port, baudrate)
            .data_bits(DataBits::Eight)
            .flow_control(FlowControl::None)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .timeout(Duration::from_millis(100))
            .open_native()
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

        let (sender, receiver) = channel();
        let running = Arc::new(AtomicBool::new(true));

        Ok(Self {
            receiver,
            running,
            join_handle: None,
            sender: Some(sender),
            serial_port: Some(Box::new(ser)),
        })
    }

    pub fn start(&mut self) {
        if self.join_handle.is_some() {
            return;
        }

        if let (Some(mut ser), Some(sender)) = (self.serial_port.take(), self.sender.take()) {
            let running_clone = self.running.clone();

            let join_handle = thread::spawn(move || {
                let mut buffer: Vec<u8> = Vec::with_capacity(4096);
                let mut buf = [0u8; 2048];

                while running_clone.load(Ordering::Relaxed) {
                    match ser.read(&mut buf) {
                        Ok(n) if n > 0 => {
                            buffer.extend_from_slice(&buf[..n]);

                            let mut i = 0;
                            while i < buffer.len() {
                                if buffer[i] == 0x75 {
                                    if i + 1 >= buffer.len() {
                                        break;
                                    }
                                    if buffer[i + 1] == 0x65 {
                                        if i + 4 > buffer.len() {
                                            break;
                                        }
                                        let len = buffer[i + 3] as usize;
                                        let total = 4 + len + 2;
                                        if i + total > buffer.len() {
                                            break;
                                        }

                                        let pkt_bytes = &buffer[i..i + total];
                                        let (cka, ckb) = fletcher_checksum(&pkt_bytes[..total - 2]);
                                        if cka == pkt_bytes[total - 2]
                                            && ckb == pkt_bytes[total - 1]
                                        {
                                            if let Some(pkt) =
                                                decode_packet(pkt_bytes[2], &pkt_bytes[4..4 + len])
                                            {
                                                if sender.send(pkt).is_err() {
                                                    return;
                                                }
                                            }
                                            i += total;
                                            continue;
                                        }
                                    }
                                }
                                i += 1;
                            }
                            buffer.drain(0..i);
                        }
                        Ok(_) => {}
                        Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {}
                        Err(_) => break,
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

impl Drop for SerialParser {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        if let Some(handle) = self.join_handle.take() {
            let _ = handle.join();
        }
    }
}

#[cfg(test)]
mod parser_tests;
