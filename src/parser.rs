use serialport::{SerialPort, DataBits, FlowControl, Parity, StopBits};
use std::io::{self, Read};
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use byteorder::{BigEndian, ReadBytesExt};
use std::io::Cursor;

use crate::structs::{EstimatedDataPacket, ImuPacket, RawDataPacket};

pub struct Field {
    descriptor: u8,
    data: Vec<u8>,
}

pub struct MipPacket {
    desc_set: u8,
    payload: Vec<u8>,
    fields: Vec<Field>,
}

pub fn fletcher_checksum(bytes_to_check: &[u8]) -> (u8, u8) {
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;
    for &byte in bytes_to_check {
        ck_a = ck_a.wrapping_add(byte);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    (ck_a, ck_b)
}

pub fn parse_payload(payload: &[u8]) -> Vec<Field> {
    let mut fields = Vec::new();
    let mut i = 0;
    while i < payload.len() {
        if i + 1 >= payload.len() {
            break;
        }
        let field_len = payload[i] as usize;
        if field_len < 2 || i + field_len > payload.len() {
            break;
        }
        let descriptor = payload[i + 1];
        let data = payload[i + 2..i + field_len].to_vec();
        fields.push(Field { descriptor, data });
        i += field_len;
    }
    fields
}

pub fn parse_mip_packet(buffer: &[u8]) -> Option<(MipPacket, usize)> {
    if buffer.len() < 6 {
        return None;
    }
    if buffer[0] != 0x75 || buffer[1] != 0x65 {
        return None;
    }
    let desc_set = buffer[2];
    let payload_len = buffer[3] as usize;
    let total_len = 4 + payload_len + 2;
    if buffer.len() < total_len {
        return None;
    }
    let payload = buffer[4..4 + payload_len].to_vec();
    let ck_a_received = buffer[4 + payload_len];
    let ck_b_received = buffer[4 + payload_len + 1];

    let bytes_to_check = &buffer[0..4 + payload_len];
    let (ck_a, ck_b) = fletcher_checksum(bytes_to_check);

    if ck_a != ck_a_received || ck_b != ck_b_received {
        return None;
    }

    let fields = parse_payload(&payload);

    Some((MipPacket { desc_set, payload, fields }, total_len))
}

pub fn decode_packet(packet: &MipPacket) -> Option<ImuPacket> {
    let timestamp = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_nanos();
    let mut invalid = Vec::new();

    match packet.desc_set {
        0x80 => {
            let mut data_pkt = RawDataPacket {
                timestamp,
                invalid_fields: None,
                scaled_accel: None,
                scaled_gyro: None,
                delta_vel: None,
                delta_theta: None,
                scaled_ambient_pressure: None,
            };
            for field in &packet.fields {
                let mut cursor = Cursor::new(&field.data);
                match field.descriptor {
                    0x04 => { // Scaled Accelerometer Vector
                        if field.data.len() == 12 {
                            let x = cursor.read_f32::<BigEndian>().ok()?;
                            let y = cursor.read_f32::<BigEndian>().ok()?;
                            let z = cursor.read_f32::<BigEndian>().ok()?;
                            data_pkt.scaled_accel = Some([x, y, z]);
                        }
                    }
                    0x05 => { // Scaled Gyro Vector
                        if field.data.len() == 12 {
                            let x = cursor.read_f32::<BigEndian>().ok()?;
                            let y = cursor.read_f32::<BigEndian>().ok()?;
                            let z = cursor.read_f32::<BigEndian>().ok()?;
                            data_pkt.scaled_gyro = Some([x, y, z]);
                        }
                    }
                    0x07 => { // Delta Theta Vector
                        if field.data.len() == 12 {
                            let x = cursor.read_f32::<BigEndian>().ok()?;
                            let y = cursor.read_f32::<BigEndian>().ok()?;
                            let z = cursor.read_f32::<BigEndian>().ok()?;
                            data_pkt.delta_theta = Some([x, y, z]);
                        }
                    }
                    0x08 => { // Delta Velocity Vector
                        if field.data.len() == 12 {
                            let x = cursor.read_f32::<BigEndian>().ok()?;
                            let y = cursor.read_f32::<BigEndian>().ok()?;
                            let z = cursor.read_f32::<BigEndian>().ok()?;
                            data_pkt.delta_vel = Some([x, y, z]);
                        }
                    }
                    0x17 => { // Scaled Ambient Pressure
                        if field.data.len() == 4 {
                            let p = cursor.read_f32::<BigEndian>().ok()?;
                            data_pkt.scaled_ambient_pressure = Some(p);
                        }
                    }
                    _ => {},
                }
            }
            if !invalid.is_empty() {
                data_pkt.invalid_fields = Some(invalid.join(","));
            }
            Some(ImuPacket::Raw(data_pkt))
        }
        0x82 => {
            let mut data_pkt = EstimatedDataPacket {
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
            for field in &packet.fields {
                let mut cursor = Cursor::new(&field.data);
                match field.descriptor {
                    0x21 => { // Pressure Altitude
                        if field.data.len() == 6 {
                            let p = cursor.read_f32::<BigEndian>().ok()?;
                            let flags = cursor.read_u16::<BigEndian>().ok()?;
                            if flags & 0x0001 != 0 {
                                data_pkt.est_pressure_alt = Some(p);
                            } else {
                                invalid.push("estPressureAlt".to_string());
                            }
                        }
                    }
                    0x03 => { // Orientation, Quaternion
                        if field.data.len() == 18 {
                            let w = cursor.read_f32::<BigEndian>().ok()?;
                            let x = cursor.read_f32::<BigEndian>().ok()?;
                            let y = cursor.read_f32::<BigEndian>().ok()?;
                            let z = cursor.read_f32::<BigEndian>().ok()?;
                            let flags = cursor.read_u16::<BigEndian>().ok()?;
                            if flags & 0x0001 != 0 {
                                data_pkt.est_orient_quaternion = Some([w, x, y, z]);
                            } else {
                                invalid.push("estOrientQuaternion".to_string());
                            }
                        }
                    }
                    0x12 => { // Attitude Uncertainty, Quaternion Elements
                        if field.data.len() == 18 {
                            let w = cursor.read_f32::<BigEndian>().ok()?;
                            let x = cursor.read_f32::<BigEndian>().ok()?;
                            let y = cursor.read_f32::<BigEndian>().ok()?;
                            let z = cursor.read_f32::<BigEndian>().ok()?;
                            let flags = cursor.read_u16::<BigEndian>().ok()?;
                            if flags & 0x0001 != 0 {
                                data_pkt.est_attitude_uncert_quaternion = Some([w, x, y, z]);
                            } else {
                                invalid.push("estAttitudeUncertQuaternion".to_string());
                            }
                        }
                    }
                    0x0E => { // Compensated Angular Rate
                        if field.data.len() == 14 {
                            let x = cursor.read_f32::<BigEndian>().ok()?;
                            let y = cursor.read_f32::<BigEndian>().ok()?;
                            let z = cursor.read_f32::<BigEndian>().ok()?;
                            let flags = cursor.read_u16::<BigEndian>().ok()?;
                            if flags & 0x0001 != 0 {
                                data_pkt.est_angular_rate = Some([x, y, z]);
                            } else {
                                invalid.push("estAngularRate".to_string());
                            }
                        }
                    }
                    0x1C => { // Compensated Acceleration
                        if field.data.len() == 14 {
                            let x = cursor.read_f32::<BigEndian>().ok()?;
                            let y = cursor.read_f32::<BigEndian>().ok()?;
                            let z = cursor.read_f32::<BigEndian>().ok()?;
                            let flags = cursor.read_u16::<BigEndian>().ok()?;
                            if flags & 0x0001 != 0 {
                                data_pkt.est_compensated_accel = Some([x, y, z]);
                            } else {
                                invalid.push("estCompensatedAccel".to_string());
                            }
                        }
                    }
                    0x0D => { // Linear Acceleration
                        if field.data.len() == 14 {
                            let x = cursor.read_f32::<BigEndian>().ok()?;
                            let y = cursor.read_f32::<BigEndian>().ok()?;
                            let z = cursor.read_f32::<BigEndian>().ok()?;
                            let flags = cursor.read_u16::<BigEndian>().ok()?;
                            if flags & 0x0001 != 0 {
                                data_pkt.est_linear_accel = Some([x, y, z]);
                            } else {
                                invalid.push("estLinearAccel".to_string());
                            }
                        }
                    }
                    0x13 => { // Gravity Vector
                        if field.data.len() == 14 {
                            let x = cursor.read_f32::<BigEndian>().ok()?;
                            let y = cursor.read_f32::<BigEndian>().ok()?;
                            let z = cursor.read_f32::<BigEndian>().ok()?;
                            let flags = cursor.read_u16::<BigEndian>().ok()?;
                            if flags & 0x0001 != 0 {
                                data_pkt.est_gravity_vector = Some([x, y, z]);
                            } else {
                                invalid.push("estGravityVector".to_string());
                            }
                        }
                    }
                    _ => {},
                }
            }
            if !invalid.is_empty() {
                data_pkt.invalid_fields = Some(invalid.join(","));
            }
            Some(ImuPacket::Estimated(data_pkt))
        }
        _ => None,
    }
}

pub struct SerialParser {
    ser: Box<dyn SerialPort>,
    buffer: Vec<u8>,
}

impl SerialParser {
    pub fn new(port: &str, baudrate: u32, timeout: Duration) -> io::Result<Self> {
        let ser = serialport::new(port, baudrate)
            .data_bits(DataBits::Eight)
            .flow_control(FlowControl::None)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .timeout(timeout)
            .open()?;

        Ok(Self {
            ser,
            buffer: Vec::new(),
        })
    }

    pub fn get_all_packets(&mut self) -> Vec<ImuPacket> {
        let mut packets = Vec::new();
        let mut temp_buffer = vec![0u8; 1024];
        // let max_reads = 100; // Prevent infinite loops
        // let mut read_count = 0;

        // Read and parse incrementally until we have at least one packet
        loop {
            // Try to read more data
            match self.ser.read(&mut temp_buffer) {
                Ok(n) if n > 0 => {
                    self.buffer.extend_from_slice(&temp_buffer[0..n]);
                    // println!("Read {} bytes from serial port", n);
                }
                _ => {
                    // No more data available right now
                    if !packets.is_empty() {
                        break; // Return if we have at least one packet
                    }
                    // Otherwise, if buffer has data, try parsing anyway
                    if self.buffer.is_empty() {
                        break;
                    }
                }
            }

            // Try to parse packets from current buffer
            let mut i = 0;
            while i < self.buffer.len() {
                if i + 1 < self.buffer.len() && self.buffer[i] == 0x75 && self.buffer[i + 1] == 0x65 {
                    if let Some((packet, consumed)) = parse_mip_packet(&self.buffer[i..]) {
                        if let Some(decoded) = decode_packet(&packet) {
                            packets.push(decoded);
                        }
                        i += consumed;
                        continue;
                    }
                }
                i += 1;
            }

            // Remove processed bytes
            self.buffer.drain(0..i);

            // If we have at least one packet, return it
            if !packets.is_empty() {
                break;
            }

            // Safety: prevent infinite loops
            // read_count += 1;
            // if read_count >= max_reads {
            //     println!("Warning: max reads reached without finding complete packet");
            //     break;
            // }
        }

        packets
    }

}