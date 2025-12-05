use std::time::{SystemTime, UNIX_EPOCH};
use crate::structs::{EstimatedDataPacket, ImuPacket, RawDataPacket};

/// Calculates the Fletcher Checksum for the given data.
/// Returns a tuple (checksum_a, checksum_b).
pub fn fletcher_checksum(data: &[u8]) -> (u8, u8) {
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

/// Decodes a raw packet payload into an ImuPacket.
pub fn decode_packet(desc_set: u8, payload: &[u8]) -> Option<ImuPacket> {
    let timestamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    
    match desc_set {
        0x80 => decode_raw_packet(payload, timestamp),
        0x82 => decode_estimated_packet(payload, timestamp),
        _ => None,
    }
}

fn decode_raw_packet(payload: &[u8], timestamp: u128) -> Option<ImuPacket> {
    let mut pkt = RawDataPacket {
        timestamp,
        ..Default::default()
    };
    
    let mut i = 0;

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

    Some(ImuPacket::Raw(pkt))
}

fn decode_estimated_packet(payload: &[u8], timestamp: u128) -> Option<ImuPacket> {
    let mut pkt = EstimatedDataPacket {
        timestamp,
        ..Default::default()
    };
    
    let mut invalid: Vec<String> = Vec::new();
    let mut i = 0;

    while i < payload.len() {
        let len = payload[i] as usize;
        if len < 2 || i + len > payload.len() {
            break;
        }
        let desc = payload[i + 1];
        let data = &payload[i + 2..i + len];

        match desc {
            0x21 => {
                if check_est_field(data, 6, "estPressureAlt", &mut invalid) {
                    pkt.est_pressure_alt = Some(read_f32(&data[0..4]));
                }
            }
            0x03 => {
                if check_est_field(data, 18, "estOrientQuaternion", &mut invalid) {
                    pkt.est_orient_quaternion = Some([
                        read_f32(&data[0..4]),
                        read_f32(&data[4..8]),
                        read_f32(&data[8..12]),
                        read_f32(&data[12..16])
                    ]);
                }
            }
            0x12 => {
                if check_est_field(data, 18, "estAttitudeUncertQuaternion", &mut invalid) {
                    pkt.est_attitude_uncert_quaternion = Some([
                        read_f32(&data[0..4]),
                        read_f32(&data[4..8]),
                        read_f32(&data[8..12]),
                        read_f32(&data[12..16])
                    ]);
                }
            }
            0x0E => {
                if check_est_field(data, 14, "estAngularRate", &mut invalid) {
                    pkt.est_angular_rate = Some([
                        read_f32(&data[0..4]),
                        read_f32(&data[4..8]),
                        read_f32(&data[8..12])
                    ]);
                }
            }
            0x1C => {
                if check_est_field(data, 14, "estCompensatedAccel", &mut invalid) {
                    pkt.est_compensated_accel = Some([
                        read_f32(&data[0..4]),
                        read_f32(&data[4..8]),
                        read_f32(&data[8..12])
                    ]);
                }
            }
            0x0D => {
                if check_est_field(data, 14, "estLinearAccel", &mut invalid) {
                    pkt.est_linear_accel = Some([
                        read_f32(&data[0..4]),
                        read_f32(&data[4..8]),
                        read_f32(&data[8..12])
                    ]);
                }
            }
            0x13 => {
                if check_est_field(data, 14, "estGravityVector", &mut invalid) {
                    pkt.est_gravity_vector = Some([
                        read_f32(&data[0..4]),
                        read_f32(&data[4..8]),
                        read_f32(&data[8..12])
                    ]);
                }
            }
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

fn check_est_field(data: &[u8], expected_len: usize, name: &str, invalid: &mut Vec<String>) -> bool {
    if data.len() != expected_len {
        return false;
    }
    let flags = read_u16(&data[expected_len - 2..]);
    if flags & 1 == 0 {
        invalid.push(name.to_string());
    }
    return true;
}
