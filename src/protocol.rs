use crate::structs::{EstimatedDataPacket, MsclPacket, RawDataPacket};
use std::time::{SystemTime, UNIX_EPOCH};

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

fn read_u16(d: &[u8]) -> u16 {
    u16::from_be_bytes(d.try_into().unwrap())
}

/// Decodes a raw packet payload into an ImuPacket.
pub fn decode_packet(desc_set: u8, payload: &[u8]) -> Option<MsclPacket> {
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

fn decode_raw_packet(payload: &[u8], timestamp: u128) -> Option<MsclPacket> {
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
            _ => {}
        }
        i += len;
    }

    Some(MsclPacket::Raw(pkt))
}

fn decode_estimated_packet(payload: &[u8], timestamp: u128) -> Option<MsclPacket> {
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
                        read_f32(&data[12..16]),
                    ]);
                }
            }
            0x12 => {
                if check_est_field(data, 18, "estAttitudeUncertQuaternion", &mut invalid) {
                    pkt.est_attitude_uncert_quaternion = Some([
                        read_f32(&data[0..4]),
                        read_f32(&data[4..8]),
                        read_f32(&data[8..12]),
                        read_f32(&data[12..16]),
                    ]);
                }
            }
            0x0E => {
                if check_est_field(data, 14, "estAngularRate", &mut invalid) {
                    pkt.est_angular_rate = Some([
                        read_f32(&data[0..4]),
                        read_f32(&data[4..8]),
                        read_f32(&data[8..12]),
                    ]);
                }
            }
            0x1C => {
                if check_est_field(data, 14, "estCompensatedAccel", &mut invalid) {
                    pkt.est_compensated_accel = Some([
                        read_f32(&data[0..4]),
                        read_f32(&data[4..8]),
                        read_f32(&data[8..12]),
                    ]);
                }
            }
            0x0D => {
                if check_est_field(data, 14, "estLinearAccel", &mut invalid) {
                    pkt.est_linear_accel = Some([
                        read_f32(&data[0..4]),
                        read_f32(&data[4..8]),
                        read_f32(&data[8..12]),
                    ]);
                }
            }
            0x13 => {
                if check_est_field(data, 14, "estGravityVector", &mut invalid) {
                    pkt.est_gravity_vector = Some([
                        read_f32(&data[0..4]),
                        read_f32(&data[4..8]),
                        read_f32(&data[8..12]),
                    ]);
                }
            }
            _ => {}
        }
        i += len;
    }

    if !invalid.is_empty() {
        pkt.invalid_fields = Some(invalid.join(","));
    }
    Some(MsclPacket::Estimated(pkt))
}

fn check_est_field(
    data: &[u8],
    expected_len: usize,
    name: &str,
    invalid: &mut Vec<String>,
) -> bool {
    if data.len() != expected_len {
        return false;
    }
    let flags = read_u16(&data[expected_len - 2..]);
    if flags & 1 == 0 {
        invalid.push(name.to_string());
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::structs::MsclPacket;

    #[test]
    fn test_fletcher_checksum() {
        let data = b"hello";
        let (a, b) = fletcher_checksum(data);
        // 'h' = 104, 'e' = 101, 'l' = 108, 'l' = 108, 'o' = 111
        // a0 = 0, b0 = 0
        // a1 = 104, b1 = 104
        // a2 = 205, b2 = 53 (309 % 256)
        // a3 = 57 (313 % 256), b3 = 110 (362 % 256)
        // a4 = 165 (165 % 256), b4 = 19 (275 % 256)
        // a5 = 20 (276 % 256), b5 = 39 (295 % 256)
        assert_eq!(a, 20);
        assert_eq!(b, 39);
    }

    #[test]
    fn test_decode_raw_packet() {
        // Construct a fake raw packet (0x80)
        // Payload: [len, desc, data...]
        // 0x04 (Accel): 12 bytes
        // 0x12 (Timestamp): 12 bytes (8 bytes double + 4 bytes padding/flags?)

        let mut payload = Vec::new();

        // Accel: len=14 (12 data + 2 header), desc=0x04
        payload.push(14);
        payload.push(0x04);
        payload.extend_from_slice(&1.0f32.to_be_bytes()); // X
        payload.extend_from_slice(&2.0f32.to_be_bytes()); // Y
        payload.extend_from_slice(&3.0f32.to_be_bytes()); // Z

        // Timestamp: len=14 (12 data + 2 header), desc=0x12
        payload.push(14);
        payload.push(0x12);
        payload.extend_from_slice(&123.456f64.to_be_bytes());
        payload.extend_from_slice(&[0, 0, 0, 0]); // Padding to reach 12 bytes

        let pkt = decode_packet(0x80, &payload).unwrap();

        if let MsclPacket::Raw(r) = pkt {
            assert_eq!(r.scaled_accel, Some([1.0, 2.0, 3.0]));
            // Timestamp is now system time, so just check it's non-zero
            assert!(r.timestamp > 0);
        } else {
            panic!("Expected Raw packet");
        }
    }

    #[test]
    fn test_decode_estimated_packet() {
        // Construct a fake estimated packet (0x82)
        let mut payload = Vec::new();

        // Timestamp: len=14 (12 data + 2 header), desc=0x11
        payload.push(14);
        payload.push(0x11);
        payload.extend_from_slice(&100.0f64.to_be_bytes());
        payload.extend_from_slice(&[0, 0, 0, 0]);

        // Pressure Alt: len=8 (6 data + 2 header), desc=0x21
        // Data: 4 bytes float + 2 bytes flags
        payload.push(8);
        payload.push(0x21);
        payload.extend_from_slice(&500.0f32.to_be_bytes());
        // Flags: 0x0001 (valid)
        payload.extend_from_slice(&1u16.to_be_bytes());

        let pkt = decode_packet(0x82, &payload).unwrap();

        if let MsclPacket::Estimated(e) = pkt {
            // Timestamp is now system time, so just check it's non-zero
            assert!(e.timestamp > 0);
            assert_eq!(e.est_pressure_alt, Some(500.0));
        } else {
            panic!("Expected Estimated packet");
        }
    }
}
