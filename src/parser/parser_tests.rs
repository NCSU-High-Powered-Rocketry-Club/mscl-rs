use super::*;
use crate::structs::ImuPacket;

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

    if let ImuPacket::Raw(r) = pkt {
        assert_eq!(r.scaled_accel, Some([1.0, 2.0, 3.0]));
        // 123.456 * 1e9 = 123456000000
        assert_eq!(r.timestamp, 123456000000);
    } else {
        panic!("Expected Raw packet");
    }
}
