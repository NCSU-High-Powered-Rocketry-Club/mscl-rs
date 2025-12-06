import serial
import struct
import time
from msgspec import Struct


class IMUDataPacket(Struct, array_like=True, tag=True):
    timestamp: int
    invalid_fields: str | None = None


class RawDataPacket(IMUDataPacket):
    scaledAccelX: float | None = None
    scaledAccelY: float | None = None
    scaledAccelZ: float | None = None
    scaledGyroX: float | None = None
    scaledGyroY: float | None = None
    scaledGyroZ: float | None = None
    deltaVelX: float | None = None
    deltaVelY: float | None = None
    deltaVelZ: float | None = None
    deltaThetaX: float | None = None
    deltaThetaY: float | None = None
    deltaThetaZ: float | None = None
    scaledAmbientPressure: float | None = None


class EstimatedDataPacket(IMUDataPacket):
    estPressureAlt: float | None = None
    estOrientQuaternionW: float | None = None
    estOrientQuaternionX: float | None = None
    estOrientQuaternionY: float | None = None
    estOrientQuaternionZ: float | None = None
    estAttitudeUncertQuaternionW: float | None = None
    estAttitudeUncertQuaternionX: float | None = None
    estAttitudeUncertQuaternionY: float | None = None
    estAttitudeUncertQuaternionZ: float | None = None
    estAngularRateX: float | None = None
    estAngularRateY: float | None = None
    estAngularRateZ: float | None = None
    estCompensatedAccelX: float | None = None
    estCompensatedAccelY: float | None = None
    estCompensatedAccelZ: float | None = None
    estLinearAccelX: float | None = None
    estLinearAccelY: float | None = None
    estLinearAccelZ: float | None = None
    estGravityVectorX: float | None = None
    estGravityVectorY: float | None = None
    estGravityVectorZ: float | None = None


RAW_MAP = {
    0x04: (">fff", ["scaledAccelX", "scaledAccelY", "scaledAccelZ"]),
    0x05: (">fff", ["scaledGyroX", "scaledGyroY", "scaledGyroZ"]),
    0x07: (">fff", ["deltaThetaX", "deltaThetaY", "deltaThetaZ"]),
    0x08: (">fff", ["deltaVelX", "deltaVelY", "deltaVelZ"]),
    0x17: (">f", ["scaledAmbientPressure"]),
}

EST_MAP = {
    0x21: (">fH", ["estPressureAlt"], "estPressureAlt"),
    0x03: (
        ">ffffH",
        [
            "estOrientQuaternionW",
            "estOrientQuaternionX",
            "estOrientQuaternionY",
            "estOrientQuaternionZ",
        ],
        "estOrientQuaternion",
    ),
    0x12: (
        ">ffffH",
        [
            "estAttitudeUncertQuaternionW",
            "estAttitudeUncertQuaternionX",
            "estAttitudeUncertQuaternionY",
            "estAttitudeUncertQuaternionZ",
        ],
        "estAttitudeUncertQuaternion",
    ),
    0x0E: (
        ">fffH",
        ["estAngularRateX", "estAngularRateY", "estAngularRateZ"],
        "estAngularRate",
    ),
    0x1C: (
        ">fffH",
        ["estCompensatedAccelX", "estCompensatedAccelY", "estCompensatedAccelZ"],
        "estCompensatedAccel",
    ),
    0x0D: (
        ">fffH",
        ["estLinearAccelX", "estLinearAccelY", "estLinearAccelZ"],
        "estLinearAccel",
    ),
    0x13: (
        ">fffH",
        ["estGravityVectorX", "estGravityVectorY", "estGravityVectorZ"],
        "estGravityVector",
    ),
}


def fletcher_checksum(data):
    a = b = 0
    for x in data:
        a = (a + x) % 256
        b = (b + a) % 256
    return a, b


def parse_mip_packet(buf):
    if fletcher_checksum(buf[:-2]) != (buf[-2], buf[-1]):
        return None

    fields = []
    payload = buf[4:-2]
    i, n = 0, len(payload)
    while i < n - 1:
        flen = payload[i]
        if flen < 2 or i + flen > n:
            break
        fields.append((payload[i + 1], payload[i + 2 : i + flen]))
        i += flen

    return {"desc": buf[2], "fields": fields}


def decode_packet(pkt_dict):
    desc = pkt_dict["desc"]
    fields = pkt_dict["fields"]
    ts = time.time_ns()
    invalid = []

    if desc == 0x80:
        pkt = RawDataPacket(timestamp=0)
        for d, data in fields:
            if d in RAW_MAP:
                fmt, names = RAW_MAP[d]
                if len(data) == struct.calcsize(fmt):
                    for n, v in zip(names, struct.unpack(fmt, data)):
                        setattr(pkt, n, v)
            elif d == 0x12 and len(data) == 12:
                ts = int(struct.unpack(">dHH", data)[0] * 1e9)

    elif desc == 0x82:
        pkt = EstimatedDataPacket(timestamp=0)
        for d, data in fields:
            if d in EST_MAP:
                fmt, names, err = EST_MAP[d]
                if len(data) == struct.calcsize(fmt):
                    vals = struct.unpack(fmt, data)
                    if vals[-1] & 1:
                        for n, v in zip(names, vals[:-1]):
                            setattr(pkt, n, v)
                    else:
                        invalid.append(err)
            elif d == 0x11 and len(data) == 12:
                tow, _, flags = struct.unpack(">dHH", data)
                if flags & 1:
                    ts = int(tow * 1e9)
                else:
                    invalid.append("timestamp")
    else:
        return None

    pkt.timestamp = ts
    if invalid:
        pkt.invalid_fields = ",".join(invalid)
    return pkt


def serial_parser(port, baudrate=115200, timeout=1):
    ser = serial.Serial(port, baudrate, timeout=timeout)
    buf = bytearray()

    while True:
        data = ser.read(ser.in_waiting or 1024)
        if data:
            buf.extend(data)

        while True:
            idx = buf.find(b"\x75\x65")
            if idx == -1:
                if buf and buf[-1] == 0x75:
                    buf = buf[-1:]
                else:
                    buf.clear()
                break

            if idx + 4 > len(buf):
                break
            total_len = 4 + buf[idx + 3] + 2
            if idx + total_len > len(buf):
                break

            packet = parse_mip_packet(buf[idx : idx + total_len])
            if packet:
                t0 = time.perf_counter_ns()
                decoded = decode_packet(packet)
                t1 = time.perf_counter_ns()
                if decoded:
                    yield decoded, t1 - t0
                buf = buf[idx + total_len :]
            else:
                buf = buf[idx + 2 :]


# Example usage:
last_raw_ts = None
last_est_ts = None
raw_dt_ms = 0.0
est_dt_ms = 0.0

# Use packet timestamp for profiling intervals
for pkt, parse_ns in serial_parser("/dev/ttyACM0", baudrate=115200):
    parse_ms = parse_ns / 1e6
    if isinstance(pkt, RawDataPacket):
        if last_raw_ts is not None:
            raw_dt_ms = (pkt.timestamp - last_raw_ts) / 1e6
        last_raw_ts = pkt.timestamp
        print(f"Raw interval: {raw_dt_ms:.3f} ms | Parse: {parse_ms:.3f} ms")

    elif isinstance(pkt, EstimatedDataPacket):
        if last_est_ts is not None:
            est_dt_ms = (pkt.timestamp - last_est_ts) / 1e6
        last_est_ts = pkt.timestamp
        print(f"Estimated interval: {est_dt_ms:.3f} ms | Parse: {parse_ms:.3f} ms")

    if pkt.invalid_fields:
        print(f"Invalid fields: {pkt.invalid_fields}")
