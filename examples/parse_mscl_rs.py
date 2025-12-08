"""Example of parsing data packets from the IMU using mscl_rs and msgspec."""

import mscl_rs as mscl_parser
import msgspec
import time


class IMUDataPacket(msgspec.Struct, array_like=True, tag=True):
    """
    Base class representing a collection of data packets from the IMU.

    The attributes should be named the same as they are when sent from the IMU -- this just means
    they're going to be in camelCase.
    """

    timestamp: int  # in nanoseconds
    # list of fields which may be invalid as reported by the IMU
    invalid_fields: str | None = None


class RawDataPacket(IMUDataPacket):
    """
    Represents a raw data packet from the IMU.

    These values are exactly what the IMU read, without any processing. It contains a timestamp and
    the raw values of the acceleration, gyroscope, delta velocity, delta theta, and ambient
    pressure.
    """

    # scaledAccel units are in "g" (9.81 m/s^2)
    scaledAccel: tuple[float, float, float] | None = None
    scaledGyro: tuple[float, float, float] | None = None
    # deltaVel units are in g seconds
    deltaVel: tuple[float, float, float] | None = None
    # in radians
    deltaTheta: tuple[float, float, float] | None = None
    # pressure in mbar
    scaledAmbientPressure: float | None = None


class EstimatedDataPacket(IMUDataPacket):
    """
    Represents an estimated data packet from the IMU.

    These values are the processed values of the raw data that the IMU internally smoothes and makes
    more accurate before sending the packet. It contains a timestamp and the estimated values of the
    relevant data points.
    """

    estPressureAlt: float | None = None
    estOrientQuaternion: tuple[float, float, float, float] | None = None
    estAttitudeUncertQuaternion: tuple[float, float, float, float] | None = None
    estAngularRate: tuple[float, float, float] | None = None
    # estCompensatedAccel units are in m/s^2, including gravity
    estCompensatedAccel: tuple[float, float, float] | None = None
    # estLinearAccel units are in m/s^2, excluding gravity
    estLinearAccel: tuple[float, float, float] | None = None
    # estGravityVector units are in m/s^2
    estGravityVector: tuple[float, float, float] | None = None


parser = mscl_parser.SerialParser(port="/dev/ttyACM0", timeout=1.0)


def main():
    parser.start()
    last_raw_ts = None
    last_est_ts = None

    while True:
        # for _ in range(500000):
        t0 = time.perf_counter_ns()
        packets = parser.get_data_packets(block=True)
        t_rust = time.perf_counter_ns()
        if not packets:
            print("No packets received")
            continue

        # time.sleep(1)  # Slight delay to make output readable
        print(f"Packets received: {len(packets)}")
        print(f"Rust parse time: {(t_rust - t0) / 1e6:.6f} ms")

        # Average Rust time per packet in this batch
        avg_rust_ns = (t_rust - t0) / len(packets)

        for pkt in packets:
            t_start_struct = time.perf_counter_ns()
            # pkt is now an instance of mscl_rs.IMUPacket
            packet_type = pkt.packet_type
            ts = pkt.timestamp

            if packet_type == "raw":
                # Access raw fields directly from the Rust object
                # pkt.scaled_accel, pkt.scaled_gyro, etc.
                t_end_struct = time.perf_counter_ns()

                if last_raw_ts is not None:
                    (ts - last_raw_ts) / 1e6
                last_raw_ts = ts

                (avg_rust_ns + (t_end_struct - t_start_struct)) / 1e6
                # print(f"Raw interval: {dt_ms:.3f} ms | Parse: {parse_ms:.6f} ms")

            elif packet_type == "estimated":
                # Access estimated fields directly
                t_end_struct = time.perf_counter_ns()

                if last_est_ts is not None:
                    (ts - last_est_ts) / 1e6
                last_est_ts = ts

                (avg_rust_ns + (t_end_struct - t_start_struct)) / 1e6
                # print(f"Estimated interval: {dt_ms:.3f} ms | Parse: {parse_ms:.6f} ms")
                # print(f"Alt: {pkt.est_pressure_alt:.3f} m")
                # print(f"Orient (quat): {pkt.est_orient_quaternion}")
                # print(f"Angular Rate: {pkt.est_angular_rate}")
                # print(f"Invalid Fields: {pkt.invalid_fields}")


if __name__ == "__main__":
    main()
    print("Stopping parser...")
    parser.stop()

    print("Staring parser again...")
    parser.start()
    main()
    print("Stopping parser...")
