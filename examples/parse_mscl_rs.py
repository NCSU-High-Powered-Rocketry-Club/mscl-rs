"""Example of parsing data packets from the IMU using mscl_rs and msgspec."""

import mscl_rs
import msgspec

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
    scaledAccelX: float | None = None
    scaledAccelY: float | None = None
    scaledAccelZ: float | None = None  # this will be ~-1.0g when the IMU is at rest
    scaledGyroX: float | None = None
    scaledGyroY: float | None = None
    scaledGyroZ: float | None = None
    # deltaVel units are in g seconds
    deltaVelX: float | None = None
    deltaVelY: float | None = None
    deltaVelZ: float | None = None
    # in radians
    deltaThetaX: float | None = None
    deltaThetaY: float | None = None
    deltaThetaZ: float | None = None
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
    # estCompensatedAccel units are in m/s^2, including gravity
    estCompensatedAccelX: float | None = None
    estCompensatedAccelY: float | None = None
    estCompensatedAccelZ: float | None = None  # this will be ~-9.81 m/s^2 when the IMU is at rest
    # estLinearAccel units are in m/s^2, excluding gravity
    estLinearAccelX: float | None = None
    estLinearAccelY: float | None = None
    estLinearAccelZ: float | None = None  # this will be ~0 m/s^2 when the IMU is at rest
    # estGravityVector units are in m/s^2
    estGravityVectorX: float | None = None
    estGravityVectorY: float | None = None
    estGravityVectorZ: float | None = None

parser = mscl_rs.PySerialParser('/dev/ttyACM0', 115200, timeout=0)  # Small timeout for hot loop

# In hot loop:

def main():
    while True:
        packets = parser.get_data_packets()
        for pkt in packets:
            packet_type, ts, invalid, *data = pkt
            if packet_type == 'raw':
                # print(*data)
                raw = RawDataPacket(ts, invalid, *data)
                # print(raw)
                # Process...
            elif packet_type == 'estimated':
                est = EstimatedDataPacket(ts, invalid, *data)
                print(est)
                # pressure_alt, orient_quat, attitude_uncert_quat, angular_rate, compensated_accel, linear_accel, gravity_vector = data
                # print(f"Orientation: {orient_quat}, Angular Rate: {angular_rate}, Compensated Accel: {compensated_accel}")
                # Process...
            # Send to queue

if __name__ == "__main__":
    main()