import pathlib
from types import TracebackType
from typing import Literal

RELEASE_BUILD: bool
"""
A boolean indicating whether the mscl_rs library was built in release mode (with optimizations
enabled). This is useful for benchmarking, as the release build is significantly faster than the
debug build.
"""

VERSION: str
"""
The version of the mscl_rs library.
"""

class IMUPacket:
    """
    A class representing a parsed IMU data packet.
    """

    packet_type: Literal["raw", "estimated"]
    """
    The type of the packet, either 'raw' or 'estimated'.
    """

    timestamp: int
    """
    The timestamp of the packet in nanoseconds. This is the system time at which the packet was
    parsed.
    """

    invalid_fields: str | None
    """
    A comma-separated string of field names that were reported as invalid by the IMU
    """

    scaled_accel: tuple[float, float, float] | None
    """
    (Raw packets only) The scaled acceleration values in g's (1g = 9.81 m/s^2). This is
    uncalibrated.
    """

    scaled_gyro: tuple[float, float, float] | None
    """
    (Raw packets only) The scaled gyroscope values in radians per second. This is uncalibrated.
    """

    delta_vel: tuple[float, float, float] | None
    """
    (Raw packets only) The delta velocity values in g-seconds. This is uncalibrated.
    """

    delta_theta: tuple[float, float, float] | None
    """
    (Raw packets only) The delta theta values in radians. This is uncalibrated.
    """

    scaled_ambient_pressure: float | None
    """
    (Raw packets only) The scaled ambient pressure in mbar. This is uncalibrated.
    """

    est_pressure_alt: float | None
    """
    (Estimated packets only) The estimated pressure altitude in meters.
    """

    est_orient_quaternion: tuple[float, float, float, float] | None
    """
    (Estimated packets only) The estimated orientation as a quaternion (w, x, y, z).
    """

    est_attitude_uncert_quaternion: tuple[float, float, float, float] | None
    """
    (Estimated packets only) The estimated attitude uncertainty as a quaternion (w, x, y
    , z).
    """

    est_angular_rate: tuple[float, float, float] | None
    """
    (Estimated packets only) The estimated angular rate in radians per second.
    """

    est_compensated_accel: tuple[float, float, float] | None
    """
    (Estimated packets only) The estimated compensated acceleration in m/s^2, including
    gravity.
    """

    est_linear_accel: tuple[float, float, float] | None
    """
    (Estimated packets only) The estimated linear acceleration in m/s^2, excluding
    gravity.
    """

    est_gravity_vector: tuple[float, float, float] | None
    """
    (Estimated packets only) The estimated gravity vector in m/s^2.
    """

class SerialParser:
    """
    A class for parsing MSCL packets from a serial port.

    :param port: The serial port to connect to.
    :param baudrate: The baud rate for the serial connection.
    :param timeout: The read timeout for the serial connection in seconds. Defaults to 0.1 seconds.

    :raises OSError: If the serial port cannot be opened.
    """

    def __init__(
        self,
        port: pathlib.Path | str,
        baudrate: int = 115200,
        timeout: float | None = 0.1,
    ) -> None: ...
    def start(self) -> None: ...
    """
    Start the parser thread to read data from the serial port.
    """

    def stop(self) -> None: ...
    """
    Stop the parser thread and close the serial port.
    """

    def get_data_packets(self, block: bool = False) -> list[IMUPacket]: ...
    """
    Retrieve all available IMU data packets parsed from the serial port.
    :param block: If True, block until the timeout specified in the constructor. If False, return
        instantly if nothing is found.

    :return: A list of IMUPacket instances.

    :raises OSError: If there is an error reading from the serial port. Can also be raised when
        `block=True` and the timeout is reached without receiving any data.
    """

    def is_running(self) -> bool: ...
    """
    Check if the parser thread is currently running.

    :return: True if the parser is running, False otherwise.
    """

    def __enter__(self) -> SerialParser: ...  # noqa: PYI034
    """
    Context manager entry. Opens the serial port and starts the parser thread.
    """

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None: ...
    """
    Context manager exit. Stops the parser thread and closes the serial port.
    """

class MockParser:
    """
    A class for parsing MSCL packets from a mock dataset file. This is useful for testing
    without a physical IMU device.

    :param path: The path to the mock dataset file. This will be a .bin file (straight binary dump
        from the IMU).
    :param timeout: The read timeout for the mock dataset file in seconds. Defaults to 0.1 seconds.
    """

    def __init__(self, path: pathlib.Path | str, timeout: float | None = 0.1) -> None: ...
    def start(self) -> None: ...
    """
    Start the parser thread to read data from the mock dataset file.
    """

    def stop(self) -> None: ...
    """
    Stop the parser thread and close the mock dataset file.
    """

    def get_data_packets(self, block: bool = False) -> list[IMUPacket]: ...
    """
    Retrieve all available IMU data packets parsed from the mock dataset file.
    :param block: If True, block until the timeout specified in the constructor. If False, return
        instantly if nothing is found.

    :return: A list of IMUPacket instances.

    :raises OSError: If there is an error reading from the serial port. Can also be raised when
        `block=True` and the timeout is reached without receiving any data.
    """

    def is_running(self) -> bool: ...
    """
    Check if the parser thread is currently running. Guaranteed to return False once all data has
    been read.

    :return: True if the parser is running, False otherwise.
    """

    def __enter__(self) -> MockParser: ...  # noqa: PYI034
    """
    Context manager entry. Opens the mock dataset file and starts the parser thread.
    """

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None: ...
    """
    Context manager exit. Stops the parser thread and closes the mock dataset file.
    """
