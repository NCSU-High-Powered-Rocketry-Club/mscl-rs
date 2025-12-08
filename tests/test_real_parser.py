"""Benchmarks using the mscl library v/s the mscl_rs library."""

import platform
import time

import msgspec
import pytest

import mscl_rs
from mscl_rs import SerialParser

if platform.python_version() >= "3.14":
    pytest.skip(
        "python-mscl does not support Python >= 3.14 yet. Use 3.13",
        allow_module_level=True,
    )

from python_mscl import mscl

ESTIMATED_DESCRIPTOR_SET = 130
RAW_DESCRIPTOR_SET = 128
DELTA_THETA_FIELD = 32775
DELTA_VEL_FIELD = 32776
SCALED_ACCEL_FIELD = 32772
SCALED_GYRO_FIELD = 32773
SCALED_AMBIENT_PRESSURE_FIELD = 32791
EST_ANGULAR_RATE_FIELD = 33294
EST_ATTITUDE_UNCERT_FIELD = 33298
EST_COMPENSATED_ACCEL_FIELD = 33308
EST_GRAVITY_VECTOR_FIELD = 33299
EST_LINEAR_ACCEL_FIELD = 33293
EST_ORIENT_QUATERNION_FIELD = 33283
EST_PRESSURE_ALT_FIELD = 33313
X_QUALIFIER = 1
Y_QUALIFIER = 2
Z_QUALIFIER = 3
ATTITUDE_UNCERT_QUALIFIER = 5
PRESSURE_ALT_QUALIFIER = 67
AMBIENT_PRESSURE_QUALIFIER = 58


class RawDataPacket(msgspec.Struct):
    """Packet structure for raw IMU data."""

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


class EstimatedDataPacket(msgspec.Struct):
    """Packet structure for estimated IMU data."""

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


# Skip running benchmark on >=3.14 since python-mscl does not support it yet
@pytest.mark.skipif(
    platform.python_version() >= "3.14",
    reason="python-mscl does not support Python >= 3.14 yet",
)
class TestRealParser:
    """Tests for the SerialParser class using a real IMU device."""

    port = "/dev/ttyACM0"  # The port where the IMU is connected to.
    time_to_wait_between_reads = 1  # seconds
    benchmark_time_s = 5  # seconds
    frequency_hz = 1000  # IMU data rate in Hz (500 raw + 500 estimated)

    def test_real_benchmark(self):
        if not mscl_rs.RELEASE_BUILD:
            pytest.fail(
                "Benchmarking requires the release build of the mscl_rs library. Please rebuild"
                " with optimizations enabled."
            )
        try:
            connection = mscl.Connection.Serial(self.port)
        except Exception as e:
            pytest.skip(f"Could not open serial port {self.port}: {e}. Skipping benchmark.")

        node = mscl.InertialNode(connection)
        packets_parsed_mscl = 0

        total_time_mscl = 0.0
        avg_parsing_times = []
        start_bench_time = time.time()

        while time.time() - start_bench_time < self.benchmark_time_s:
            time.sleep(self.time_to_wait_between_reads)  # Allow time for packets to accumulate
            start_time = time.perf_counter()
            mscl_packets: mscl.MipDataPackets = node.getDataPackets(timeout=10)
            end_time = time.perf_counter()

            total_time_mscl = (end_time - start_time) * 1e3  # in milliseconds
            avg_parsing_times.append(total_time_mscl)

            packets_parsed_mscl += len(mscl_packets)

        avg_parsing_time_mscl = sum(avg_parsing_times) / len(avg_parsing_times)
        print(
            f"MSCL parsed {packets_parsed_mscl} packets, with parsing "
            f"{self.time_to_wait_between_reads * self.frequency_hz} "
            f"packets at a time taking {avg_parsing_time_mscl:.6f} ms."
        )
        connection.disconnect()

        parser = SerialParser(port=self.port, timeout=0.1)
        parser.start()
        packets_parsed_rs = 0
        total_time_rs = 0.0
        avg_parsing_times_rs = []
        start_bench_time = time.time()

        while time.time() - start_bench_time < self.benchmark_time_s:
            time.sleep(self.time_to_wait_between_reads)  # Allow time for packets to accumulate
            start_time = time.perf_counter()
            rs_packets = parser.get_data_packets(block=True)
            end_time = time.perf_counter()

            total_time_rs = (end_time - start_time) * 1e3  # in milliseconds
            avg_parsing_times_rs.append(total_time_rs)
            # print(
            #     f"mscl_rs Packets received: {len(rs_packets)}, Time taken: {total_time_rs:.6f} ms"
            # )

            packets_parsed_rs += len(rs_packets)
        avg_parsing_time_rs = sum(avg_parsing_times_rs) / len(avg_parsing_times_rs)
        print(
            f"mscl_rs parsed {packets_parsed_rs} packets, with parsing "
            f"{self.time_to_wait_between_reads * self.frequency_hz} "
            f"packets at a time taking {avg_parsing_time_rs:.6f} ms."
        )
        parser.stop()

        # Assert that both parsers parsed similar number of packets (within 10% tolerance)
        assert abs(packets_parsed_rs - packets_parsed_mscl) < 0.1 * packets_parsed_mscl, (
            "mscl_rs and mscl parsed different number of packets!"
        )
        assert avg_parsing_time_rs < avg_parsing_time_mscl, (
            "mscl_rs parsing time is not faster than mscl parsing time!"
        )
        # We should be ~10x faster than mscl:
        assert avg_parsing_time_rs < 0.1 * avg_parsing_time_mscl, (
            "mscl_rs parsing time is not at least 10x faster than mscl parsing time! Make sure you"
            "are running this benchmark with the release build of the mscl_rs library."
        )

    def test_expected_values(self):
        try:
            parser = SerialParser(port=self.port, timeout=1.0)
        except Exception as e:
            pytest.skip(
                f"Could not open serial port {self.port}: {e}. Skipping expected values test."
            )
        parser.start()
        time.sleep(0.01)
        packets_rs = parser.get_data_packets(block=True)
        parser.stop()
        del parser  # Ensure parser is cleaned up, freeing the port

        assert len(packets_rs) >= 3, "No packets received for content test"

        connection = mscl.Connection.Serial(self.port)
        node = mscl.InertialNode(connection)
        time.sleep(0.05)
        packets_mscl: mscl.MipDataPackets = node.getDataPackets(timeout=10)
        connection.disconnect()

        assert len(packets_mscl) >= 3, "No packets received from mscl for content test"

        # Parse packets:
        imu_data_packets_mscl = self.parse_mscl_packets(packets_mscl)

        # Compare values from both parsers
        # First get a estimated and raw packet from each parser
        raw_packet_rs = next(pkt for pkt in packets_rs if pkt.packet_type == "raw")
        est_packet_rs = next(pkt for pkt in packets_rs if pkt.packet_type == "estimated")
        raw_packet_mscl = next(
            pkt for pkt in imu_data_packets_mscl if isinstance(pkt, RawDataPacket)
        )
        est_packet_mscl = next(
            pkt for pkt in imu_data_packets_mscl if isinstance(pkt, EstimatedDataPacket)
        )
        err_tol = 1e-1

        # Compare raw packet values are similar
        assert raw_packet_rs.scaled_accel[0] == pytest.approx(
            raw_packet_mscl.scaledAccelX, abs=err_tol
        )
        assert raw_packet_rs.scaled_accel[1] == pytest.approx(
            raw_packet_mscl.scaledAccelY, abs=err_tol
        )
        assert raw_packet_rs.scaled_accel[2] == pytest.approx(
            raw_packet_mscl.scaledAccelZ, abs=err_tol
        )
        assert raw_packet_rs.scaled_gyro[0] == pytest.approx(
            raw_packet_mscl.scaledGyroX, abs=err_tol
        )
        assert raw_packet_rs.scaled_gyro[1] == pytest.approx(
            raw_packet_mscl.scaledGyroY, abs=err_tol
        )
        assert raw_packet_rs.scaled_gyro[2] == pytest.approx(
            raw_packet_mscl.scaledGyroZ, abs=err_tol
        )
        assert raw_packet_rs.delta_vel[0] == pytest.approx(raw_packet_mscl.deltaVelX, abs=err_tol)
        assert raw_packet_rs.delta_vel[1] == pytest.approx(raw_packet_mscl.deltaVelY, abs=err_tol)
        assert raw_packet_rs.delta_vel[2] == pytest.approx(raw_packet_mscl.deltaVelZ, abs=err_tol)
        assert raw_packet_rs.delta_theta[0] == pytest.approx(
            raw_packet_mscl.deltaThetaX, abs=err_tol
        )
        assert raw_packet_rs.delta_theta[1] == pytest.approx(
            raw_packet_mscl.deltaThetaY, abs=err_tol
        )
        assert raw_packet_rs.delta_theta[2] == pytest.approx(
            raw_packet_mscl.deltaThetaZ, abs=err_tol
        )
        assert raw_packet_rs.scaled_ambient_pressure == pytest.approx(
            raw_packet_mscl.scaledAmbientPressure, abs=err_tol
        )
        # Compare estimated packet values are similar
        assert est_packet_rs.est_pressure_alt == pytest.approx(
            est_packet_mscl.estPressureAlt, abs=1
        )
        assert est_packet_rs.est_orient_quaternion[0] == pytest.approx(
            est_packet_mscl.estOrientQuaternionW, abs=err_tol
        )
        assert est_packet_rs.est_orient_quaternion[1] == pytest.approx(
            est_packet_mscl.estOrientQuaternionX, abs=err_tol
        )
        assert est_packet_rs.est_orient_quaternion[2] == pytest.approx(
            est_packet_mscl.estOrientQuaternionY, abs=err_tol
        )
        assert est_packet_rs.est_orient_quaternion[3] == pytest.approx(
            est_packet_mscl.estOrientQuaternionZ, abs=err_tol
        )
        assert est_packet_rs.est_attitude_uncert_quaternion[0] == pytest.approx(
            est_packet_mscl.estAttitudeUncertQuaternionW, abs=err_tol
        )
        assert est_packet_rs.est_attitude_uncert_quaternion[1] == pytest.approx(
            est_packet_mscl.estAttitudeUncertQuaternionX, abs=err_tol
        )
        assert est_packet_rs.est_attitude_uncert_quaternion[2] == pytest.approx(
            est_packet_mscl.estAttitudeUncertQuaternionY, abs=err_tol
        )
        assert est_packet_rs.est_attitude_uncert_quaternion[3] == pytest.approx(
            est_packet_mscl.estAttitudeUncertQuaternionZ, abs=err_tol
        )
        assert est_packet_rs.est_angular_rate[0] == pytest.approx(
            est_packet_mscl.estAngularRateX, abs=err_tol
        )
        assert est_packet_rs.est_angular_rate[1] == pytest.approx(
            est_packet_mscl.estAngularRateY, abs=err_tol
        )
        assert est_packet_rs.est_angular_rate[2] == pytest.approx(
            est_packet_mscl.estAngularRateZ, abs=err_tol
        )
        assert est_packet_rs.est_compensated_accel[0] == pytest.approx(
            est_packet_mscl.estCompensatedAccelX, abs=err_tol
        )
        assert est_packet_rs.est_compensated_accel[1] == pytest.approx(
            est_packet_mscl.estCompensatedAccelY, abs=err_tol
        )
        assert est_packet_rs.est_compensated_accel[2] == pytest.approx(
            est_packet_mscl.estCompensatedAccelZ, abs=err_tol
        )
        assert est_packet_rs.est_linear_accel[0] == pytest.approx(
            est_packet_mscl.estLinearAccelX, abs=err_tol
        )
        assert est_packet_rs.est_linear_accel[1] == pytest.approx(
            est_packet_mscl.estLinearAccelY, abs=err_tol
        )
        assert est_packet_rs.est_linear_accel[2] == pytest.approx(
            est_packet_mscl.estLinearAccelZ, abs=err_tol
        )
        assert est_packet_rs.est_gravity_vector[0] == pytest.approx(
            est_packet_mscl.estGravityVectorX, abs=err_tol
        )
        assert est_packet_rs.est_gravity_vector[1] == pytest.approx(
            est_packet_mscl.estGravityVectorY, abs=err_tol
        )
        assert est_packet_rs.est_gravity_vector[2] == pytest.approx(
            est_packet_mscl.estGravityVectorZ, abs=err_tol
        )

    def parse_mscl_packets(
        self, packets_mscl: mscl.MipDataPackets
    ) -> list[EstimatedDataPacket | RawDataPacket]:
        imu_data_packets = []

        for packet in packets_mscl:
            descriptor_set = packet.descriptorSet()

            # Initialize packet with the timestamp, determines if the packet is raw or estimated
            if descriptor_set == RAW_DESCRIPTOR_SET:
                imu_data_packet = RawDataPacket()
                # Iterate through each data point in the packet.
                for data_point in packet.data():
                    # Extract the channel name of the data point.
                    qualifier = data_point.qualifier()
                    field_name = data_point.field()

                    if field_name == SCALED_ACCEL_FIELD:
                        # Scaled acceleration data
                        if qualifier == X_QUALIFIER:
                            imu_data_packet.scaledAccelX = data_point.as_float()
                        elif qualifier == Y_QUALIFIER:
                            imu_data_packet.scaledAccelY = data_point.as_float()
                        elif qualifier == Z_QUALIFIER:
                            imu_data_packet.scaledAccelZ = data_point.as_float()

                    elif field_name == SCALED_GYRO_FIELD:
                        # Scaled gyroscope data
                        if qualifier == X_QUALIFIER:
                            imu_data_packet.scaledGyroX = data_point.as_float()
                        elif qualifier == Y_QUALIFIER:
                            imu_data_packet.scaledGyroY = data_point.as_float()
                        elif qualifier == Z_QUALIFIER:
                            imu_data_packet.scaledGyroZ = data_point.as_float()

                    elif field_name == DELTA_VEL_FIELD:
                        # Delta velocity (change in velocity)
                        if qualifier == X_QUALIFIER:
                            imu_data_packet.deltaVelX = data_point.as_float()
                        elif qualifier == Y_QUALIFIER:
                            imu_data_packet.deltaVelY = data_point.as_float()
                        elif qualifier == Z_QUALIFIER:
                            imu_data_packet.deltaVelZ = data_point.as_float()

                    elif field_name == DELTA_THETA_FIELD:
                        # Delta theta (change in orientation)
                        if qualifier == X_QUALIFIER:
                            imu_data_packet.deltaThetaX = data_point.as_float()
                        elif qualifier == Y_QUALIFIER:
                            imu_data_packet.deltaThetaY = data_point.as_float()
                        elif qualifier == Z_QUALIFIER:
                            imu_data_packet.deltaThetaZ = data_point.as_float()

                    elif (
                        field_name == SCALED_AMBIENT_PRESSURE_FIELD
                        and qualifier == AMBIENT_PRESSURE_QUALIFIER
                    ):
                        # Scaled ambient pressure data
                        imu_data_packet.scaledAmbientPressure = data_point.as_float()
                imu_data_packets.append(imu_data_packet)

            elif descriptor_set == ESTIMATED_DESCRIPTOR_SET:
                imu_data_packet = EstimatedDataPacket()
                for data_point in packet.data():
                    # Extract the channel name of the data point.
                    qualifier = data_point.qualifier()
                    field_name = data_point.field()

                    if field_name == EST_PRESSURE_ALT_FIELD and qualifier == PRESSURE_ALT_QUALIFIER:
                        # Estimated pressure altitude
                        imu_data_packet.estPressureAlt = data_point.as_float()

                    elif (
                        field_name == EST_ORIENT_QUATERNION_FIELD
                        and qualifier == ATTITUDE_UNCERT_QUALIFIER
                    ):
                        # Estimated orientation quaternion
                        matrix = data_point.as_Matrix()
                        # The imu sends the quaternions as a matrix, so we have to unpack it
                        imu_data_packet.estOrientQuaternionW = matrix.as_floatAt(0, 0)
                        imu_data_packet.estOrientQuaternionX = matrix.as_floatAt(0, 1)
                        imu_data_packet.estOrientQuaternionY = matrix.as_floatAt(0, 2)
                        imu_data_packet.estOrientQuaternionZ = matrix.as_floatAt(0, 3)

                    elif (
                        field_name == EST_ATTITUDE_UNCERT_FIELD
                        and qualifier == ATTITUDE_UNCERT_QUALIFIER
                    ):
                        # Estimated attitude uncertainty quaternion
                        matrix = data_point.as_Matrix()
                        imu_data_packet.estAttitudeUncertQuaternionW = matrix.as_floatAt(0, 0)
                        imu_data_packet.estAttitudeUncertQuaternionX = matrix.as_floatAt(0, 1)
                        imu_data_packet.estAttitudeUncertQuaternionY = matrix.as_floatAt(0, 2)
                        imu_data_packet.estAttitudeUncertQuaternionZ = matrix.as_floatAt(0, 3)

                    elif field_name == EST_ANGULAR_RATE_FIELD:
                        # Estimated angular rate
                        if qualifier == X_QUALIFIER:
                            imu_data_packet.estAngularRateX = data_point.as_float()
                        elif qualifier == Y_QUALIFIER:
                            imu_data_packet.estAngularRateY = data_point.as_float()
                        elif qualifier == Z_QUALIFIER:
                            imu_data_packet.estAngularRateZ = data_point.as_float()

                    elif field_name == EST_COMPENSATED_ACCEL_FIELD:
                        # Estimated compensated acceleration
                        if qualifier == X_QUALIFIER:
                            imu_data_packet.estCompensatedAccelX = data_point.as_float()
                        elif qualifier == Y_QUALIFIER:
                            imu_data_packet.estCompensatedAccelY = data_point.as_float()
                        elif qualifier == Z_QUALIFIER:
                            imu_data_packet.estCompensatedAccelZ = data_point.as_float()

                    elif field_name == EST_LINEAR_ACCEL_FIELD:
                        # Estimated linear acceleration
                        if qualifier == X_QUALIFIER:
                            imu_data_packet.estLinearAccelX = data_point.as_float()
                        elif qualifier == Y_QUALIFIER:
                            imu_data_packet.estLinearAccelY = data_point.as_float()
                        elif qualifier == Z_QUALIFIER:
                            imu_data_packet.estLinearAccelZ = data_point.as_float()

                    elif field_name == EST_GRAVITY_VECTOR_FIELD:
                        # Estimated gravity vector
                        if qualifier == X_QUALIFIER:
                            imu_data_packet.estGravityVectorX = data_point.as_float()
                        elif qualifier == Y_QUALIFIER:
                            imu_data_packet.estGravityVectorY = data_point.as_float()
                        elif qualifier == Z_QUALIFIER:
                            imu_data_packet.estGravityVectorZ = data_point.as_float()
                imu_data_packets.append(imu_data_packet)

        return imu_data_packets
