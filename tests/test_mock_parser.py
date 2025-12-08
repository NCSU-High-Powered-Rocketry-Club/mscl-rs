import pytest

from mscl_rs import MockParser


class TestMockParser:
    """Tests for the MockParser class."""

    def test_initialization(self):
        parser = MockParser("datasets/500hz_10secs.bin")
        assert parser is not None

    def test_no_packets_before_start(self):
        parser = MockParser("datasets/500hz_10secs.bin")
        with pytest.raises(OSError, match="timed out waiting on channel"):
            parser.get_data_packets(block=True)

    def test_parser_stop(self):
        parser = MockParser("datasets/500hz_10secs.bin")
        parser.start()
        assert parser.is_running(), "Parser did not start correctly"
        parser.stop()
        assert not parser.is_running(), "Parser did not stop correctly"

    def test_start_and_get_packets(self):
        parser = MockParser("datasets/500hz_10secs.bin")
        parser.start()

        packets = parser.get_data_packets(block=True)
        assert isinstance(packets, list)
        assert all(hasattr(pkt, "timestamp") for pkt in packets)

    def test_context_manager(self):
        parser = MockParser("datasets/500hz_10secs.bin")
        with parser:
            assert parser.is_running(), "Parser should be running inside context manager"
            packets = parser.get_data_packets(block=True)
            assert isinstance(packets, list)
        assert not parser.is_running(), "Parser should not be running outside context manager"

    def test_packet_content(self):
        parser = MockParser("datasets/500hz_10secs.bin")
        parser.start()
        # time.sleep(0.001) # Allow some time for packets to be read

        packets = parser.get_data_packets(block=True)
        assert len(packets) > 0, "No packets received for content test"

        for packet in packets:
            assert hasattr(packet, "packet_type")
            assert hasattr(packet, "timestamp")
            assert hasattr(packet, "invalid_fields")
            assert isinstance(packet.packet_type, str)
            assert packet.packet_type in ["raw", "estimated"]

            if packet.packet_type == "raw":
                # Check for raw packet specific fields
                assert hasattr(packet, "scaled_accel")
                assert hasattr(packet, "scaled_gyro")
                assert hasattr(packet, "delta_vel")
                assert hasattr(packet, "delta_theta")
                assert hasattr(packet, "scaled_ambient_pressure")

                # Validate types
                assert isinstance(packet.scaled_accel, list)
                assert isinstance(packet.scaled_gyro, list)
                assert isinstance(packet.delta_vel, list)
                assert isinstance(packet.delta_theta, list)
                assert isinstance(packet.scaled_ambient_pressure, float)

                # Validate lengths
                assert len(packet.scaled_accel) == 3
                assert len(packet.scaled_gyro) == 3
                assert len(packet.delta_vel) == 3
                assert len(packet.delta_theta) == 3

                # Validate element types
                assert all(isinstance(x, float) for x in packet.scaled_accel)
                assert all(isinstance(x, float) for x in packet.scaled_gyro)
                assert all(isinstance(x, float) for x in packet.delta_vel)
                assert all(isinstance(x, float) for x in packet.delta_theta)

            elif packet.packet_type == "estimated":
                assert hasattr(packet, "est_pressure_alt")
                assert hasattr(packet, "est_orient_quaternion")
                assert hasattr(packet, "est_attitude_uncert_quaternion")
                assert hasattr(packet, "est_angular_rate")
                assert hasattr(packet, "est_compensated_accel")
                assert hasattr(packet, "est_linear_accel")
                assert hasattr(packet, "est_gravity_vector")

                # Validate types
                assert isinstance(packet.est_pressure_alt, float)
                assert isinstance(packet.est_orient_quaternion, list)
                assert isinstance(packet.est_attitude_uncert_quaternion, list)
                assert isinstance(packet.est_angular_rate, list)
                assert isinstance(packet.est_compensated_accel, list)
                assert isinstance(packet.est_linear_accel, list)
                assert isinstance(packet.est_gravity_vector, list)
                # Validate lengths
                assert len(packet.est_orient_quaternion) == 4
                assert len(packet.est_attitude_uncert_quaternion) == 4
                assert len(packet.est_angular_rate) == 3
                assert len(packet.est_compensated_accel) == 3
                assert len(packet.est_linear_accel) == 3
                assert len(packet.est_gravity_vector) == 3
                # Validate element types
                assert all(isinstance(x, float) for x in packet.est_orient_quaternion)
                assert all(isinstance(x, float) for x in packet.est_attitude_uncert_quaternion)
                assert all(isinstance(x, float) for x in packet.est_angular_rate)
                assert all(isinstance(x, float) for x in packet.est_compensated_accel)
                assert all(isinstance(x, float) for x in packet.est_linear_accel)
                assert all(isinstance(x, float) for x in packet.est_gravity_vector)
