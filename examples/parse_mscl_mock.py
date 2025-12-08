import mscl_rs
from pathlib import Path

mock_parser = mscl_rs.MockParser(Path("datasets/500hz_10secs.bin"))


def main():
    packets_parsed = 0
    with mock_parser:
        while mock_parser.is_running():
            packets = mock_parser.get_data_packets()
            packets_parsed += len(packets)
            if packets:
                print(f"Packets received: {len(packets)}")
                print(f"First packet timestamp: {packets[0].timestamp} ns")
                print(f"First packet: {packets[0]}")
                print(f"Total packets parsed: {packets_parsed}")
            # time.sleep(0.1)


if __name__ == "__main__":
    main()
