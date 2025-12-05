import mscl_rs
import time

mock_parser = mscl_rs.PyMockParser("dataset.bin")
mock_parser.start()

def main():
    for i in range(50):
        packets = mock_parser.get_data_packets()
        if packets:
            print(packets)
        time.sleep(0.1)

if __name__ == "__main__":
    main()
