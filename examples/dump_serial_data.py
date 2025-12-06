import serial
import time
import argparse


def dump_serial(port, baudrate, filename, duration):
    print(f"Opening {port} at {baudrate}...")
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        print(
            "Make sure the device is connected and you have permissions (e.g., sudo)."
        )
        return

    print(f"Recording to {filename} for {duration} seconds...")

    bytes_written = 0
    try:
        with open(filename, "wb") as f:
            start_time = time.time()

            while time.time() - start_time < duration:
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    f.write(data)
                    f.flush()
                    bytes_written += len(data)
                    print(f"\rCaptured {bytes_written} bytes...", end="", flush=True)
                time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nRecording stopped by user.")
    except Exception as e:
        print(f"\nError recording data: {e}")
    finally:
        if "ser" in locals() and ser.is_open:
            ser.close()

    print(f"\nDone! Saved {bytes_written} bytes to {filename}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Dump raw serial data to a binary file."
    )
    parser.add_argument(
        "--port", default="/dev/ttyACM0", help="Serial port (default: /dev/ttyACM0)"
    )
    parser.add_argument(
        "--baud", type=int, default=115200, help="Baud rate (default: 115200)"
    )
    parser.add_argument(
        "--file",
        default="new_dataset.bin",
        help="Output filename (default: new_dataset.bin)",
    )
    parser.add_argument(
        "--time", type=float, default=10.0, help="Duration in seconds (default: 10.0)"
    )

    args = parser.parse_args()
    dump_serial(args.port, args.baud, args.file, args.time)
