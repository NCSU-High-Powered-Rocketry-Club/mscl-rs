# mscl-rs

Rust parser for data from the 3DM®-CX5-15 IMU. This library provides
python bindings in a convenient format so that the client side (Python) does not need to spend 
precious processing time parsing the data.

This library has been benchmarked to be ~10x faster than the official Python [python-mscl](https://github.com/harshil21/python-mscl) equivalent (which has a C++ backend).

This library is not intended to be a full replacement for the C++ [MSCL](https://github.com/LORD-MicroStrain/MSCL) library
but rather a lightweight parser for the data packets, and intended to be more well maintained and Python free
threading compatible.

Features will be added as needed, open an issue or PR if you need something!

## Installation

This package is available on [PyPI](https://pypi.org/project/mscl-rs/), you can install it via pip:

```
pip install mscl-rs
```

Wheels are available for Linux (x86_64 and aarch64) 3.13, 3.14, and 3.14t. Any other platform or
python version will require building from source, which your package manager (e.g. `pip`) will
automatically handle, but you still need to install [Rust](https://www.rust-lang.org/tools/install).

## Public API

The main entrypoint is the `SerialParser` class, which can be used to read data from the IMU
over a serial connection. E.g.

```python
from mscl_rs import SerialParser

parser = SerialParser(port="/dev/ttyUSB0", timeout=1.0)  # Timeout of 1 second

with parser:  # calls parser.start() and parser.stop() automatically
    while True:
        packets = parser.get_data_packets(block=True)  # Block until data is available
        for packet in packets:
            print(packet)
```

To see all available methods, documentation, and the data packet structure, see the
[`mscl_rs.pyi`](./mscl_rs.pyi) file.

## Local Development Setup:

You will need [uv](https://docs.astral.sh/uv/getting-started/installation/), and [Rust](https://www.rust-lang.org/tools/install) installed.

Then clone the repository:

```bash
git clone https://github.com/NCSU-High-Powered-Rocketry-Club/mscl_rs.git && cd mscl-rs && uv run pre-commit install
```

If you have plugged in your IMU via USB, you can test the example parser script by the command
below. Make sure to change the serial port in `examples/parse_mscl_rs.py` if needed.

```bash
uv run examples/parse_mscl_rs.py
```

This will automatically build the Rust code and install the package in a virtual environment, and 
automatically run the example script.

You do not need to separately run `maturin develop` at all.

To switch between development and release builds, you can change the `config-settings` in
`pyproject.toml`.

### Testing

This library has tests both in the Rust and Python layers. To run the tests, run the following
commands:

```bash
# Run Rust tests
cargo test
# Run Python tests
uv run pytest tests/
```

If you have an actual IMU connected via serial, this will also run tests that parse real data from
the IMU, and benchmark the performance against the official `python-mscl` library.


### Cross compiling and releasing

To build wheels for all platforms, you can use `maturin`'s and Zig's cross compiling support. If
you are on linux, you can run the `compile.sh` script to build wheels for the platforms and
python versions we provide first class support for.

```bash
chmod +x compile.sh
./compile.sh
```

This will create wheels in the `target/wheels/` folder.

We can now publish to PyPI using `uv`:

```bash
uv publish target/wheels/*
```

## Changelog

See the GitHub releases page for the changelog.

## License

This project is licensed under the MIT License. See the LICENSE file for details.


