# mscl-rs

Rust parser for data from the 3DM®-CX5-15 IMU. This library provides
python bindings in a convenient format so that the client side (Python) does not need to spend 
precious processing time parsing the data.

This library is not intended to be a full replacement for the C++ [MSCL](https://github.com/LORD-MicroStrain/MSCL) library
but rather a lightweight parser for the data packets, and intended to be more well maintained and Python free
threading compatible.

Features will be added as needed.

## Installation

This will eventually be available on PyPI, right now this only works locally with cargo and maturin.

```bash
git clone https://github.com/NCSU-High-Powered-Rocketry-Club/mscl_rs.git && cd mscl-rs
```

## Example Usage

You will need [uv](https://docs.astral.sh/uv/getting-started/installation/) to run the example.

```bash
uv run examples/parse_mscl_rs.py
```

This will automatically build the rust code and install the package in a virtual environment.

You do not need to separately run `maturin develop` at all.