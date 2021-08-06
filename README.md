[![CI](https://github.com/IMRCLab/libmotioncapture/actions/workflows/CI.yml/badge.svg)](https://github.com/IMRCLab/libmotioncapture/actions/workflows/CI.yml)

# libmotioncapture
Interface Abstraction for Motion Capture System APIs such as VICON, OptiTrack, Qualisys, or VRPN.

This is a fork of https://github.com/USC-ACTLab/libmotioncapture/ with the following changes:

- Python bindings
- Factory method that takes a yaml-string as input
- Refactored API
- Support for VRPN by default

## Compile options
By default, `libmotioncapture` supports the following hardware:

- VICON - SDK git submodule
- Qualisys - SDK git submodule
- OptiTrack - binary parsing over network (no dependency)
- VRPN - SDK git submodule

CMake flags can be used to disable individual systems in `CMakeLists.txt`.

## Prerequisites

```
sudo apt install libboost-system-dev libboost-thread-dev libeigen3-dev ninja-build
```

## Python

```
git submodule init
git submodule update
python3 setup.py develop --user
python3 examples/python.py
```

Wheels for Linux and Mac are built by the CI. Those can be downloaded and installed using `pip install *.whl`.

## C++

```
git submodule init
git submodule update
mkdir build
cd build
cmake ..
```