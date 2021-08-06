[![CI](https://github.com/IMRCLab/libmotioncapture/actions/workflows/CI.yml/badge.svg)](https://github.com/IMRCLab/libmotioncapture/actions/workflows/CI.yml)

# libmotioncapture
Interface Abstraction for Motion Capture System APIs such as VICON, OptiTrack, Qualisys, PhaseSpace, or VRPN.

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

Support for following hardware requires additional action by the user:

- Phasespace - manually obtain SDK and copy to `externalDependencies/phasespace_sdk/`.

After setup, enable the appropriate compile flags in `CMakeLists.txt`.

## Prerequisites

```
sudo apt install libpcl-dev libyaml-cpp-dev ninja-build
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