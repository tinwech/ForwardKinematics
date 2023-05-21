# Forward Kinematics

This repository implements forward kinematics using C++ and OpenGL library.

## Demo

![ezgif-5-aff95fee08](https://github.com/tinwech/ForwardKinematics/assets/80531783/a01b6486-bc25-495d-a179-d3c82214ea8a)

## Build on Microsoft Windows with Visual Studio 2017/2019

### Instruction

- Open SoftSim.sln
- Build
- Executable will be in ./bin

## Build on other platforms and/or compilers

### Prerequisite

- [Cmake](https://cmake.org) (version >= 3.14)
- Compiler (e.g. GCC)

### Instruction

- Run:

```bash=
cmake -S . -B build
cmake --build build --config Release --target install --parallel 8
```
- Executable will be in ./bin

If you are building on Linux, you need one of these dependencies, usually `xorg-dev`

- `xorg-dev` (For X11)
- `libwayland-dev wayland-protocols extra-cmake-modules libxkbcommon-dev` (For Wayland)
- `libosmesa6-dev` (For OSMesa)
