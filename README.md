# Telephonk

- Buy old rotary phone
- Insert GSM modem like SIM800 into it
- Hello? This is a restaurant?
- ???
- Profit

## Requirements

- [arm-gnu-toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
- [CMake](https://cmake.org/download/)
- [Ninja](https://github.com/ninja-build/ninja/releases)

## Build

```bash
cmake --toolchain arm-gnu-toolchain.cmake -G Ninja -B build
cmake --build build
```

## Authors

- **kossnikita**
- **honloan**
