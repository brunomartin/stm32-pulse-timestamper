# Blinky Hello World application

This project is intended to be build with [ARM GCC][arm gcc] toolchain under 
[CMake][cmake] project manager.

To generate native makefiles structure, please create *build* folder to not pollyte 
your current workspace and launch CMake generation. It is assumed that ARM GCC toolchain path is added to the PATH environment variable.
If not - you could add it with `set` (Windows) or `export` (Unix) command: `export PATH=<path/to/arm-none-eabi/toolchain>:$PATH`.

- Generate for minimum size release
```bash
mkdir -p build && cd build
# Command to generate MinGW Makefiles
cmake -DCMAKE_TOOLCHAIN_FILE=../arm-gcc-toolchain.cmake -DCMAKE_BUILD_TYPE=Release ..
```

- Generate MinGW Makefiles
```bash
mkdir cmake-build-debug
cd cmake-build-debug
# Command to generate MinGW Makefiles
cmake .. -G "MinGW Makefiles" -DCMAKE_TOOLCHAIN_FILE=../arm-gcc-toolchain.cmake -DCMAKE_BUILD_TYPE=Debug
```
- Generate Unix Makefiles
```bash
mkdir cmake-build-debug
cd cmake-build-debug
# Command to generate MinGW Makefiles
cmake .. -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../arm-gcc-toolchain.cmake -DCMAKE_BUILD_TYPE=Debug
```
- Generate Ninja build
```bash
mkdir cmake-build-debug
cd cmake-build-debug
# Command to generate MinGW Makefiles
cmake .. -G Ninja -DCMAKE_TOOLCHAIN_FILE=../arm-gcc-toolchain.cmake -DCMAKE_BUILD_TYPE=Debug
```

[arm gcc]: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm
[cmake]: https://cmake.org/
