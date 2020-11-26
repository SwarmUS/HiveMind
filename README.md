# HiveMind

The HiveMind is the embedded application that runs on SwarmUS HiveBoard and uses the HiveSight.

## Building

If you have clang-tidy installed, the build will use it for static analysis.
Check the cmake options for more information.

```
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Debug ..
make
```

If you want to build for the embedded target, use the toolchain on your cmake build.
Note that you need arm-gcc-none-eabi available in your path, you can find it [here](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)

```
cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_TOOLCHAIN_FILE=../cmake/stm32_f429zi_gcc.cmake .. 
```

Note that as of now, the tests can only be built on the native target.

## Flashing

Install [OpenOCD](http://openocd.org/). Then you can flash using the provided `make flash`command or directly via openocd.

```
openocd -f ./tools/openocd/stm32_f4/stm32_f4.cfg -c "program build/src/hive-mind.elf verify reset exit"
```

## Running tests
You can use `make test` or `ctest` to launch the tests.

## Formatting
You can run `make format` and `make check-format` to match the formatting convention used.

## Debugging
OpenOCD has a gdb server that defaults to port 3333, you can then connect to it using arm-none-eabi-gdb. The server can be launched using this command

```
openocd -f ./tools/openocd/stm32_f4/stm32_f4.cfg -c init -c \"reset init\"
```
