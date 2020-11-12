# HiveMind

The HiveMind is the embedded application that runs on SwarmUS HiveBoard and uses the HiveSight

## Building

Note that you need arm-gcc-none-eabi available in your path, you can find it [here](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)

```
mkdir build
cd build
cmake ..
make
```

If you want to build for the embedded target, use the toolchain on your cmake build.

```
cmake -D CMAKE_TOOLCHAIN_FILE=../cmake/stm32_f429zi_gcc.cmake .. 
```

Note that as of now, the tests can only be built on the native target.

## Flashing

Install [OpenOCD](http://openocd.org/)
You can flash using the provided make command

```
 openocd -f openocd.cfg -c "program src/hive-mind.elf verify reset exit
```