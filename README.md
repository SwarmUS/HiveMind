# HiveMind

The HiveMind is the embedded application that runs on SwarmUS HiveBoard and uses the HiveSight

## Building

```
mkdir build
cd build
cmake ..
make
```

If you want to build for the embedded target, use the toolchain on your cmake build.

```
cmake -D CMAKE_TOOLCHAIN_FILE=../cmake/stm32_gcc.cmake .. 
```

Note that as of now, the tests can only be build on the native target
