# HiveMind

The HiveMind is the embedded application that runs on SwarmUS HiveBoard and uses the HiveSight.

## Requirements

* [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) for building the POSIX target
* [Gcc](https://gcc.gnu.org/) or [arm-gcc-none-eabi](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) for the embedded targets.

* [Buzz](https://github.com/MISTLab/Buzz) to compile the user buzz script.

* [Clang tools](https://clang.llvm.org/docs/ClangTools.html) are used to match the style and warnings used in the project
    * [clang-format](https://clang.llvm.org/docs/ClangFormat.html) to match the coding style
    * [clang-tidy](https://clang.llvm.org/extra/clang-tidy/) for additional compiler warnings
* [Doxygen](https://github.com/doxygen/doxygen) and [graphviz](https://gitlab.com/graphviz/graphviz/) to generate the documentation
* [Protoc] and some python deps (https://github.com/doxygen/doxygen) to build [Pheromones](https://github.com/SwarmUS/Pheromones). Check Pheromones repo for more info


## Building
Before building any target other than for the embedded target, the ROS environment variables need to be sourced by running the command 
or adding them to `.bashrc` otherwise the build will fail.
```
source /opt/ros/noetic/setup.bash
```

If you have clang-tidy installed, the build will use it for static analysis.
Check the cmake options for more information.

```
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Debug ..
make
```

If you don't want to use the build tools and warnings, you can disable them
```
cmake -DENABLE_ERROR_ON_MISSING_TOOL=OFF -DENABLE_WARNINGS_AS_ERROR=OFF -DENABLE_WARNINGS=OFF -DENABLE_CLANG_TIDY_CHECK=OFF ..

```


If you want to build for the embedded target, use the toolchain on your cmake build.

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

## Doc
The documentation is built using [doxygen](https://github.com/doxygen/doxygen).
The doc will be built on `make all` target, if you need to only rebuild the doc, use `make doc`.

An up to date version of the master branch documentation for the native build can be found [here](https://swarmus.github.io/HiveMind/)

## Debugging
OpenOCD has a gdb server that defaults to port 3333, you can then connect to it using arm-none-eabi-gdb. The server can be launched using this command

```
openocd -f ./tools/openocd/stm32_f4/stm32_f4.cfg -c init -c \"reset init\"
```

### TODO
#### BittyBuzz
* Extern C on generated .h file
