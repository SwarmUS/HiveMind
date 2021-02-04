# HiveMind

The HiveMind is the embedded application that runs on SwarmUS HiveBoard and uses the HiveSight.

## Requirements

* [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) for building the POSIX target
  * [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) should compile and run normally if you have cmake 3.13, but you won't be able to compile the tests since it comes with googletest 1.8 and not 1.10

* [CMake](https://cmake.org/) 3.13

* [Gcc](https://gcc.gnu.org/) or [arm-gcc-none-eabi](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) for the embedded targets.

* [Buzz](https://github.com/MISTLab/Buzz) to compile the user buzz script.

* [Clang tools](https://clang.llvm.org/docs/ClangTools.html) are used to match the style and warnings used in the project
    * [clang-format](https://clang.llvm.org/docs/ClangFormat.html) to match the coding style
    * [clang-tidy](https://clang.llvm.org/extra/clang-tidy/) for additional compiler warnings
* [Doxygen](https://github.com/doxygen/doxygen) and [graphviz](https://gitlab.com/graphviz/graphviz/) to generate the documentation
* [Protoc](https://developers.google.com/protocol-buffers) and some python deps to build [Propolis](https://github.com/SwarmUS/Propolis). Check Propolis repo for more info


## Building
Before building any target other than for the embedded target, the ROS environment variables need to be sourced by running the command 
or adding them to `.bashrc` otherwise the ROS build will fail.
```
source /opt/ros/noetic/setup.bash
```

If you have clang-tidy installed, the build will use it for static analysis.
Check the cmake options for more information.

```
mkdir build
cd build
cmake ..
make
```

### Development
If you want all the warnings used by the team, use this command for cmake build generation

```
cmake -DENABLE_ERROR_ON_MISSING_TOOL=ON -DENABLE_WARNINGS_AS_ERROR=ON -DENABLE_WARNINGS=ON -DENABLE_CLANG_TIDY_CHECK=ON -DENABLE_TESTS=ON -DCMAKE_BUILD_TYPE=Debug ..

```

or using catkin

```
cd catkin_ws
catkin_make -DENABLE_ERROR_ON_MISSING_TOOL=ON -DENABLE_WARNINGS_AS_ERROR=ON -DENABLE_WARNINGS=ON -DENABLE_CLANG_TIDY_CHECK=ON -DENABLE_TESTS=ON -DCMAKE_BUILD_TYPE=Debug ..

```

If you want to build for the embedded target, use the toolchain on your cmake build.

```
cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_TOOLCHAIN_FILE=../cmake/stm32_f429zi_gcc.cmake .. 
```

Note that as of now, the tests can only be built on the native target.

## Running 
You can run the ROS build using this command

```
roslaunch hive_mind hive_mind.launch
```

you can edit, or create a new launch file, to change the parameters

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

### External connections
#### Ethernet port
The firmware assigns a static IP of 192.168.1.10 to the device with a subnet mask of 255.255.255.0.

The ethernet speed can be measured with [iperf 2](https://iperf.fr/iperf-download.php).
To open the server on the device, simply build with the `ENABLE_TARGET_IPERF_SERVER` CMake option:
```
cmake -DCMAKE_BUILD_TYPE=Debug -DENABLE_TARGET_IPERF_SERVER -DCMAKE_TOOLCHAIN_FILE=../cmake/stm32_f429zi_gcc.cmake .. 
```


#### Cellphone
The firmware has been configured with the following pins for the uart connection to a cellphone
| RX  | TX  |
| --- | --- |
| PD6 | PD5 |

![alt text](https://os.mbed.com/media/uploads/jeromecoutant/nucleo_f429zi_zio_left_2019_8_29.png "NUCLEO CONNECTION")


### Configuration
Different firmware settings can be configured at build time (for the STM32 target) or runtime (for the ROS target).

#### STM32 Target
CMake variables can be used to override certain default firmware settings.
| Variable  | Default value   |
| ---       | ---             | 
| UUID      | 1 (0 is reserved for broadcast)|
| HOST_PORT | 5555            |
| HOST_IP   | 192.168.1.101   |

#### ROS Target
ROS launch parameters are used to configure variables.
| Variable          | Default value   |
| ---               | ---             |
| board_uuid        | 1 (0 is reserved for broadcast) |
| host_tcp_port     | 5555            |
| host_tcp_address  | 127.0.0.1       |