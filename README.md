# HiveMind

The HiveMind is the embedded application that provides swarm capabilities which runs on SwarmUS HiveBoard.

## Requirements

- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) for building the POSIX target

  - [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) should compile and run normally if you have cmake 3.13, but you won't be able to compile the tests since it comes with googletest 1.8 and not 1.10

- [CMake](https://cmake.org/) 3.13

- [Gcc](https://gcc.gnu.org/) or [arm-gcc-none-eabi](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) for the embedded targets.

- [Buzz](https://github.com/buzz-lang/Buzz) to compile the user buzz script.

- Development tools
  - [Clang tools](https://clang.llvm.org/docs/ClangTools.html) are used to match the style and warnings used in the project
    - [clang-format](https://clang.llvm.org/docs/ClangFormat.html) to match the coding style
    - [clang-tidy](https://clang.llvm.org/extra/clang-tidy/) for additional compiler warnings
  - [Doxygen](https://github.com/doxygen/doxygen) and [graphviz](https://gitlab.com/graphviz/graphviz/) to generate the documentation
  - [Protoc](https://developers.google.com/protocol-buffers) and some python deps to build [Propolis](https://github.com/SwarmUS/Propolis). Check Propolis repo for more info

## Building

### ROS
Before building for the ROS environment variables need to be sourced by running the command
or adding them to `.bashrc` otherwise the ROS build will fail.

```
source /opt/ros/noetic/setup.bash
```

In the source directory.

```
mkdir build
cd build
cmake ..
make
```

Or you can simply use `catkin_make` instead.
```sh
cd catkin_ws
catkin_make 
```

### Embedded

You **may** need to add the udev rules and add groups permissions to your user. If you get a `LIBUSB_ERROR_ACCESS` follow these step.

Copy the udev rules
```sh
sudo cp ./tools/udev-rules/* /etc/udev/rules.d/
```


Reload the udev manager
```sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```


Add yourself in the plugdev (to flash) and dialout to read the tty output
```sh
sudo usermod -a -G plugdev $USER
sudo usermod -a -G dialout $USER
```

Then to generate the build system and compile

For the F4
``` sh
cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_TOOLCHAIN_FILE=../cmake/stm32_f429zi_gcc.cmake ..
```

For the H7/HiveBoard
``` sh
cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_TOOLCHAIN_FILE=../cmake/stm32_h735zg_gcc.cmake ..
```

You can then flash using `make flash` and see the log output using `screen /dev/ttyUSBXXX` where XXX is the USB number of the board.

### Development
For development you may want to use the warnings and static checks used by the team

If you want all the warnings used by the team, use this command for cmake build generation.
And if you have clang-tidy installed, the build will use it for static analysis.

Check the cmake options for more information.

```sh
cmake -DENABLE_ERROR_ON_MISSING_TOOL=ON -DENABLE_WARNINGS_AS_ERROR=ON -DENABLE_WARNINGS=ON -DENABLE_CLANG_TIDY_CHECK=ON -DENABLE_TESTS=ON -DCMAKE_BUILD_TYPE=Debug ..
```

or using catkin

```sh
cd catkin_ws
catkin_make -DENABLE_ERROR_ON_MISSING_TOOL=ON -DENABLE_WARNINGS_AS_ERROR=ON -DENABLE_WARNINGS=ON -DENABLE_CLANG_TIDY_CHECK=ON -DENABLE_TESTS=ON -DCMAKE_BUILD_TYPE=Debug ..
```

Just add the required toolchain to build for the embedded targets.

```sh
cmake -DENABLE_ERROR_ON_MISSING_TOOL=ON -DENABLE_WARNINGS_AS_ERROR=ON -DENABLE_WARNINGS=ON -DENABLE_CLANG_TIDY_CHECK=ON -DENABLE_TESTS=ON -DCMAKE_BUILD_TYPE=Debug CMAKE_TOOLCHAIN_FILE=../cmake/TOOLCHAINFILE.cmake ..
```

Note that as of now, the tests can only be built on the native target.

## Running

You can run the ROS build using this command.

```
roslaunch hive_mind hive_mind.launch
```

you can edit, or create a new launch file to change the parameters

## Flashing

Install [OpenOCD](http://openocd.org/). Then you can flash using the provided `make flash`command or directly via openocd.

For the F4
```
openocd -f ./tools/openocd/stm32_f4/stm32_f4.cfg -c "program build/src/hive-mind.elf verify reset exit"
```

For the H7
```
openocd -f ./tools/openocd/stm32_h7/hiveboard.cfg -c "program build/src/hive-mind.elf verify reset exit"
```

## Running tests

### Software Tests

You can use `make test` or `ctest` to launch the tests on the native build.

### Hardware Tests

Two different kind of tests can be run to verify the functionality of the HiveBoard.

The first series of tests use the HAL library directly and are therefore purely hardware/driver tests.
These tests can only be run one the HiveBoard (STM32H7) as some tested components are not available
on the previous Nucleo STM32F4 + HiveSight assembly. To compile the tests, CMake must be run with the
`ENABLE_HARDWARE_TESTS` variable.

```sh
cmake -D CMAKE_BUILD_TYPE=Debug -D ENABLE_HARDWARE_TESTS -D CMAKE_TOOLCHAIN_FILE=../cmake/stm32_h735zg_gcc.cmake ..
```

From there, a second target, `hardware_tests.elf` can be compiled and flashed with OpenOCD.
The `make flash_tests` target can be used to flash.

The tests are not self-validating and must be run through a debugger stepping instruction by instruction to
verify everything is working.

The second series of tests are aimed at validating the functionality of the BeeBoards and more specifically,
the DW1000 UWB Radio. These tests are activated by setting the `DECAWAVE_TESTS` variable to one of three
values:

- `TX`, to transmit data via UWB in a loop
- `RX`, to receive UWB data and log it to the serial logger
- `SPI`, to perform a SPI write/readback test on the DW1000 in a loop
- `LED`, to flash all LEDs that are controlled by the DW1000

For example, to test TX, you would compile with:

```sh
cmake -D CMAKE_BUILD_TYPE=Debug -D DECAWAVE_TESTS=TX -D CMAKE_TOOLCHAIN_FILE=../cmake/TOOLCHAINFILE.cmake ..
```

As with the hardware tests, a make target (`make flash_deca_tests`) was created for flashing the board.

## Formatting

You can run `make format` and `make check-format` to match the formatting convention used.

## Doc

The documentation is built using [doxygen](https://github.com/doxygen/doxygen).
The doc will be built on `make all` target, if you need to only rebuild the doc, use `make doc`.

An up to date version of the master branch documentation for the native build can be found [here](https://swarmus.github.io/HiveMind/)

## Debugging

OpenOCD has a gdb server that defaults to port 3333, you can then connect to it using arm-none-eabi-gdb. The server can be launched using this command

For the F4:
```sh
openocd -f ./tools/openocd/stm32_f4/stm32_f4.cfg -c init -c \"reset init\"
```


For the H7/HiveBoard:
```sh
openocd -f ./tools/openocd/stm32_h7/hiveboard.cfg -c init -c \"reset init\"
```

### External connections

#### Ethernet port

The firmware assigns a static IP of 192.168.1.10 to the device with a subnet mask of 255.255.255.0.

The ethernet speed can be measured with [iperf 2](https://iperf.fr/iperf-download.php).
To open the server on the device, simply build with the `ENABLE_TARGET_IPERF_SERVER` CMake option:

```sh
cmake -DCMAKE_BUILD_TYPE=Debug -DENABLE_TARGET_IPERF_SERVER -DCMAKE_TOOLCHAIN_FILE=TOOLCHAINFILE.cmake ..
```

### Configuration

Different firmware settings can be configured at build time (for the STM32 target) or runtime (for the ROS target).

#### Log level

It's possible to change the log level, check how to configure the build depending on the platform. Here are the levels available:

- Debug
- Info
- Warn
- Error

#### STM32 Target

CMake variables can be used to override certain default firmware settings.
| Variable | Default value |
| --- | --- |
| HOST_PORT | 5555 |
| HOST_IP | 192.168.1.101 |
| LOG_LEVEL | Info |
| MAX_ROBOTS_IN_SWARM | 10 |
| BBZVM_STEP_DELAY_MS | 10 |


The CMake variable `UUID_OVERRIDE` may also be used to change the UUID value currently saved in the
non-volatile memory. Make sure each UUID in your swarm is unique.

#### ROS Target

ROS launch parameters are used to configure variables.
| Variable | Default value |
|---------------------|---------------------------------|
| board_uuid | 1 (0 is reserved for broadcast) |
| host_tcp_port | 5555 |
| host_tcp_address | 127.0.0.1 |
| remote_mock_port | 12346 |
| log_level | Info |
| bbzvm_step_delay_ms | 10 |
