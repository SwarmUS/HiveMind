set(LIB_NAME "swarmus-hivemind-bsp-stm32-interloc")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP::Platform::Interloc")

configure_file(include/interloc/InterlocBSPSettings.in.h include/interloc/InterlocBSPSettings.h)

set(LIB_SOURCES
        src/Decawave.cpp
        src/DecawaveUtils.cpp
        src/InterlocManager.cpp
        src/UWBRxFrame.cpp
        src/InterlocStateHandler.cpp
        src/InterlocBSPContainer.cpp
        src/TwoWayRanging.cpp
        src/InterlocTimeManager.cpp
        src/DecawaveArray.cpp
        src/AngleCalculator.cpp
        src/CertitudeCalculator.cpp)

set(LIB_HEADERS
        include/interloc/Decawave.h
        include/interloc/DecawaveUtils.h
        include/interloc/UWBRxFrame.h
        include/interloc/InterlocManager.h
        include/interloc/UWBMessages.h
        include/interloc/InterlocStateHandler.h
        include/interloc/InterlocBSPContainer.h
        include/interloc/TwoWayRanging.h
        include/interloc/DecawaveArray.h
        include/interloc/InterlocTimeManager.h
        include/interloc/AngleCalculator.h
        include/interloc/CertitudeCalculator.h
        include/interloc/InterlocBSPSettings.h)

if (${COMPILE_STM32_F429ZI})
    list(APPEND LIB_SOURCES src/DecawaveArray_HiveSight.cpp)
elseif (${COMPILE_STM32_H735ZG})
    list(APPEND LIB_SOURCES src/DecawaveArray_HiveBoard.cpp)
else ()
    message(FATAL_ERROR "BSP Interloc: The current board is not supported")
endif ()

add_library(${LIB_NAME}
        ${LIB_SOURCES}
        ${LIB_HEADERS}
        )

add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${BSP_CMAKE_DIR}/include
        ${BSP_PLATFORM_CMAKE_DIR}/include

        #Include the generated Settings.h
        ${CMAKE_CURRENT_BINARY_DIR}/include/interloc
        )

target_link_libraries(${LIB_NAME}
        PUBLIC
        # Defined libraries
        SwarmUS::HiveMind::OS
        SwarmUS::HiveMind::Logger
        SwarmUS::HiveMind::HAL
        SwarmUS::HiveMind::BSP::Platform::Interloc::States
        SwarmUS::Propolis::Cpp::Common
        SwarmUS::HiveMind::ApplicationInterface
        SwarmUS::HiveMind::Interloc
        )

add_subdirectory(src/states)

if (DISABLE_EXTERNAL_WARNINGS)
    set_target_properties(${LIB_NAME} PROPERTIES CXX_CLANG_TIDY "")
    set_target_properties(${LIB_NAME} PROPERTIES C_CLANG_TIDY "")
endif ()