set(LIB_NAME "swarmus-hivemind-bsp-stm32-interloc")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP::Platform::Interloc")

set(LIB_SOURCES
        src/Decawave.cpp
        src/DecawaveUtils.cpp
        src/InterlocManager.cpp
        src/UWBRxFrame.cpp)

set(LIB_HEADERS
        include/Decawave.h
        include/DecawaveUtils.h
        include/UWBRxFrame.h
        include/InterlocManager.h
        include/UWBMessages.h)

add_library(${LIB_NAME}
        ${LIB_SOURCES}
        ${LIB_HEADERS}
        )

add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        ${BSP_CMAKE_DIR}/include
        include
        )

target_link_libraries(${LIB_NAME}
        PUBLIC
        # Defined libraries
        SwarmUS::HiveMind::OS
        SwarmUS::HiveMind::Logger

        PRIVATE
        SwarmUS::HiveMind::HAL
        )