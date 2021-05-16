set(LIB_NAME "swarmus-hivemind-bsp-stm32-interloc")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP::Platform::Interloc")

set(LIB_SOURCES
        src/Decawave.cpp
        src/DecawaveUtils.cpp
        src/InterlocManager.cpp
        src/UWBRxFrame.cpp
        src/InterlocStateHandler.cpp
        src/InterlocBSPContainer.cpp
        src/TwoWayRanging.cpp
        src/InterlocTimeManager.cpp)

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
        include/interloc/InterlocTimeManager.h)

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
        )

target_link_libraries(${LIB_NAME}
        PUBLIC
        # Defined libraries
        SwarmUS::HiveMind::OS
        SwarmUS::HiveMind::Logger
        SwarmUS::HiveMind::HAL
        SwarmUS::HiveMind::BSP::Platform::Interloc::States
        SwarmUS::Propolis::Cpp::Common
        )

add_subdirectory(src/states)

if (DISABLE_EXTERNAL_WARNINGS) 
        target_compile_options(${LIB_NAME} PRIVATE -w)
        set_target_properties(${LIB_NAME} PROPERTIES CXX_CLANG_TIDY "" )
        set_target_properties(${LIB_NAME} PROPERTIES C_CLANG_TIDY "" )
endif()