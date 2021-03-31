set(LIB_NAME "swarmus-hivemind-interloc")
set(LIB_ALIAS "SwarmUS::HiveMind::Interloc")

include(DefaultSettings.cmake)
configure_file(include/interloc/InterlocSettings.in.h include/interloc/InterlocSettings.h)

set(LIB_SOURCES
    src/Interloc.cpp
)

set(LIB_HEADERS
    include/interloc/IInterloc.h
    include/interloc/Interloc.h
    include/interloc/RelativePosition.h
    include/interloc/InterlocSettings.h)

add_library(${LIB_NAME} STATIC ${LIB_HEADERS} ${LIB_SOURCES})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
    PUBLIC
       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
       $<INSTALL_INTERFACE:include>

        #Include the generated Settings.h
        ${CMAKE_CURRENT_BINARY_DIR}/include/interloc
)

target_link_libraries(${LIB_NAME}
    PUBLIC
        SwarmUS::HiveMind::BSP
        SwarmUS::HiveMind::OS
        SwarmUS::HiveMind::Logger
)

if(ENABLE_TESTS)
    add_subdirectory(tests)
endif()

