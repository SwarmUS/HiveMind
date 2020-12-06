set(LIB_NAME "swarmus-hivemind-logger")
set(LIB_ALIAS "SwarmUS::HiveMind::Logger")

set(LIB_SOURCES
    src/logger_impl.cpp
)

set(LIB_HEADERS
    include/logger/logger.h
    include/logger/logger_impl.h
)

add_library(${LIB_NAME} STATIC ${LIB_HEADERS} ${LIB_SOURCES})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
    PUBLIC
       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
       $<INSTALL_INTERFACE:include>
)

target_link_libraries(${LIB_NAME}
    PRIVATE
        SwarmUS::HiveMind::BSP
        SwarmUS::HiveMind::FreeRTOS
)

if(ENABLE_TESTS)
    add_subdirectory(tests)
endif()
