set(LIB_NAME "swarmus-hivemind-logger")
set(LIB_ALIAS "SwarmUS::HiveMind::Logger")

set(LIB_SOURCES
    src/Logger.cpp
)

set(LIB_HEADERS
    include/logger/ILogger.h
    include/logger/Logger.h
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

target_compile_options(${LIB_NAME}
    PRIVATE
        $<$<BOOL:${ENABLE_WARNINGS}>:${WARNING_FLAG}>
        $<$<BOOL:${ENABLE_WARNINGS_AS_ERROR}>:${WARNING_AS_ERROR_FLAG}>
)

# Removing warnings from freertos compilation on executable target
if (DISABLE_EXTERNAL_WARNINGS) 
    # Since set_source_files_properties scope is only added in cmake 3.18
    # we need to add it everywhere freertos is used
    disable_freertos_warnings()
endif()

if(ENABLE_TESTS)
    add_subdirectory(tests)
endif()

