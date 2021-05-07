set(LIB_NAME "swarmus-hivemind-os")
set(LIB_ALIAS "SwarmUS::HiveMind::OS")

set(LIB_SOURCES
 )

set(LIB_HEADERS
)

add_library(${LIB_NAME} STATIC ${LIB_SOURCES} ${LIB_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

# Wraps the interface library in a static library
get_target_property(PROPOLIS_OS_INCLUDES SwarmUS::Propolis::OS::POSIX INTERFACE_INCLUDE_DIRECTORIES)

target_include_directories(${LIB_NAME}
    PUBLIC
       ${PROPOLIS_OS_INCLUDES}
)

target_link_libraries(${LIB_NAME}
    PUBLIC
        SwarmUS::Propolis::OS::Common
    PRIVATE
        SwarmUS::Propolis::OS::POSIX
)


# Removing warnings from freertos compilation on executable target
if (DISABLE_EXTERNAL_WARNINGS) 
    target_compile_options(${LIB_NAME} PRIVATE -w)
endif()
