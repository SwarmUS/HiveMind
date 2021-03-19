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
get_target_property(PROPOLIS_OS_COMMON_INCLUDES SwarmUS::Propolis::OS::Common INTERFACE_INCLUDE_DIRECTORIES)

target_include_directories(${LIB_NAME}
    PUBLIC
       ${PROPOLIS_OS_INCLUDES}
       ${PROPOLIS_OS_COMMON_INCLUDES}
)

target_link_libraries(${LIB_NAME}
    PRIVATE
        SwarmUS::Propolis::OS::Common
        SwarmUS::Propolis::OS::POSIX
)


# Removing warnings from freertos compilation on executable target
if (DISABLE_EXTERNAL_WARNINGS) 
    target_compile_options(${LIB_NAME} PRIVATE -w)
    set_target_properties(${LIB_NAME} PROPERTIES CXX_CLANG_TIDY "" )
    set_target_properties(${LIB_NAME} PROPERTIES C_CLANG_TIDY "" )
endif()
