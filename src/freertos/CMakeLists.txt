set(LIB_NAME "swarmus-hivemind-freertos")
set(LIB_ALIAS "SwarmUS::HiveMind::FreeRTOS")

set(LIB_SOURCES
)

set(LIB_HEADERS
    FreeRTOSConfig.h
)

add_library(${LIB_NAME} INTERFACE)
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
   INTERFACE
       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
       $<INSTALL_INTERFACE:include>
  )
