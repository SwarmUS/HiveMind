set(LIB_NAME "SwarmUS::HiveMind::BSP")

set(LIB_SOURCES
    src/stm32f4/stm32f4xx_hal_conf.h
)

set(LIB_HEADERS
)

add_library(${LIB_NAME} ${LIB_SOURCES})
target_include_directories(${LIB_NAME} PUBLIC inc/)
target_include_directories(${LIB_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE 
    src)
