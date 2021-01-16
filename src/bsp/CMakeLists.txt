set(LIB_NAME "swarmus-hivemind-bsp")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP")
set(LIB_BSP_LINKER_ALIAS "SwarmUS::HiveMind::BSP::Linker")

get_filename_component(BSP_CMAKE_DIR ${CMAKE_CURRENT_LIST_FILE} DIRECTORY)

add_subdirectory(src)

set(LIB_HEADERS
    include/bsp/IBSP.h
    include/bsp/BSPContainer.h

    include/bsp/IUserInterface.h
    include/bsp/IPhoneCommunication.h

    include/bsp/ITCPClient.h
    include/bsp/SocketContainer.h
)

add_library(${LIB_NAME} STATIC ${LIB_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        src
)

target_link_libraries(${LIB_NAME}
    PRIVATE
        SwarmUS::HiveMind::BSP::Platform
)

if(NOT ${CMAKE_CROSSCOMPILING})
    set_target_properties(${LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)
endif()

