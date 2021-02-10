set(LIB_NAME "swarmus-hivemind-message-handler")
set(LIB_ALIAS "SwarmUS::HiveMind::MessageHandler")

set(LIB_SOURCES
    src/MessageDispatcher.cpp
    src/MessageSender.cpp
)

set(LIB_HEADERS
    include/message-handler/MessageDispatcher.h
    include/message-handler/IMessageDispatcher.h

    include/message-handler/MessageSender.h
)

add_library(${LIB_NAME} STATIC ${LIB_HEADERS} ${LIB_SOURCES})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
    PUBLIC
       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
       $<INSTALL_INTERFACE:include>

    PRIVATE
       include/message-handler
)

target_link_libraries(${LIB_NAME}
    PUBLIC
        SwarmUS::HiveMind::BSP
        SwarmUS::HiveMind::Logger
        SwarmUS::Propolis::Cpp::Common
        SwarmUS::Propolis::Pheromones::HiveMind::Host
)

if(ENABLE_TESTS)
#    add_subdirectory(tests)
endif()
