set(LIB_NAME "swarmus-hivemind-message-handler")
set(LIB_ALIAS "SwarmUS::HiveMind::MessageHandler")

set(LIB_SOURCES
    src/MessageDispatcher.cpp
    src/MessageSender.cpp
    src/HiveMindHostApiRequestHandler.cpp
    src/HiveConnectHiveMindApiMessageHandler.cpp
    src/MessageHandlerContainer.cpp

    src/GreetHandler.cpp
    src/GreetSender.cpp
)

set(LIB_HEADERS
    include/message-handler/MessageDispatcher.h
    include/message-handler/IMessageDispatcher.h

    include/message-handler/IMessageSender.h
    include/message-handler/MessageSender.h

    include/message-handler/IGreetHandler.h
    include/message-handler/GreetHandler.h

    include/message-handler/IGreetSender.h
    include/message-handler/GreetSender.h

    include/message-handler/IHiveMindHostApiRequestHandler.h
    include/message-handler/HiveMindHostApiRequestHandler.h

    include/message-handler/IHiveConnectHiveMindApiMessageHandler.h
    include/message-handler/HiveConnectHiveMindApiMessageHandler.h

    include/message-handler/MessageHandlerContainer.h
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
        SwarmUS::HiveMind::OS
        SwarmUS::HiveMind::Logger
        SwarmUS::HiveMind::Interloc
        SwarmUS::Propolis::Cpp::Common
        SwarmUS::Propolis::Pheromones
)

if(ENABLE_TESTS)
    add_subdirectory(tests)
endif()
