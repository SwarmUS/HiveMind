set(LIB_NAME "swarmus-hivemind-bsp-stm32-interloc-states")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP::Platform::Interloc::States")

set(LIB_SOURCES
    src/InterlocStateContainer.cpp
    src/DefaultState.cpp
    src/SendPollState.cpp
    src/SendFinalState.cpp
    src/WaitPollState.cpp
    src/SendResponseState.cpp
    src/WaitFinalState.cpp
    src/WaitResponseState.cpp
    src/AbstractInterlocState.cpp
        )

set(LIB_HEADERS
    include/states/InterlocStateContainer.h
    include/states/AbstractInterlocState.h
    include/states/DefaultState.h
    include/states/SendFinalState.h
    include/states/SendPollState.h
    include/states/SendResponseState.h
    include/states/WaitPollState.h
    include/states/StateTransition.h
    include/states/WaitFinalState.h
    include/states/WaitResponseState.h
    )

add_library(${LIB_NAME} STATIC ${LIB_SOURCES} ${LIB_HEADERS})

add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
        PUBLIC
        include
        )

target_link_libraries(${LIB_NAME}
        PRIVATE
        # Defined libraries
        SwarmUS::HiveMind::OS
        SwarmUS::HiveMind::Logger
        SwarmUS::HiveMind::HAL
        SwarmUS::HiveMind::BSP::Platform::Interloc
        )