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
        src/IdleState.cpp
        src/SyncState.cpp
        src/SendPollFromSyncState.cpp
        src/UpdateInterloc.cpp
        src/AngleReceiverState.cpp
        src/AngleSenderState.cpp
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
        include/states/IdleState.h
        include/states/SyncState.h
        include/states/SendPollFromSyncState.h
        include/states/UpdateInterloc.h
        include/states/AngleSenderState.h
        include/states/AngleReceiverState.h
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

if (DISABLE_EXTERNAL_WARNINGS) 
        set_target_properties(${LIB_NAME} PROPERTIES CXX_CLANG_TIDY "" )
        set_target_properties(${LIB_NAME} PROPERTIES C_CLANG_TIDY "" )
endif()