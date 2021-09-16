set(LIB_NAME "swarmus-hivemind-application-interface")
set(LIB_ALIAS "SwarmUS::HiveMind::ApplicationInterface")

set(LIB_SOURCES
  src/ButtonCallbackRegister.cpp

  src/UserUI.cpp

  src/DeviceStateUI.cpp

  src/ConnectionStateUI.cpp

  src/RemoteHandshakeUI.cpp
  src/HostHandshakeUI.cpp

  src/ApplicationInterface.cpp
  src/ApplicationInterfaceContainer.cpp
)

set(LIB_HEADERS
  include/application-interface/IButtonCallbackRegister.h
  include/application-interface/ButtonCallbackRegister.h

  include/application-interface/IUserUI.h
  include/application-interface/UserUI.h

  include/application-interface/IDeviceStateUI.h
  include/application-interface/DeviceStateUI.h

  include/application-interface/IConnectionStateUI.h
  include/application-interface/ConnectionStateUI.h

  include/application-interface/IHandshakeUI.h
  include/application-interface/HostHandshakeUI.h
  include/application-interface/RemoteHandshakeUI.h

  include/application-interface/ApplicationStates.h
  include/application-interface/SystemStates.h
  include/application-interface/UserStates.h

  include/application-interface/IApplicationInterface
  include/application-interface/ApplicationInterface.h

  include/application-interface/ApplicationInterfaceContainer.h
)

add_library(${LIB_NAME} STATIC ${LIB_HEADERS} ${LIB_SOURCES})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
    PUBLIC
       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
       $<INSTALL_INTERFACE:include>

    PRIVATE
       include/application-interface
)

target_link_libraries(${LIB_NAME}
    PUBLIC
        SwarmUS::HiveMind::OS
        SwarmUS::HiveMind::BSP
        SwarmUS::HiveMind::Logger
)

if(ENABLE_TESTS)
    add_subdirectory(tests)
endif()
