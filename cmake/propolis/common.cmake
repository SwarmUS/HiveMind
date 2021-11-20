include(FetchContent)

# Adding modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})

function(propolis_fetch_populate)
    if(PROPOLIS_PATH)
        message(INFO "PROPOLIS_PATH specified, skipping fetch")
    endif()

    FetchContent_Declare(
            ${PROJECT_NAME}_propolis

            GIT_REPOSITORY  https://github.com/SwarmUS/Propolis
            GIT_TAG         1d516c1c2c25089d4cb838a6a99872db7a107ffd
            GIT_PROGRESS    TRUE
    )

    set(PROPOLIS ${PROJECT_NAME}_propolis)
    string(TOLOWER ${PROPOLIS} PROPOLIS_L)

    FetchContent_GetProperties(${PROPOLIS} POPULATED PROPOLIS_POPULATED)
    if(NOT PROPOLIS_POPULATED)
        message("Cloning propolis library")
        set(FETCHCONTENT_QUIET FALSE) # To see progress

        set(ENABLE_TESTS 0) #disable tests for faster builds
        FetchContent_Populate(${PROPOLIS})

        list(APPEND CMAKE_MODULE_PATH ${${PROPOLIS_L}_SOURCE_DIR}/cmake/)
        add_subdirectory(${${PROPOLIS_L}_SOURCE_DIR}/src ${${PROPOLIS_L}_BINARY_DIR})

        # Removing warnings from freertos compilation on executable target
        if (DISABLE_EXTERNAL_WARNINGS)
            target_compile_options(swarmus-propolis-pheromones PRIVATE -w)
            set_target_properties(swarmus-propolis-pheromones PROPERTIES CXX_CLANG_TIDY "" )
        endif()

    endif()

endfunction()