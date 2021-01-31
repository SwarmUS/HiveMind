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

        GIT_REPOSITORY https://github.com/SwarmUS/Propolis
        GIT_TAG        feat/SWARINFO-154_pheromones-serealization-lib
        GIT_PROGRESS   TRUE
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
        add_subdirectory(${${PROPOLIS_L}_SOURCE_DIR}/src/pheromones ${${PROPOLIS_L}_BINARY_DIR})
    endif()

endfunction()
