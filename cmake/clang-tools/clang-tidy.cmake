find_program (CLANG_TIDY_EXE NAMES "clang-tidy")
if(CLANG_TIDY_EXE AND ENABLE_CLANG_TIDY_CHECK)
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}" CACHE STRING "" FORCE)
else()
    message(STATUS "clang-tidy NOT found!")

    if(ENABLE_ERROR_ON_MISSING_TOOL)
        message(FATAL_ERROR "Install clang-tidy or disable ENABLE_ERROR_ON_MISSING_TOOL ")
    endif()
endif()

