find_program (CLANG_TIDY_EXE NAMES "clang-tidy")
if(CLANG_TIDY_EXE AND ENABLE_CLANG_TIDY_CHECK)
    set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
    # extra args are a small hack so clang-tidy doesn't wine about missing headers like stdio.h, stc
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE};--extra-arg-before=-I/usr/include" CACHE STRING "" FORCE)
else()
    message(STATUS "clang-tidy NOT found!")

    if(ENABLE_ERROR_ON_MISSING_TOOL AND ENABLE_CLANG_TIDY_CHECK)
        message(FATAL_ERROR "Install clang-tidy or disable ENABLE_ERROR_ON_MISSING_TOOL ")
    endif()
endif()

