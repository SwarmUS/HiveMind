 find_program (CLANG_TIDY_EXE NAMES "clang-tidy")
if(CLANG_TIDY_EXE)
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE};-format-style=file;-header-filter='${CMAKE_SOURCE_DIR}/*'" CACHE STRING "" FORCE)
else()
    message(STATUS "clang-tidy NOT found!")
    set(CMAKE_CXX_CLANG_TIDY "" CACHE STRING "" FORCE) # clear it
endif()

# 
# find_program(CLANG_TIDY "clang-tidy")
# 
# if(CLANG_TIDY)
#     set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)

#     file(GLOB_RECURSE ALL_SOURCE_FILES
#         ${SOURCE_DIR}/*.[ch]pp
#         ${SOURCE_DIR}/*.[ch]
#         ${SOURCE_DIR}/*.[h]pp
#         ${SOURCE_DIR}/*.[h]
#     )

#     add_custom_target(
#         tidy
#         COMMAND clang-tidy
#         ${ALL_SOURCE_FILES}
#         -format-style=file
#         --
#         -std=c++17
#     )
# endif()
