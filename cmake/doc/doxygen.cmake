find_package(Doxygen)

if (DOXYGEN_FOUND)

    # Set doxygen settings here
    set(DOXYGEN_PROJECT_NAME HiveMind)
    set(DOXYGEN_OUTPUT_DIRECTORY doc)
    set(DOXYGEN_COLLABORATION_GRAPH YES)
    set(DOXYGEN_EXTRACT_ALL YES)
    set(DOXYGEN_CLASS_DIAGRAMS YES)
    set(DOXYGEN_HIDE_UNDOC_RELATIONS NO)
    set(DOXYGEN_HAVE_DOT YES)
    set(DOXYGEN_CLASS_GRAPH YES)
    set(DOXYGEN_CALL_GRAPH YES)
    set(DOXYGEN_CALLER_GRAPH YES)
    set(DOXYGEN_COLLABORATION_GRAPH YES)
    set(DOXYGEN_BUILTIN_STL_SUPPORT YES)
    set(DOXYGEN_EXTRACT_PRIVATE YES)
    set(DOXYGEN_EXTRACT_PACKAGE YES)
    set(DOXYGEN_EXTRACT_STATIC YES)
    set(DOXYGEN_EXTRACT_LOCALMETHODS YES)
    set(DOXYGEN_UML_LOOK YES)
    set(DOXYGEN_UML_LIMIT_NUM_FIELDS 50)
    set(DOXYGEN_TEMPLATE_RELATIONS YES)
    set(DOXYGEN_DOT_GRAPH_MAX_NODES 100)
    set(DOXYGEN_MAX_DOT_GRAPH_DEPTH 0)
    set(DOXYGEN_DOT_TRANSPARENT YES)
    set(DOXYGEN_EXCLUDE_PATTERNS
        "*/tests/*"
        "*/hal/*/Core/*")

    if (${COMPILE_STM32_F429ZI})
        LIST(APPEND DOXYGEN_EXCLUDE_PATTERNS "*/posix/*")
    else()
        LIST(APPEND DOXYGEN_EXCLUDE_PATTERNS "*/stm32/*")
    endif()

    if(ENABLE_WARNINGS_AS_ERROR )
        set(DOXYGEN_WARN_AS_ERROR YES)
    endif()

    doxygen_add_docs(doc
        ALL
        ${PROJECT_SOURCE_DIR}/src
        COMMENT "Generate HiveMind documentation")

else()
    message(STATUS "Doxygen need to be installed to generate the doxygen documentation")

    if(ENABLE_ERROR_ON_MISSING_TOOL)
        message(FATAL_ERROR "Install doxygen or disable ENABLE_ERROR_ON_MISSING_TOOL ")
    endif()
endif()
