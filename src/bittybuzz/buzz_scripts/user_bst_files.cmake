# All the strings symbols have string definition stored on flash to use them a C string.
# This  can take some space on the flash of the device so you may want to remove some global symbols
# Use this file to add user defined bst file to remove them from the flash


# Don't change this variable name it's used
set(USER_BUZZ_BST_FILES
        ${CMAKE_CURRENT_LIST_DIR}/utils/executor.bst
        ${CMAKE_CURRENT_LIST_DIR}/utils/types.bst
)

# Add your included directories
set(USER_BUZZ_INCLUDE_DIRS
)
