#ifndef BITTYBUZZUTILS_H_
#define BITTYBUZZUTILS_H_

#include "BittyBuzzSystem.h"
#include <bbzvm.h>
#include <cstdio>

/**
 *@brief Namespace with various utilities for bittybuzz */
namespace BittyBuzzUtils {

    /**
     *@brief Logs an bittybuzz object to a human readable format
     *
     *@param [in] obj the object to log
     *@param [out] str the output string
     *@param [in] maxSize the maximum size of the string
     *@return On success, the total number of characters written is returned. This count does not
     *include the additional null-character automatically appended at the end of the string. On
     *failure, a negative number is returned.
     */
    int logObj(bbzobj_t* obj, char* str, uint16_t maxSize);

} // namespace BittyBuzzUtils

#endif // BITTYBUZZUTILS_H_
