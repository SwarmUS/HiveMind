#ifndef BITTYBUZZUIFUNCTIONS_H_
#define BITTYBUZZUIFUNCTIONS_H_

#include <bittybuzz/BittyBuzzSystem.h>

/**@brief Buzz library that provides functions for the user to interact with the UI of the platform
 * (LED, Hex display, etc)*/
namespace BittyBuzzUIFunctions {
    /**@brief Sets the user LED on or off*/
    void setLed();

    /**@brief Sets the user sevent segment to a value*/
    void setSegment();
} // namespace BittyBuzzUIFunctions

#endif // BITTYBUZZUIFUNCTIONS_H_
