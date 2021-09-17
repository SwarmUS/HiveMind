#ifndef BITTYBUZZUIFUNCTIONS_H_
#define BITTYBUZZUIFUNCTIONS_H_

#include <bittybuzz/BittyBuzzSystem.h>

/**
 * @brief Buzz library that provides functions for the user to interact with the UI of the platform
 * (LED, Hex display, etc)
 * */
namespace BittyBuzzUIFunctions {
    /**
     *@brief Sets the user LED on or off
     *@b Signature set_led(arg1)
     *@details This closure expects an epxression. all ints that are not 0 evaluates to true (led
     *on), all other types evaluates to false (led off)
     *@code
     *  ui.set_led(1); # on
     *  ui.set_led(16); # on
     *  ui.set_led(0); # off
     *  ui.set_led(nil); # off
     *  ui.set_led(""); # off
     *@endcode
     */
    void setLed();

    /**
     * @brief Sets the user sevent segment to a value
     *@b Signature set_hex(arg1)
     *@details This closure expects an int and will bound between 0 and 15
     *@code
     *  ui.set_hex(6); # 0x06
     *  ui.set_hex(10); # 0x0A
     *  ui.set_hex(20); # Bounded 0x0F
     *  ui.set_hex(-30); # Bounded 0x00
     *  ui.set_hex(""); # Type error
     *@endcode */
    void setHex();
} // namespace BittyBuzzUIFunctions

#endif // BITTYBUZZUIFUNCTIONS_H_
