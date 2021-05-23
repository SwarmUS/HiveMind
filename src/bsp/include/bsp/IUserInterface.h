#ifndef __IUSERINTERFACE_H_
#define __IUSERINTERFACE_H_

#include <Mutex.h>
#include <cstdarg>

/**
 * @brief Represents LEDs present on the boards.
 * (On the HiveSight, LED 3 is not present)
 */
enum class Led { LED_0 = 0, LED_1, LED_2, LED_3 };

/**
 * @brief Possible colors obtainable with an RGB LED
 */
enum class RgbColor { RED = 0, GREEN, BLUE, VIOLET, YELLOW, BROWN, WHITE, OFF };

/**
 * @brief Buttons present on the board.
 * (Button 1 is not available on the HiveSight)
 */
enum class Button { BUTTON_0 = 0, BUTTON_1 };

/**
 * @brief Prototype for a callback from a button press
 */
typedef void (*buttonCallbackFunction_t)(void* instance);

/**
 * @brief Manages the user interface
 * The user interface can consist of buttons, LED, serial print ports, etc.
 * It is not a graphical interface.
 * The board will communicate its state via the UI and the user can send commands via the UI.
 */
class IUserInterface {
  public:
    virtual ~IUserInterface() = default;

    /**
     *@brief get the mutex for printing. Note that the mutex is not used in any functions, the user
     *needs to lock and unlock this mutex*/
    virtual Mutex& getPrintMutex() = 0;

    /**
     *@brief Adds a newline and flushes the input to the serial port */
    virtual void flush() = 0;

    /**
     * @brief Provides an interface to print to the console or serial port.
     * The arguments and return values match the standard printf library. A flush needs to be called
     *after this function
     * @param [in] format Text to be written, can contain format specifiers that will be replaced by
     *values specified in the additional arguments, matches the standard printf function
     * @param [in] ... Additionnal arguments for the format parameter
     * @return Matches the standard printf return. The total number of characters is returned or a
     *negative number on error
     */
    virtual int print(const char* format, ...) = 0;

    /**
     * @brief Provides an interface to print to the console or serial port using an initialized
     *va_list. The return value matches the standard printf library. A flush needs to be called
     *after this function
     * @param [in] format Text to be written, can contain format specifiers that will be replaced by
     *values specified in the additional arguments, matches the standard printf function
     * @param [in] args Previously initialized va_list
     * @return Matches the standard printf return. The total number of characters is returned or a
     *negative number on error
     */
    virtual int print(const char* format, va_list args) = 0;

    /**
     * @brief Provides an interface to print a line to the console or serial port. Flushes the input
     *and adds a newline. The arguments and return values match the standard printf library
     * @param [in] format Text to be written, can contain format specifiers that will be replaced by
     *values specified in the additional arguments, matches the standard printf function
     * @param [in] ... Additionnal arguments for the format parameter
     * @return Matches the standard printf return. The total number of characters is returned or a
     *negative number on error
     */
    virtual int printLine(const char* format, ...) = 0;

    /**
     * @brief Provides an interface to print to the console or serial port using an initialized
     *va_list. Flushes the input and adds a newline. The return value matches the standard printf
     *library.
     * @param [in] format Text to be written, can contain format specifiers that will be replaced by
     *values specified in the additional arguments, matches the standard printf function
     * @param [in] args Previously initialized va_list
     * @return Matches the standard printf return. The total number of characters is returned or a
     *negative number on error
     */
    virtual int printLine(const char* format, va_list args) = 0;

    /**
     * @brief Sets the state of a LED on the board
     * @param led The led to operate
     * @param state True for ON and false for OFF
     */
    virtual void setLedState(Led led, bool state) = 0;

    /**
     * @brief Sets the RGB LED to a given color
     * @param color The color to set
     */
    virtual void setRGBLed(RgbColor color) = 0;

    /**
     * @brief Sets the hex display to a given 8 bit value (not available on the HiveSight)
     * @param value The value to set
     */
    virtual void setHexDisplay(uint8_t value) = 0;

    /**
     * @brief Sets the callback associated with a given button press
     * @param button Button to register the callback on
     * @param callback Callback to call
     * @param context Context to pass to the callback
     */
    virtual void setButtonCallback(Button button,
                                   buttonCallbackFunction_t callback,
                                   void* context) = 0;
};

#endif // __IUSERINTERFACE_H_
