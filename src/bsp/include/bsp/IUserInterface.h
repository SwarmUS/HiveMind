#ifndef __IUSERINTERFACE_H_
#define __IUSERINTERFACE_H_

#include <cstdarg>

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
};

#endif // __IUSERINTERFACE_H_
