#ifndef __IUSERINTERFACE_H_
#define __IUSERINTERFACE_H_

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
     * @brief Provides an interface to print to the console or serial port.
     * The arguments and return values match the standard printf library
     *
     * @param [in] format Text to be written, can contain format specifiers that will be replaced by
     *values specified in the additionnal arguments, matches the standard printf function
     *
     * @param [in] ... Additionnal arguments for the format parameter
     *
     * @return Matches the standard printf return. The total number of characters is returned or a
     *negative number on error
     */
    virtual int print(const char* format, ...) const = 0;
};

#endif // __IUSERINTERFACE_H_
