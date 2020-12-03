#ifndef __UI_H_
#define __UI_H_

/**
 * @brief Manages the user interface
 * The user interface can consist of buttons, LED, serial print ports, etc.
 * It is not a graphical interface.
 * The board will communicate it's state via the UI and the user can send commands via the UI.
 */
class UI {
  public:
    UI();
    ~UI();

    /**
     * @brief Provides an interface to print to the console or serial port.
     * The arguemnts and return values matches the standard printf library
     *
     * @param format text to be written, can contain format specifiers that will be replace by
     *values specified in the additionnal arguments.
     *
     */
    virtual int printf(const char* format, ...) = 0;
};

#endif // __UI_H_
