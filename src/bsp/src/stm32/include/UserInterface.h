#ifndef __USERINTERFACE_H_
#define __USERINTERFACE_H_

#include "bsp/IUserInterface.h"

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

class UserInterface : public IUserInterface {
  public:
    UserInterface();
    ~UserInterface() override = default;

    Mutex& getPrintMutex() override;

    void flush() override;
    int print(const char* format, ...) override;
    int print(const char* format, va_list args) override;
    int printLine(const char* format, ...) override;
    int printLine(const char* format, va_list args) override;

    // TODO: The following functions should be made private and have more generic setStateX()
    // functions that then call these functions appropriately
    /**
     * @brief Sets the RGB LED to a given color
     * @param color The color to set
     */
    void setRGBLed(RgbColor color);

    /**
     * @brief Sets the hex display to a given 8 bit value (not available on the HiveSight)
     * @param value The value to set
     */
    void setHexDisplay(uint8_t value);

    /**
     * @brief Sets the callback associated with a given button press
     * @param button Button to register the callback on
     * @param callback Callback to call
     * @param context Context to pass to the callback
     */
    void setButtonCallback(Button button, buttonCallbackFunction_t callback, void* context);

  private:
    Mutex m_mutex;
};

#endif // __USERINTERFACE_H_
