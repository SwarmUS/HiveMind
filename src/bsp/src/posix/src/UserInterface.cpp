#include "UserInterface.h"
#include <boost/function.hpp>
#include <cstdio>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

char colorToChar(RgbColor color) {
    switch (color) {
    case RgbColor::RED:
        return 'R';
    case RgbColor::GREEN:
        return 'G';
    case RgbColor::BLUE:
        return 'B';
    case RgbColor::VIOLET:
        return 'V';
    case RgbColor::YELLOW:
        return 'Y';
    case RgbColor::ORANGE:
        return 'O';
    case RgbColor::WHITE:
        return 'W';
    case RgbColor::OFF:
        return '-';
    default:
        return 'x';
    }
}

UIState::UIState() : m_rgbLed(RgbColor::OFF), m_hexDisplay(0x00) {
    m_buttonStates.fill(false);
    m_ledStates.fill(false);
}

UserInterface::UserInterface(const IBSP& bsp) : m_bsp(bsp), m_mutex(10) {}

Mutex& UserInterface::getPrintMutex() { return m_mutex; }

void UserInterface::flush() {
    ROS_INFO("[HM: %d] %s %s", m_bsp.getUUId(), uiStateToString().c_str(),
             m_accumulatedString.c_str());
    m_accumulatedString = "";
}

int UserInterface::print(const char* format, ...) {
    va_list args;
    va_start(args, format);
    int retValue = print(format, args);
    va_end(args);

    return retValue;
}

int UserInterface::print(const char* format, va_list args) {
    // Copy varargs
    va_list vaCopy;
    va_copy(vaCopy, args);
    const int requiredLength = std::vsnprintf(NULL, 0, format, vaCopy);
    va_end(vaCopy);

    // Create a string with adequate length
    std::string tmpStr;
    tmpStr.resize((size_t)requiredLength);

    // Build a new string
    int retValue = vsnprintf(tmpStr.data(), tmpStr.size() + 1, format, args);
    m_accumulatedString = m_accumulatedString + tmpStr;

    return retValue;
}

int UserInterface::printLine(const char* format, ...) {
    va_list args;
    va_start(args, format);
    int retValue = printLine(format, args);
    va_end(args);

    return retValue;
}

int UserInterface::printLine(const char* format, va_list args) {
    int retValue = print(format, args);
    flush();

    return retValue;
}

void UserInterface::setRGBLed(RgbColor color) { m_uiState.m_rgbLed = color; }

void UserInterface::setLed(LED led, bool state) {
    m_uiState.m_ledStates[static_cast<uint>(led)] = state;
}

void UserInterface::setHexDisplay(uint8_t value) { m_uiState.m_hexDisplay = value; }

void UserInterface::setButtonCallback(Button button,
                                      buttonCallbackFunction_t callback,
                                      void* context) {
    ros::NodeHandle nodeHandle = ros::NodeHandle("~");
    uint buttonId = static_cast<uint>(button);

    std::string buttonTopic =
        nodeHandle.param("buttonTopic", std::string("/agent_1/user_interface/button"));
    buttonTopic += "/";
    buttonTopic += std::to_string(buttonId);

    auto bindedFunction = [callback, context](const std_msgs::Empty::ConstPtr& msg) {
        (void)msg;
        callback(context);
    };

    m_buttonSubscribers[buttonId] =
        nodeHandle.subscribe<std_msgs::Empty>(buttonTopic, 10, bindedFunction);
}

std::string UserInterface::uiStateToString() {
    static const std::string_view s_formatStr = "[UI: rgb: %c led: %s hex: %02X]";

    std::string ledStr;
    for (const bool& ledState : m_uiState.m_ledStates) {
        ledStr += std::to_string(static_cast<uint8_t>(ledState));
    }
    const size_t requiredLength =
        (size_t)std::snprintf(NULL, 0, s_formatStr.data(), colorToChar(m_uiState.m_rgbLed),
                              ledStr.data(), m_uiState.m_hexDisplay);
    std::string formattedStr;
    formattedStr.resize(requiredLength);

    std::snprintf(formattedStr.data(), requiredLength + 1, s_formatStr.data(),
                  colorToChar(m_uiState.m_rgbLed), ledStr.data(), m_uiState.m_hexDisplay);

    return formattedStr;
}
