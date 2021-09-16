#ifndef USERINTERFACEMOCK_H_
#define USERINTERFACEMOCK_H_

#include <bsp/IUserInterface.h>
#include <gmock/gmock.h>

class UserInterfaceMock : public IUserInterface {

  public:
    UserInterfaceMock() {}
    ~UserInterfaceMock() override = default;

    MOCK_METHOD(Mutex&, getPrintMutex, (), (override));

    MOCK_METHOD(void, flush, (), (override));

    int printLine(const char* format, ...) override {
        va_list args;
        va_start(args, format);
        va_end(args);
        return 0;
    }

    int printLine(const char* format, va_list args) override {
        (void)format;
        (void)args;
        return 0;
    }

    // GMock des not support variable arguments, so lets mock it ourselves
    int print(const char* format, ...) override {
        va_list args;
        va_start(args, format);
        va_end(args);
        return 0;
    }

    int print(const char* format, va_list args) override {
        (void)format;
        (void)args;
        return 0;
    }

    MOCK_METHOD(void, setRGBLed, (RgbColor color), (override));

    MOCK_METHOD(void, setLed, (LED led, bool state), (override));

    MOCK_METHOD(void, setHexDisplay, (uint8_t value), (override));

    MOCK_METHOD(void,
                setButtonCallback,
                (Button button, buttonCallbackFunction_t callback, void* context),
                (override));
};

#endif // USERINTERFACEMOCK_H_
