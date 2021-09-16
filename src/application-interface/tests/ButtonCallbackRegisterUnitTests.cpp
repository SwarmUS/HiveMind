#include "mocks/ApplicationInterfaceMock.h"
#include <application-interface/ButtonCallbackRegister.h>
#include <gtest/gtest.h>

class ButtonCallbackRegisterFixture : public testing::Test {
  protected:
    ButtonCallbackRegister* m_buttonCallbackRegister;

    static const Button m_button = Button::BUTTON_0;
    testing::StrictMock<ApplicationInterfaceMock> m_appInterfaceMock;

    void SetUp() override {
        m_buttonCallbackRegister = new ButtonCallbackRegister(m_appInterfaceMock, m_button);
    }
    void TearDown() override { delete m_buttonCallbackRegister; }
};

void btn_testfun(void* context) { (void)context; }
TEST_F(ButtonCallbackRegisterFixture, ButtonCallbackRegister_setSystemButtonCallback) {
    // Given
    EXPECT_CALL(m_appInterfaceMock, setSystemButtonCallback(m_button, btn_testfun, NULL)).Times(1);

    // Then
    m_buttonCallbackRegister->setCallback(btn_testfun, NULL);

    // Expect
}
