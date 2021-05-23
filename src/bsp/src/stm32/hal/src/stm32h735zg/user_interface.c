#include "hal/user_interface.h"
#include "tca9539.h"

// https://forum.hosteng.com/wndm/HTMLHelp1/Instruction_Set/SEG_Hex_BCD_to_7_Segment_Display.htm
static uint8_t g_binaryToSegmentMap[16] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07,
                                           0x7F, 0x67, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71};

void UI_initialize() { tca9539_configure(0x00, 0xC0); }

void UI_setHexOutput(uint8_t hexValue) {
    uint8_t hexDigit0 = hexValue & 0x0F;
    uint8_t hexDigit1 = (hexValue >> 4) & 0x0F;

    // IO0 has segment A_1 on pin 7 and G_0 to A_0 on pins 6 to 0
    uint8_t io0 = (((g_binaryToSegmentMap[hexDigit1] << 7) & 0x80) |
                   (g_binaryToSegmentMap[hexDigit0] & 0x7F));

    // IO1 has segments G_1 to B_1 on pins 5 to 0
    uint8_t io1 = (g_binaryToSegmentMap[hexDigit1] >> 1) & 0x3F;

    tca9539_setOutputs(io0, io1);
}
