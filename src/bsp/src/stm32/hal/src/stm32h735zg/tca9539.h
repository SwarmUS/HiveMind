#ifndef __TCA9539_H__
#define __TCA9539_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Driver for the TCA9539 IO Expander
 */

/**
 * @brief Configures each pin as input or output
 * @param io0 Bit mask for Port 0 pins (1=Input, 0=Output)
 * @param io1 Bit mask for Port 1 pins (1=Input, 0=Output)
 */
void tca9539_configure(uint8_t io0, uint8_t io1);

/**
 * @brief Sets the outputs for each pin. If a pin is not configured as output, nothing will happen
 * @param io0 Bit mask for Port 0 pins
 * @param io1 Bit mask for Port 1 pins
 */
void tca9539_setOutputs(uint8_t io0, uint8_t io1);

/**
 * @brief Reads inputs from each pin. If a pin is set as an output. The outputted value is returned
 * @param io0 [out] State of Port 0 pins
 * @param io1 [out] State of Port 1 pins
 */
void tca9539_readInputs(uint8_t* io0, uint8_t* io1);

/**
 * Clears any pending interrupts by reading from both IO ports
 */
void tca9539_clearInterrupts();

#ifdef __cplusplus
}
#endif
#endif //__TCA9539_H__
