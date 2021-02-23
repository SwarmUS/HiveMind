/*! ----------------------------------------------------------------------------
 * @file	deca_mutex.c
 * @brief	IRQ interface / mutex implementation
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#include "deca_device_api.h"
#include "port.h"
// ---------------------------------------------------------------------------
//
// NB: The purpose of this file is to provide for microprocessor interrupt enable/disable, this is
// used for
//     controlling mutual exclusion from critical sections in the code where interrupts and
//     background processing may interact.  The code using this is kept to a minimum and the
//     disabling time is also kept to a minimum, so blanket interrupt disable may be the easiest way
//     to provide this.  But at a minimum those interrupts coming from the decawave device should be
//     disabled/re-enabled by this activity.
//
//     In porting this to a particular microprocessor, the implementer may choose to use #defines in
//     the deca_irq.h include file to map these calls transparently to the target system.
//     Alternatively the appropriate code may be embedded in the functions provided below.
//
//     This mutex dependent on HW port.
//	   If HW port uses EXT_IRQ line to receive ready/busy status from DW1000 then mutex should
// use this signal
//     If HW port not use EXT_IRQ line (i.e. SW polling) then no necessary for decamutex(on/off)
//
//	   For critical section use this mutex instead
//	   __save_intstate()
//     __restore_intstate()
// ---------------------------------------------------------------------------

#define DECAIRQ_EXTI_IRQn (EXTI_LINE_7)

/**
 * @brief  Checks whether the specified EXTI line is enabled or not.
 * @param  EXTI_Line: specifies the EXTI line to check.
 *   This parameter can be:
 *     @arg EXTI_Linex: External interrupt line x where x(0..19)
 * @retval The "enable" state of EXTI_Line (SET or RESET).
 */
ITStatus EXTI_GetITEnStatus(uint32_t x) {
    return ((NVIC->ISER[(((uint32_t)x) >> 5UL)] & (uint32_t)(1UL << (((uint32_t)x) & 0x1FUL))) ==
            (uint32_t)RESET)
               ? (RESET)
               : (SET);
}

/* @fn      port_DisableEXT_IRQ
 * @brief   wrapper to disable DW_IRQ pin IRQ
 *          in current implementation it disables all IRQ from lines 5:9
 * */
__INLINE void port_DisableEXT_IRQ(void) { NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn); }

/* @fn      port_EnableEXT_IRQ
 * @brief   wrapper to enable DW_IRQ pin IRQ
 *          in current implementation it enables all IRQ from lines 5:9
 * */
__INLINE void port_EnableEXT_IRQ(void) { NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn); }

/* @fn      port_GetEXT_IRQStatus
 * @brief   wrapper to read a DW_IRQ pin IRQ status
 * */
__INLINE uint32_t port_GetEXT_IRQStatus(void) { return EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn); }

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * Function: decamutexon()
 *
 * Description: This function should disable interrupts. This is called at the start of a
 * critical section It returns the irq state before disable, this value is used to re-enable in
 * decamutexoff call
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
decaIrqStatus_t decamutexon(void) {
    decaIrqStatus_t s = port_GetEXT_IRQStatus();

    if (s) {
        port_DisableEXT_IRQ(); // disable the external interrupt line
    }
    return s; // return state before disable, value is used to re-enable in decamutexoff call
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * Function: decamutexoff()
 *
 * Description: This function should re-enable interrupts, or at least restore their state as
 * returned(&saved) by decamutexon This is called at the end of a critical section
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 * @param s - the state of the DW1000 interrupt as returned by decamutexon
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
void decamutexoff(decaIrqStatus_t s) // put a function here that re-enables the interrupt at the
                                     // end of the critical section
{
    if (s) { // need to check the port state as we can't use level sensitive interrupt on the
             // STM ARM
        port_EnableEXT_IRQ();
    }
}
