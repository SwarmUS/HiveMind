/*! ----------------------------------------------------------------------------
 * @file    deca_sleep.c
 * @brief   platform dependent sleep implementation
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include "deca_device_api.h"
#include "port.h"
#include "sleep.h"

/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
__INLINE void deca_sleep(unsigned int time_ms) { HAL_Delay(time_ms); }
