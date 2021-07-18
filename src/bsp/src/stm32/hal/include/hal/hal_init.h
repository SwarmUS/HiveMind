#ifndef __HAL_INIT_H__
#define __HAL_INIT_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes board specific HAL peripherals
 */
void Hal_initPlatformSpecific();

#ifdef __cplusplus
}
#endif

#endif //__HAL_INIT_H__
