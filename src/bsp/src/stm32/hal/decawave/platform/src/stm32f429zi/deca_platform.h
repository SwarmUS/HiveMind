#ifndef HIVE_MIND_DECA_PLATFORM_H
#define HIVE_MIND_DECA_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#define DECA_SPI_SLOW_RATE SPI_BAUDRATEPRESCALER_32
#define DECA_SPI_FAST_RATE SPI_BAUDRATEPRESCALER_32

#define DECA_IRQ_IDLE_STATE GPIO_PIN_RESET

/**
 * @brief Enum to specify which decawave we are addressing
 */
typedef enum { DW_A0 = 0, DW_B0 = 1 } decaDevice_t;

#ifdef __cplusplus
}
#endif

#endif // HIVE_MIND_DECA_PLATFORM_H
