#ifndef HIVE_MIND_CERTITUDECALCULATOR_H
#define HIVE_MIND_CERTITUDECALCULATOR_H

#include <bsp/AngleCalculatorParameters.h>
#include <logger/ILogger.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

float getTdoaValueCertitude(float tdValue);
void getPdoaValueCertiture(
    const AngleCalculatorParameters& parameters,
    const float tdValue,
    const float pdValue,
    uint8_t antennaPair,
    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS>& tdoaProducedValue,
    std::array<std::array<float, 12 << 1>, 3>& pdoaProducedValue,
    std::array<std::array<float, 12 << 1>, 3>& pdoaCertitude);

void getTdoaSelectionCertitude(
    const float tdValue,
    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS>& certitude,
    uint8_t antennaPair);
void getDecisionCertitude(
    std::array<float, NUM_ANTENNA_PAIRS>& tdValue,
    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS>& fallingSlopeCertitude,
    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS>& risingSlopeCertitude,
    const AngleCalculatorParameters& parameters);

#endif // HIVE_MIND_CERTITUDECALCULATOR_H
