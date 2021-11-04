#include "interloc/CertitudeCalculator.h"

float getTdoaValueCertitude(float tdValue) {
    // TDOA certitude is function of the absolute of the TDOA value.The closer it is to ±90 the
    // worst the certitude is
    uint8_t cuttingVal = 10;

    if (abs(tdValue) > cuttingVal) {
        return 1;
    }

    float m = (1 - 0) / (cuttingVal - 90);
    float b = 1 - m * cuttingVal;
    return abs(tdValue) * m + b;
}
float producePdoa(float pdValue,
                  const AngleCalculatorParameters& parameters,
                  const uint8_t pdSlopeId,
                  const uint8_t antennaPair) {
    return (pdValue - parameters.m_pdoaIntercepts[pdSlopeId][antennaPair]) /
           parameters.m_pdoaSlopes[antennaPair] * ((-1) ^ (int)(pdSlopeId + 1));
}
void getPdoaValueCertiture(
    const AngleCalculatorParameters& parameters,
    const float tdValue,
    const float pdValue,
    uint8_t antennaPair,
    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS>& tdoaProducedValue,
    std::array<std::array<float, NUM_PDOA_SLOPES << NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS>&
        pdoaProducedValue,
    std::array<std::array<float, NUM_PDOA_SLOPES << NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS>&
        pdoaCertitude) {

    for (unsigned int tdSlopeId = 0; tdSlopeId < NUM_TDOA_SLOPES; tdSlopeId++) {
        tdoaProducedValue[tdSlopeId][antennaPair] =
            ((tdValue - parameters.m_tdoaIntercepts[tdSlopeId][antennaPair]) /
             parameters.m_tdoaSlopes[tdSlopeId][antennaPair]);

        while (tdoaProducedValue[tdSlopeId][antennaPair] > 360) {
            tdoaProducedValue[tdSlopeId][antennaPair] -= 360;
        }

        for (unsigned int pdSlopeId = 0; pdSlopeId < NUM_PDOA_SLOPES; pdSlopeId++) {
            pdoaProducedValue[pdSlopeId][antennaPair] =
                producePdoa(pdValue, parameters, pdSlopeId, antennaPair);

            float compareVal = abs(pdoaProducedValue[pdSlopeId][antennaPair] -
                                   tdoaProducedValue[tdSlopeId][antennaPair]);

            if (compareVal < 5) {
                pdoaCertitude[pdSlopeId][antennaPair] = 1;
            } else if (compareVal > 30) {
                pdoaCertitude[pdSlopeId][antennaPair] = 0;
            } else {
                // 5 / x^1.5
                pdoaCertitude[pdSlopeId][antennaPair] = 5 / powf(compareVal, 1.5);
            }
        }
    }
}

void reverse(std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS>& table,
             uint8_t antennaPair) {
    float temp = table[0][antennaPair];
    table[0][antennaPair] = table[1][antennaPair];
    table[1][antennaPair] = temp;
}

void getTdoaSelectionCertitude(
    const float tdValue,
    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS>& certitude,
    uint8_t antennaPair) {
    float val = abs(tdValue);
    uint8_t cuttingVal = 45;
    uint8_t deadZone = 10;
    float m = (0 - 1) / (deadZone - cuttingVal);
    float b = 1 - m * cuttingVal;
    if (val > cuttingVal) {
        certitude[0][antennaPair] = 1;
        certitude[1][antennaPair] = 0;
    } else if (val < deadZone) {
        certitude[0][antennaPair] = 0;
        certitude[1][antennaPair] = 0;
    } else {
        certitude[0][antennaPair] = val * m + b;
        certitude[1][antennaPair] = 0;
    }
    if (tdValue < 0) {
        reverse(certitude, antennaPair);
    }
}

void getDecisionCertitude(
    std::array<float, NUM_ANTENNA_PAIRS>& tdValue,
    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS>& fallingSlopeCertitude,
    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS>& risingSlopeCertitude,
    const AngleCalculatorParameters& parameters) {

    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS> certitude1;
    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS> certitude2;

    for (uint8_t antennaPair = 0; antennaPair < NUM_ANTENNA_PAIRS; antennaPair++) {
        uint8_t otherPair1 = (antennaPair + 1) % NUM_ANTENNA_PAIRS;
        uint8_t otherPair2 = (antennaPair + 2) % NUM_ANTENNA_PAIRS;

        getTdoaSelectionCertitude(tdValue[otherPair1], certitude1, antennaPair);
        if (parameters.m_slopeDecisionMatrix[antennaPair][otherPair1] == 1) {
            reverse(certitude1, antennaPair);
        }

        getTdoaSelectionCertitude(tdValue[otherPair2], certitude2, antennaPair);
        if (parameters.m_slopeDecisionMatrix[antennaPair][otherPair2] == 1) {
            reverse(certitude2, antennaPair);
        }

        fallingSlopeCertitude[0][antennaPair] = certitude1[0][antennaPair];
        fallingSlopeCertitude[1][antennaPair] = certitude2[0][antennaPair];
        risingSlopeCertitude[0][antennaPair] = certitude1[1][antennaPair];
        risingSlopeCertitude[1][antennaPair] = certitude2[1][antennaPair];
    }
}