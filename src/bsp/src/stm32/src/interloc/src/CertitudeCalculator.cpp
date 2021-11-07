#include "interloc/CertitudeCalculator.h"

float getTdoaValueCertitude(float tdValue) {
    // TDOA certitude is function of the absolute of the TDOA value.The closer it is to ±90 the
    // worst the certitude is
    float cuttingVal = 10;

    if (abs(tdValue) < cuttingVal) {
        return 1;
    }

    float m = (1 - 0) / (cuttingVal - 90);
    float b = 1 - m * cuttingVal;
    return abs(tdValue) * m + b;
}
float producePdoa(float pdValue,
                  const AngleCalculatorParameters& parameters,
                  const uint8_t pdSlopeId,
                  const uint8_t tdSlopeId,
                  const uint8_t antennaPair) {
    float angle = (pdValue - parameters.m_pdoaIntercepts[antennaPair][pdSlopeId]) /
                  parameters.m_pdoaSlopes[antennaPair] * (pow((float)(-1), (float)(tdSlopeId + 1)));

    while (angle > (float)360) {
        angle -= (float)360;
    }
    while (angle < -(float)360) {
        angle += (float)360;
    };

    return angle;
}
void getPdoaValueCertiture(
    const AngleCalculatorParameters& parameters,
    const float tdValue,
    const float pdValue,
    uint8_t antennaPair,
    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS>& tdoaProducedValue,
    std::array<std::array<float, NUM_PDOA_SLOPES << 1>, NUM_ANTENNA_PAIRS>& pdoaProducedValue,
    std::array<std::array<float, NUM_PDOA_SLOPES << 1>, NUM_ANTENNA_PAIRS>& pdoaCertitude) {

    for (unsigned int tdSlopeId = 0; tdSlopeId < NUM_TDOA_SLOPES; tdSlopeId++) {
        tdoaProducedValue[antennaPair][tdSlopeId] =
            ((tdValue - parameters.m_tdoaIntercepts[antennaPair][tdSlopeId]) /
             parameters.m_tdoaSlopes[antennaPair][tdSlopeId]);

        while (tdoaProducedValue[antennaPair][tdSlopeId] > (float)360) {
            tdoaProducedValue[antennaPair][tdSlopeId] -= (float)360;
        }

        while (tdoaProducedValue[antennaPair][tdSlopeId] < -(float)360) {
            tdoaProducedValue[antennaPair][tdSlopeId] += (float)360;
        }

        for (unsigned int pdSlopeId = 0 + NUM_PDOA_SLOPES * tdSlopeId;
             pdSlopeId < NUM_PDOA_SLOPES * (tdSlopeId + 1); pdSlopeId++) {
            pdoaProducedValue[antennaPair][pdSlopeId] =
                producePdoa(pdValue, parameters, pdSlopeId, tdSlopeId, antennaPair);
        }
    }
    for (unsigned int tdSlopeId = 0; tdSlopeId < NUM_TDOA_SLOPES; tdSlopeId++) {

        for (unsigned int pdSlopeId = 0 + NUM_PDOA_SLOPES * tdSlopeId;
             pdSlopeId < NUM_PDOA_SLOPES * (tdSlopeId + 1); pdSlopeId++) {
            float compareVal = abs(pdoaProducedValue[antennaPair][pdSlopeId] -
                                   tdoaProducedValue[antennaPair][tdSlopeId]);
            if (compareVal < 5) {
                pdoaCertitude[antennaPair][pdSlopeId] = 1;
            } else if (compareVal > 30) {
                pdoaCertitude[antennaPair][pdSlopeId] = 0;
            } else {
                // 5 / x^1.5
                pdoaCertitude[antennaPair][pdSlopeId] = 5 / powf(compareVal, 1.5);
            }
        }
    }
}

void reverse(std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS>& table,
             uint8_t antennaPair) {
    float temp = table[antennaPair][0];
    table[antennaPair][0] = table[antennaPair][1];
    table[antennaPair][1] = temp;
}

void getTdoaSelectionCertitude(
    const float tdValue,
    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS>& certitude,
    uint8_t antennaPair) {
    float val = abs(tdValue);
    float cuttingVal = 45;
    float deadZone = 10;
    float m = (0 - 1) / (deadZone - cuttingVal);
    float b = 1 - m * cuttingVal;
    if (val > cuttingVal) {
        certitude[antennaPair][0] = 1;
        certitude[antennaPair][1] = 0;
    } else if (val < deadZone) {
        certitude[antennaPair][0] = 0;
        certitude[antennaPair][1] = 0;
    } else {
        certitude[antennaPair][0] = val * m + b;
        certitude[antennaPair][1] = 0;
    }
    if (tdValue < 0) {
        reverse(certitude, antennaPair);
    }
}

void getDecisionCertitude(std::array<float, NUM_ANTENNA_PAIRS>& tdValue,
                          std::array<float, NUM_ANTENNA_PAIRS>& fallingSlopeCertitude,
                          std::array<float, NUM_ANTENNA_PAIRS>& risingSlopeCertitude,
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

        if (certitude1[antennaPair][0] > certitude2[antennaPair][0]) {
            fallingSlopeCertitude[antennaPair] = certitude1[antennaPair][0];
        } else {
            fallingSlopeCertitude[antennaPair] = certitude2[antennaPair][0];
        }
        if (certitude1[antennaPair][1] > certitude2[antennaPair][1]) {
            risingSlopeCertitude[antennaPair] = certitude1[antennaPair][1];
        } else {
            risingSlopeCertitude[antennaPair] = certitude2[antennaPair][1];
        }
    }
}