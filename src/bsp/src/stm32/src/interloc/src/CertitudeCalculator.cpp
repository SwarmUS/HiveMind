#include "interloc/CertitudeCalculator.h"

float getPdoaValueCertitude(float tdValue) {
    // PDOA certitude is function of the absolute of the PDOA value.The closer it is to ±90 the
    // worst the certitude is
    constexpr float cuttingVal = 10;

    if (abs(tdValue) < cuttingVal) {
        return 1;
    }

    return 0.1F / (abs(tdValue) * 0.01F + pow((abs(tdValue) * 0.01F + 0.3F), 10.0F));
}

void reverse(std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS>& table,
             uint8_t antennaPair) {
    float temp = table[antennaPair][0];
    table[antennaPair][0] = table[antennaPair][1];
    table[antennaPair][1] = temp;
}

void getPdoaSelectionCertitude(
    const float tdValue,
    std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS>& certitude,
    uint8_t antennaPair) {
    constexpr float cuttingVal = 45.0;
    constexpr float deadZone = 10.0;
    constexpr float m = (0 - 1) / (deadZone - cuttingVal);
    constexpr float b = 1 - m * cuttingVal;

    float val = abs(tdValue);

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

void getDecisionCertitude(std::array<float, NUM_ANTENNA_PAIRS>& pdValue,
                          std::array<float, NUM_ANTENNA_PAIRS>& rawPdoasCertitude,
                          std::array<float, NUM_ANTENNA_PAIRS>& fallingSlopeCertitude,
                          std::array<float, NUM_ANTENNA_PAIRS>& risingSlopeCertitude,
                          const AngleCalculatorParameters& parameters) {

    std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS> certitude1{};
    std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS> certitude2{};

    for (uint8_t antennaPair = 0; antennaPair < NUM_ANTENNA_PAIRS; antennaPair++) {
        uint8_t otherPair1 = (antennaPair + 1) % NUM_ANTENNA_PAIRS;
        uint8_t otherPair2 = (antennaPair + 2) % NUM_ANTENNA_PAIRS;

        getPdoaSelectionCertitude(pdValue[otherPair1], certitude1, antennaPair);
        certitude1[antennaPair][0] *= rawPdoasCertitude[otherPair1];
        certitude1[antennaPair][1] *= rawPdoasCertitude[otherPair1];
        if (parameters.m_slopeDecisionMatrix[antennaPair][otherPair1] == 1) {
            reverse(certitude1, antennaPair);
        }

        getPdoaSelectionCertitude(pdValue[otherPair2], certitude2, antennaPair);

        certitude2[antennaPair][0] *= rawPdoasCertitude[otherPair2];
        certitude2[antennaPair][1] *= rawPdoasCertitude[otherPair2];
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