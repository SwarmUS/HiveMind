#include "interloc/AngleCalculator.h"
#include "interloc/CertitudeCalculator.h"
#include "interloc/Decawave.h"
#include <cmath>

AngleCalculator::AngleCalculator(ILogger& logger) : m_logger(logger) {}

void AngleCalculator::setCalculatorParameters(const AngleCalculatorParameters& parameters) {
    m_calculatorParameters = parameters;

    m_parametersValid = true;
    for (unsigned int i = 0; i < NUM_ANTENNA_PAIRS; i++) {
        if (m_calculatorParameters.m_parametersValidSecretNumbers[i] !=
            ANGLE_PARAMETERS_VALID_SECRET_NUMBER) {
            m_parametersValid = false;
        }
    }
}

std::optional<float> AngleCalculator::calculateAngle(BspInterlocRawAngleData& rawData) {
    if (rawData.m_framesLength < MINIMUM_ANGLE_MEAN || !m_parametersValid) {
        return {};
    }

    std::array<volatile float, NUM_ANTENNA_PAIRS> rawTdoas{};
    std::array<volatile float, NUM_ANTENNA_PAIRS> rawTdoasCertitude{};
    std::array<float, NUM_ANTENNA_PAIRS> fittedTdoas{};
    std::array<int8_t, NUM_ANTENNA_PAIRS> tdoaSlopes{};
    std::array<volatile float, NUM_ANTENNA_PAIRS> rawPdoas{};
    std::array<float, NUM_ANTENNA_PAIRS> fittedpdoas{};

    (void)fittedTdoas;
    (void)fittedpdoas;
    (void)tdoaSlopes;

    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS> tdoaProducedValue;
    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS> fallingSlopeCertitude;
    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS> risingSlopeCertitude;

    std::array<std::array<float, NUM_PDOA_SLOPES << 1>, NUM_ANTENNA_PAIRS> pdoaProducedValue;
    std::array<std::array<float, NUM_PDOA_SLOPES << 1>, NUM_ANTENNA_PAIRS> pdoaCertitude;

    (void)rawPdoas;
    (void)tdoaProducedValue;
    (void)fallingSlopeCertitude;
    (void)risingSlopeCertitude;
    (void)pdoaProducedValue;
    (void)pdoaCertitude;

    for (unsigned int i = 0; i < NUM_ANTENNA_PAIRS; i++) {
        rawTdoas[i] = getRawTdoa(rawData, i, -1);
        rawTdoasCertitude[i] = getTdoaValueCertitude(rawTdoas[i]);
        rawPdoas[i] = getRawPdoa(rawData, i, 1); // TODO: Decide if we want the mean of
        getPdoaValueCertiture(m_calculatorParameters, rawTdoas[i], rawPdoas[i], i,
                              tdoaProducedValue, pdoaProducedValue, pdoaCertitude);
    }
    //    getDecisionCertitude(
    //        rawTdoas, reinterpret_cast<std::array<std::array<float, 2>,
    //        3>&>(fallingSlopeCertitude), risingSlopeCertitude, m_calculatorParameters);

    //    for (unsigned int i = 0; i < NUM_ANTENNA_PAIRS; i++) {
    //                int8_t slopeIdx = getTdoaSlopeIndex(rawTdoas, i);
    //                tdoaSlopes[i] = slopeIdx;
    //    }

    //    for (unsigned int i = 0; i < NUM_ANTENNA_PAIRS; i++) {
    //        if (tdoaSlopes[i] >= 0) {
    //            fittedTdoas[i] = getFittedTdoa(rawTdoas[i], (uint8_t)tdoaSlopes[i], i);
    //            rawPdoas[i] = getRawPdoa(rawData, i, 1); // TODO: Decide if we want the mean of
    //            PDOAs fittedpdoas[i] = getFittedPdoa(rawPdoas[i], fittedTdoas[i],
    //            (uint8_t)tdoaSlopes[i], i);
    //        }
    //    }

    // TODO: Implement logic to decide which value to use
    return fittedpdoas[0];
}

float AngleCalculator::getRawTdoa(BspInterlocRawAngleData& rawData,
                                  uint8_t antennaPair,
                                  int32_t meanLength) {
    volatile float angleAccumulator = 0;
    volatile float tmp = 0;
    uint32_t length = (meanLength <= 0) ? rawData.m_framesLength : (uint32_t)meanLength;

    const auto& antennaIds = m_calculatorParameters.m_antennaPairs[antennaPair];
    for (uint32_t i = 0; i < length; i++) {
        volatile int64_t timeDiff =
            (int64_t)(rawData.m_frames[i].m_frameInfos[antennaIds[0]].m_rxTimestamp -
                      rawData.m_frames[i].m_frameInfos[antennaIds[1]].m_rxTimestamp);

        volatile float tdoaDist = (float)timeDiff / UUS_TO_DWT_TIME * 299.792458;
        tmp += timeDiff;
        tdoaDist /= m_calculatorParameters.m_tdoaNormalizationFactors[antennaPair];
        // tdoaDist *= 1.5;

        if (tdoaDist > 1) {
            tdoaDist = 1;
        } else if (tdoaDist < -1) {
            tdoaDist = -1;
        }

        angleAccumulator += asin(tdoaDist) * 180 / M_PI;
    }

    return angleAccumulator / length;
}

int8_t AngleCalculator::getTdoaSlopeIndex(std::array<float, NUM_ANTENNA_PAIRS> rawTdoas,
                                          uint8_t antennaPair) {
    uint8_t otherPair1 = (antennaPair + 1) % NUM_ANTENNA_PAIRS;
    uint8_t otherPair2 = (antennaPair + 2) % NUM_ANTENNA_PAIRS;

    int8_t decision1 = -1;
    int8_t decision2 = -1;

    if (rawTdoas[otherPair1] >= CONFUSION_LINE_DEGREES) {
        decision1 = (int8_t)(m_calculatorParameters.m_slopeDecisionMatrix[antennaPair][otherPair1]);
    } else if (rawTdoas[otherPair1] <= -CONFUSION_LINE_DEGREES) {
        decision1 =
            m_calculatorParameters.m_slopeDecisionMatrix[antennaPair][otherPair1] == 0 ? 1 : 0;
    }

    if (rawTdoas[otherPair2] >= CONFUSION_LINE_DEGREES) {
        decision2 = (int8_t)(m_calculatorParameters.m_slopeDecisionMatrix[antennaPair][otherPair2]);
    } else if (rawTdoas[otherPair2] <= -CONFUSION_LINE_DEGREES) {
        decision2 =
            m_calculatorParameters.m_slopeDecisionMatrix[antennaPair][otherPair2] == 0 ? 1 : 0;
    }

    if (decision1 == -1 && decision2 == -1) {
        m_allBetweenConfusionLinesErrors++;
        return -1;
    }

    if (decision1 != -1 && decision2 != -1 && decision1 != decision2) {
        m_conflictingSlopeDecisionsErrors++;
        return -1;
    }

    return decision1 != -1 ? decision1 : decision2;
}

float AngleCalculator::getFittedTdoa(float rawTdoa, uint8_t tdoaSlopeIndex, uint8_t antennaPair) {
    float fittedTdoa =
        (rawTdoa - m_calculatorParameters.m_tdoaIntercepts[antennaPair][tdoaSlopeIndex]) /
        m_calculatorParameters.m_tdoaSlopes[antennaPair][tdoaSlopeIndex];

    if (fittedTdoa >= 360) {
        fittedTdoa -= 360;
    }

    return fittedTdoa;
}

float AngleCalculator::getRawPdoa(BspInterlocRawAngleData& rawData,
                                  uint8_t antennaPair,
                                  int32_t meanLength) {
    float angleAccumulator = 0;
    uint32_t length = (meanLength <= 0) ? rawData.m_framesLength : (uint32_t)meanLength;

    const auto& antennaIds = m_calculatorParameters.m_antennaPairs[antennaPair];
    for (uint32_t i = 0; i < length; i++) {
        float phaseDiff = rawData.m_frames[i].m_frameInfos[antennaIds[0]].m_accumulatorAngle -
                          rawData.m_frames[i].m_frameInfos[antennaIds[0]].m_sfdAngle -
                          rawData.m_frames[i].m_frameInfos[antennaIds[1]].m_accumulatorAngle +
                          rawData.m_frames[i].m_frameInfos[antennaIds[1]].m_sfdAngle + M_PI;

        while (phaseDiff > (2 * M_PI)) {
            phaseDiff -= (2 * M_PI);
        }

        while (phaseDiff < 0) {
            phaseDiff += (2 * M_PI);
        }

        phaseDiff -= M_PI;

        angleAccumulator += asin(phaseDiff / M_PI) *
                            m_calculatorParameters.m_pdoaNormalizationFactors[antennaPair] * 180 /
                            M_PI;
    }

    return angleAccumulator / length;
}

float AngleCalculator::getFittedPdoa(float rawPdoa,
                                     float fittedTdoa,
                                     uint8_t tdoaSlopeIndex,
                                     uint8_t antennaPair) {
    volatile float slopeSign = 1;

    if (m_calculatorParameters.m_tdoaSlopes[antennaPair][tdoaSlopeIndex] > 0) {
        slopeSign = -1;
    }

    volatile unsigned int selectedPdoa = 0;
    for (unsigned int i = 0; i < NUM_PDOA_SLOPES; i++) {
        if (fittedTdoa > m_calculatorParameters.m_pdoaOrigins[antennaPair][i]) {
            selectedPdoa = i;
        }
    }

    m_logger.log(LogLevel::Debug, "C");

    return (rawPdoa - m_calculatorParameters.m_pdoaIntercepts[antennaPair][selectedPdoa]) /
           (slopeSign * m_calculatorParameters.m_pdoaSlopes[antennaPair]);
}
