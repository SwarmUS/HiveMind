#include "interloc/AngleCalculator.h"
#include "interloc/Decawave.h"
#include <cmath>

AngleCalculator::AngleCalculator(ILogger& logger) : m_logger(logger) {}

void AngleCalculator::setCalculatorParameters(const AngleCalculatorParameters& parameters) {
    m_calculatorParameters = parameters;

    // *****************************
    // TEMPORARY, DEFAULT PARAMS
    // *****************************
    m_calculatorParameters.m_antennaPairs[0] = {0, 1};
    m_calculatorParameters.m_slopeDecisionMatrix[0] = {0, 1, 1};
    m_calculatorParameters.m_tdoaNormalizationFactors[0] = 1;
    m_calculatorParameters.m_tdoaSlopes[0] = {1, -1};
    m_calculatorParameters.m_tdoaIntercepts[0] = {0, 1};
    m_calculatorParameters.m_pdoaNormalizationFactors[0] = 1 / 0.75;
    m_calculatorParameters.m_pdoaSlopes[0] = 1;
    m_calculatorParameters.m_pdoaIntercepts[0] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    m_calculatorParameters.m_pdoaOrigins[0] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

    m_calculatorParameters.m_antennaPairs[1] = {0, 2};
    m_calculatorParameters.m_slopeDecisionMatrix[1] = {0, 1, 1};
    m_calculatorParameters.m_tdoaNormalizationFactors[1] = 1;
    m_calculatorParameters.m_tdoaSlopes[1] = {1, -1};
    m_calculatorParameters.m_tdoaIntercepts[1] = {0, 1};
    m_calculatorParameters.m_pdoaNormalizationFactors[1] = 1 / 0.75;
    m_calculatorParameters.m_pdoaSlopes[1] = 1;
    m_calculatorParameters.m_pdoaIntercepts[1] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    m_calculatorParameters.m_pdoaOrigins[1] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

    m_calculatorParameters.m_antennaPairs[2] = {1, 2};
    m_calculatorParameters.m_slopeDecisionMatrix[2] = {0, 1, 1};
    m_calculatorParameters.m_tdoaNormalizationFactors[2] = 1;
    m_calculatorParameters.m_tdoaSlopes[2] = {1, -1};
    m_calculatorParameters.m_tdoaIntercepts[2] = {0, 1};
    m_calculatorParameters.m_pdoaNormalizationFactors[2] = 1 / 0.75;
    m_calculatorParameters.m_pdoaSlopes[2] = 1;
    m_calculatorParameters.m_pdoaIntercepts[2] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    m_calculatorParameters.m_pdoaOrigins[2] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
}

std::optional<float> AngleCalculator::calculateAngle(BspInterlocRawAngleData& rawData) {
    if (rawData.m_framesLength < MINIMUM_ANGLE_MEAN) {
        return {};
    }

    std::array<float, NUM_ANTENNA_PAIRS> tdoas{};
    std::array<uint8_t, NUM_ANTENNA_PAIRS> tdoaSlopes{};
    std::array<float, NUM_ANTENNA_PAIRS> pdoas{};

    for (unsigned int i = 0; i < NUM_ANTENNA_PAIRS; i++) {
        tdoas[i] = getRawTdoa(rawData, i, -1);
    }

    for (unsigned int i = 0; i < NUM_ANTENNA_PAIRS; i++) {
        int8_t slopeIdx = getTdoaSlopeIndex(tdoas, i);
        if (slopeIdx < 0) {
            return {};
        }

        tdoaSlopes[i] = (uint8_t)slopeIdx;
    }

    for (unsigned int i = 0; i < NUM_ANTENNA_PAIRS; i++) {
        float fittedTdoa = getFittedTdoa(tdoas[i], tdoaSlopes[i], i);
        float rawPdoa = getRawPdoa(rawData, i, 1); // TODO: Decide if we want the mean of PDOAs
        pdoas[i] = getFittedPdoa(rawPdoa, fittedTdoa, tdoaSlopes[i], i);
    }

    // TODO: Implement logic to decide which value to use
    return (pdoas[0] + pdoas[1] + pdoas[2]) / 3;
}

float AngleCalculator::getRawTdoa(BspInterlocRawAngleData& rawData,
                                  uint8_t antennaPair,
                                  int32_t meanLength) {
    float angleAccumulator = 0;
    uint32_t length = (meanLength <= 0) ? rawData.m_framesLength : (uint32_t)meanLength;

    const auto& antennaIds = m_calculatorParameters.m_antennaPairs[antennaPair];
    for (uint32_t i = 0; i < length; i++) {
        int64_t timeDiff = (int64_t)(rawData.m_frames[i].m_frameInfos[antennaIds[0]].m_rxTimestamp -
                                     rawData.m_frames[i].m_frameInfos[antennaIds[1]].m_rxTimestamp);

        float tdoaDist = (float)timeDiff / UUS_TO_DWT_TIME * 299.792458;
        tdoaDist /= m_calculatorParameters.m_tdoaNormalizationFactors[antennaPair];

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
    } else if (rawTdoas[otherPair1] <= CONFUSION_LINE_DEGREES) {
        decision1 =
            m_calculatorParameters.m_slopeDecisionMatrix[antennaPair][otherPair1] == 0 ? 1 : 0;
    }

    if (rawTdoas[otherPair2] >= CONFUSION_LINE_DEGREES) {
        decision2 = (int8_t)(m_calculatorParameters.m_slopeDecisionMatrix[antennaPair][otherPair2]);
    } else if (rawTdoas[otherPair2] <= CONFUSION_LINE_DEGREES) {
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
    float slopeSign = 1;

    if (m_calculatorParameters.m_tdoaSlopes[antennaPair][tdoaSlopeIndex] > 0) {
        slopeSign = -1;
    }

    unsigned int selectedPdoa = 0;
    for (unsigned int i = 0; i < NUM_PDOA_SLOPES; i++) {
        if (fittedTdoa > m_calculatorParameters.m_pdoaOrigins[antennaPair][i]) {
            selectedPdoa = i;
            break;
        }
    }

    return (rawPdoa - m_calculatorParameters.m_pdoaIntercepts[antennaPair][selectedPdoa]) /
           (slopeSign * m_calculatorParameters.m_pdoaSlopes[antennaPair]);
}