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

std::tuple<std::optional<float>, std::optional<float>> AngleCalculator::calculateAngle(
    BspInterlocRawAngleData& rawData) {
    if (rawData.m_framesLength < MINIMUM_ANGLE_MEAN || !m_parametersValid) {
        return {{}, {}};
    }

    std::array<float, NUM_ANTENNA_PAIRS> rawPdoas{};
    std::array<float, NUM_ANTENNA_PAIRS> rawPdoasCertitude{};

    std::array<float, NUM_ANTENNA_PAIRS> fallingSlopeCertitude;
    std::array<float, NUM_ANTENNA_PAIRS> risingSlopeCertitude;

    std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS> pdoaProducedValue;

    std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS> pairResult;

    std::array<std::array<float, NUM_ANTENNA_PAIRS>, MAX_ANGLE_FRAMES> frameLosConfidence;
    std::array<float, NUM_ANTENNA_PAIRS> meanLosConfidence;

    computeLineOfSight(rawData, frameLosConfidence, meanLosConfidence);

    for (unsigned int i = 0; i < NUM_ANTENNA_PAIRS; i++) {
        rawPdoas[i] = getRawPdoa(rawData, frameLosConfidence, i, 1);
        rawPdoasCertitude[i] =
            getPdoaValueCertitude(rawPdoas[i]); // TODO: Decide if we want the mean of PDOAs
    }

    for (unsigned int i = 0; i < NUM_ANTENNA_PAIRS; i++) {
        for (unsigned int j = 0; j < NUM_PDOA_SLOPES; j++) {
            pdoaProducedValue[i][j] = producePdoa(rawPdoas[i], j, i);
        }
    }

    getDecisionCertitude(rawPdoas, fallingSlopeCertitude, risingSlopeCertitude,
                         m_calculatorParameters);

    getPairAngle(pairResult, pdoaProducedValue, rawPdoasCertitude, fallingSlopeCertitude,
                 risingSlopeCertitude, meanLosConfidence);

    float maxLos = *std::max_element(meanLosConfidence.begin(), meanLosConfidence.end());
    return {getFinalAngle(pairResult), maxLos};
}

void AngleCalculator::getPairAngle(
    std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS>& pairResult,
    std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS>& pdoaProducedValue,
    std::array<float, NUM_ANTENNA_PAIRS>& rawPdoaCertitude,
    std::array<float, NUM_ANTENNA_PAIRS>& fallingSlopeCertitude,
    std::array<float, NUM_ANTENNA_PAIRS>& risingSlopeCertitude,
    std::array<float, NUM_ANTENNA_PAIRS>& meanLosConfidence) {
    for (unsigned int antennaPair = 0; antennaPair < NUM_ANTENNA_PAIRS; antennaPair++) {
        float angleSum = 0;
        float pondSum = 0;

        for (unsigned int slopeId = 0; slopeId < NUM_PDOA_SLOPES; slopeId++) {

            float slopeConfidence = slopeId < 1 ? fallingSlopeCertitude[antennaPair]
                                                : risingSlopeCertitude[antennaPair];

            float pond = rawPdoaCertitude[antennaPair] * slopeConfidence;
            if (!isnan(pdoaProducedValue[antennaPair][slopeId]) && !isnan(pond)) {
                angleSum += pdoaProducedValue[antennaPair][slopeId] * pond;
                pondSum += pond;
            }
        }

        if (!isnan(pondSum)) {
            pairResult[antennaPair][0] = angleSum / pondSum;
            pairResult[antennaPair][1] = pondSum * meanLosConfidence[antennaPair];
        } else {
            pairResult[antennaPair][0] = 0;
            pairResult[antennaPair][1] = 0;
        }
    }
}

std::optional<float> weightedAverage(std::array<std::array<float, 2>, 3>& table, uint8_t size) {
    float angleSum = 0;
    float pondSum = 0;

    for (unsigned int i = 0; i < size; i++) {
        angleSum += table[i][0] * table[i][1];
        pondSum += table[i][1];
    }
    if (pondSum == 0) {
        return {};
    }

    float angle = angleSum / pondSum;
    while (angle > (float)360) {
        angle -= (float)360;
    }
    while (angle < -(float)360) {
        angle += (float)360;
    };
    return angle;
}

std::optional<float> AngleCalculator::getFinalAngle(
    std::array<std::array<float, 2>, NUM_ANTENNA_PAIRS>& pairResult) {
    uint8_t hasNoNan = 0;
    std::array<std::array<float, 2>, NUM_ANTENNA_PAIRS> goodVals;
    uint8_t itemCnt = 0;
    for (unsigned int i = 0; i < NUM_ANTENNA_PAIRS; i++) {
        if (!isnan(pairResult[i][0]) && !isnan(pairResult[i][1])) {
            hasNoNan++;
            goodVals[itemCnt][0] = pairResult[i][0];
            goodVals[itemCnt][1] = pairResult[i][1];
            itemCnt++;
        }
    }
    if (hasNoNan == 2) {
        return weightedAverage(goodVals, 2);
    } else if (hasNoNan == 3 or hasNoNan == 0) {
        return weightedAverage(goodVals, 3);
    } else { // hasNoNan == 1
        if (goodVals[0][1] == 0) {
            return {};
        }
        return goodVals[0][0];
    }
}

void AngleCalculator::computeLineOfSight(
    BspInterlocRawAngleData& rawData,
    std::array<std::array<float, NUM_ANTENNA_PAIRS>, MAX_ANGLE_FRAMES>& frameLosConfidence,
    std::array<float, NUM_ANTENNA_PAIRS>& meanLosConfidence) {
    for (unsigned int i = 0; i < rawData.m_framesLength; i++) {
        for (unsigned int j = 0; j < NUM_ANTENNA_PAIRS; j++) {
            uint8_t ant1 = m_calculatorParameters.m_antennaPairs[j][0];
            uint8_t ant2 = m_calculatorParameters.m_antennaPairs[j][1];
            frameLosConfidence[i][j] =
                std::min(rawData.m_frames[i].m_frameInfos[ant1].m_losConfidence,
                         rawData.m_frames[i].m_frameInfos[ant2].m_losConfidence);

            meanLosConfidence[j] += frameLosConfidence[i][j];
        }
    }

    for (unsigned int i = 0; i < NUM_ANTENNA_PAIRS; i++) {
        meanLosConfidence[i] /= rawData.m_framesLength;
    }
}

float AngleCalculator::getRawPdoa(
    BspInterlocRawAngleData& rawData,
    std::array<std::array<float, NUM_ANTENNA_PAIRS>, MAX_ANGLE_FRAMES>& losConfidence,
    uint8_t antennaPair,
    int32_t meanLength) {
    float angleAccumulator = 0;
    float confidenceAccumulator = 0;
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
        float angle = asin(phaseDiff / M_PI) *
                      m_calculatorParameters.m_pdoaNormalizationFactors[antennaPair] * 180 / M_PI;
        angleAccumulator += angle * losConfidence[i][antennaPair];
        confidenceAccumulator += losConfidence[i][antennaPair];
    }

    return angleAccumulator / confidenceAccumulator;
}

float AngleCalculator::producePdoa(float pdValue,
                                   const uint8_t pdSlopeId,
                                   const uint8_t antennaPair) {
    float angle = (pdValue - m_calculatorParameters.m_pdoaIntercepts[antennaPair][pdSlopeId]) /
                  m_calculatorParameters.m_pdoaSlopes[antennaPair][pdSlopeId];

    while (angle > (float)360) {
        angle -= (float)360;
    }
    while (angle < -(float)360) {
        angle += (float)360;
    };

    return angle;
}