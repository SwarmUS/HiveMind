#include "interloc/AngleCalculator.h"
#include "interloc/CertitudeCalculator.h"
#include "interloc/Decawave.h"
#include <cmath>

constexpr float TWO_PI = 2 * M_PI;

AngleCalculator::AngleCalculator(ILogger& logger) : m_logger(logger) {}

float mod(float val, float div) {
    while (val > div) {
        val -= div;
    }
    while (val < 0) {
        val += div;
    }
    return val;
}

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

    std::array<float, NUM_ANTENNA_PAIRS> fallingSlopeCertitude{};
    std::array<float, NUM_ANTENNA_PAIRS> risingSlopeCertitude{};

    std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS> pdoaProducedValue{};

    std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS> pairResult{};

    std::array<std::array<float, NUM_ANTENNA_PAIRS>, MAX_ANGLE_FRAMES> frameLosConfidence{};
    std::array<float, NUM_ANTENNA_PAIRS> meanLosConfidence{};

    computeLineOfSight(rawData, frameLosConfidence, meanLosConfidence);

    for (unsigned int i = 0; i < NUM_ANTENNA_PAIRS; i++) {
        rawPdoas[i] = getRawPdoa(rawData, frameLosConfidence, i, -1);
        rawPdoasCertitude[i] = getPdoaValueCertitude(rawPdoas[i]);

        for (unsigned int j = 0; j < NUM_PDOA_SLOPES; j++) {
            pdoaProducedValue[i][j] = fitPdoa(rawPdoas[i], j, i);
        }
    }

    getDecisionCertitude(rawPdoas, rawPdoasCertitude, fallingSlopeCertitude, risingSlopeCertitude,
                         m_calculatorParameters);

    getPairAngle(pairResult, pdoaProducedValue, rawPdoasCertitude, fallingSlopeCertitude,
                 risingSlopeCertitude, meanLosConfidence);

    float maxLos = *std::max_element(meanLosConfidence.begin(), meanLosConfidence.end());

    std::optional val = getFinalAngle(pairResult);
    if (val && isnan(val.value())) {
        return {{}, {}};
    }
    m_logger.log(LogLevel::Debug, "X");

    if (val) {
        val = mod(val.value() + m_calculatorParameters.m_boardOrientationOffset, 360.0F);
    }
    return {val, maxLos};
}

void AngleCalculator::getPairAngle(
    std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS>& pairResult,
    std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS>& pdoaProducedValue,
    std::array<float, NUM_ANTENNA_PAIRS>& rawPdoaCertitude,
    std::array<float, NUM_ANTENNA_PAIRS>& fallingSlopeCertitude,
    std::array<float, NUM_ANTENNA_PAIRS>& risingSlopeCertitude,
    std::array<float, NUM_ANTENNA_PAIRS>& meanLosConfidence) {
    for (unsigned int antennaPair = 0; antennaPair < NUM_ANTENNA_PAIRS; antennaPair++) {
        if (fallingSlopeCertitude[antennaPair] > risingSlopeCertitude[antennaPair]) {
            pairResult[antennaPair][0] = pdoaProducedValue[antennaPair][0];
            pairResult[antennaPair][1] = fallingSlopeCertitude[antennaPair] *
                                         rawPdoaCertitude[antennaPair] *
                                         meanLosConfidence[antennaPair];
        } else {
            pairResult[antennaPair][0] = pdoaProducedValue[antennaPair][1];
            pairResult[antennaPair][1] = risingSlopeCertitude[antennaPair] *
                                         rawPdoaCertitude[antennaPair] *
                                         meanLosConfidence[antennaPair];
        }
    }
}

std::optional<float> AngleCalculator::getFinalAngle(
    std::array<std::array<float, 2>, NUM_ANTENNA_PAIRS>& pairResult) {
    constexpr float similarCutoff = 25.0F;
    std::array<uint8_t, 3> error = {};
    uint8_t strayCount = 0;

    for (uint8_t i = 0; i < 3; i++) {
        uint8_t otherPair1 = (i + 1) % 3;
        uint8_t otherPair2 = (i + 2) % 3;
        if (i < otherPair1) {
            if (abs(pairResult[i][0] - pairResult[otherPair1][0]) > similarCutoff) {
                error[i]++;
                error[otherPair1]++;
                strayCount++;
            }
        }
        if (i < otherPair2) {
            if (abs(pairResult[i][0] - pairResult[otherPair2][0]) > similarCutoff) {
                error[i]++;
                error[otherPair2]++;
                strayCount++;
            }
        }
    }

    bool skip = false;
    uint8_t skipIdx = 0;
    if (strayCount == 2) {
        skip = true;
        uint8_t maxVal = 0;

        for (uint8_t i = 0; i < 3; i++) {
            if (error[i] > maxVal) {
                maxVal = error[i];
                skipIdx = i;
            }
        }
    }

    float angleSum = 0;
    float pondSum = 0;

    for (unsigned int i = 0; i < 3; i++) {
        if (!skip || i != skipIdx) {
            angleSum += pairResult[i][0] * pairResult[i][1];
            pondSum += pairResult[i][1];
        }
    }

    if (pondSum <= 0.001) {
        return {};
    }

    float angle = angleSum / pondSum;

    return mod(angle, 360.0F);
}

float AngleCalculator::getRawPdoa(
    BspInterlocRawAngleData& rawData,
    std::array<std::array<float, NUM_ANTENNA_PAIRS>, MAX_ANGLE_FRAMES>& losConfidence,
    uint8_t antennaPair,
    int32_t meanLength) {
    float angleAccumulatorReal = 0;
    float angleAccumulatorImaginary = 0;
    float confidenceAccumulator = 0;

    uint32_t length = (meanLength <= 0) ? rawData.m_framesLength : (uint32_t)meanLength;

    const auto& antennaIds = m_calculatorParameters.m_antennaPairs[antennaPair];
    for (uint32_t i = 0; i < length; i++) {
        float phaseDiff = rawData.m_frames[i].m_frameInfos[antennaIds[0]].m_accumulatorAngle -
                          rawData.m_frames[i].m_frameInfos[antennaIds[0]].m_sfdAngle -
                          rawData.m_frames[i].m_frameInfos[antennaIds[1]].m_accumulatorAngle +
                          rawData.m_frames[i].m_frameInfos[antennaIds[1]].m_sfdAngle -
                          (float)M_PI_2;
        phaseDiff = mod(phaseDiff, TWO_PI);

        phaseDiff += m_calculatorParameters.m_pdoaNormalizationFactors[antennaPair];
        phaseDiff = mod(phaseDiff, TWO_PI);

        phaseDiff -= M_PI;

        angleAccumulatorReal += std::cos(phaseDiff) * losConfidence[i][antennaPair];
        angleAccumulatorImaginary += std::sin(phaseDiff) * losConfidence[i][antennaPair];
        confidenceAccumulator += losConfidence[i][antennaPair];
    }

    angleAccumulatorReal /= confidenceAccumulator;
    angleAccumulatorImaginary /= confidenceAccumulator;

    float angle = std::atan2(angleAccumulatorImaginary, angleAccumulatorReal);
    angle = asin(angle / (float)M_PI) / 0.98;
    angle = angle * 180.0F / (float)M_PI;

    return angle;
}

float AngleCalculator::fitPdoa(float pdValue, const uint8_t pdSlopeId, const uint8_t antennaPair) {
    float angle = (pdValue - m_calculatorParameters.m_pdoaIntercepts[antennaPair][pdSlopeId]) /
                  m_calculatorParameters.m_pdoaSlopes[antennaPair][pdSlopeId];

    angle = mod(angle, 360.0F);

    return angle;
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
        meanLosConfidence[i] /= (float)rawData.m_framesLength;
    }
}