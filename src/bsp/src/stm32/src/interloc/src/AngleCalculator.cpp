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
void getPairAngle(
    std::array<std::array<float, 2>, NUM_ANTENNA_PAIRS>& pairResult,
    std::array<float, NUM_ANTENNA_PAIRS>& rawTdoasCertitude,
    std::array<std::array<float, NUM_PDOA_SLOPES << 1>, NUM_ANTENNA_PAIRS>& pdoaProducedValue,
    std::array<std::array<float, NUM_PDOA_SLOPES << 1>, NUM_ANTENNA_PAIRS>& pdoaCertitude,
    std::array<float, NUM_ANTENNA_PAIRS>& fallingSlopeCertitude,
    std::array<float, NUM_ANTENNA_PAIRS>& risingSlopeCertitude) {
    for (unsigned int antennaPair = 0; antennaPair < NUM_ANTENNA_PAIRS; antennaPair++) {
        float angleSum = 0;
        float pondSum = 0;

        for (unsigned int tdSlopeId = 0; tdSlopeId < NUM_TDOA_SLOPES; tdSlopeId++) {
            for (unsigned int pdSlopeId = 0 + NUM_PDOA_SLOPES * tdSlopeId;
                 pdSlopeId < NUM_PDOA_SLOPES * (tdSlopeId + 1); pdSlopeId++) {

                float slopeConfiance = tdSlopeId < 1 ? fallingSlopeCertitude[antennaPair]
                                                     : risingSlopeCertitude[antennaPair];
                float pond = pdoaCertitude[antennaPair][pdSlopeId] *
                             rawTdoasCertitude[antennaPair] * slopeConfiance;
                if (!isnan(pdoaProducedValue[antennaPair][pdSlopeId]) && !isnan(pond)) {
                    angleSum += pdoaProducedValue[antennaPair][pdSlopeId] * pond;
                    pondSum += pond;
                }
            }
        }
        if (!isnan(pondSum)) {
            pairResult[antennaPair][0] = angleSum / pondSum;
            pairResult[antennaPair][1] = pondSum;
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

std::optional<float> getFinalAngle(
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

std::optional<float> AngleCalculator::calculateAngle(BspInterlocRawAngleData& rawData) {
    if (rawData.m_framesLength < MINIMUM_ANGLE_MEAN || !m_parametersValid) {
        return {};
    }

    std::array<float, NUM_ANTENNA_PAIRS> rawTdoas{};
    std::array<float, NUM_ANTENNA_PAIRS> rawTdoasCertitude{};
    std::array<float, NUM_ANTENNA_PAIRS> rawPdoas{};

    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS> tdoaProducedValue;
    std::array<float, NUM_ANTENNA_PAIRS> fallingSlopeCertitude;
    std::array<float, NUM_ANTENNA_PAIRS> risingSlopeCertitude;

    std::array<std::array<float, NUM_PDOA_SLOPES << 1>, NUM_ANTENNA_PAIRS> pdoaProducedValue;
    std::array<std::array<float, NUM_PDOA_SLOPES << 1>, NUM_ANTENNA_PAIRS> pdoaCertitude;
    std::array<std::array<float, 2>, NUM_ANTENNA_PAIRS> pairResult;

    for (unsigned int i = 0; i < NUM_ANTENNA_PAIRS; i++) {
        rawTdoas[i] = getRawTdoa(rawData, i, -1);
        rawTdoasCertitude[i] = getTdoaValueCertitude(rawTdoas[i]);
        rawPdoas[i] = getRawPdoa(rawData, i, 1); // TODO: Decide if we want the mean of
        getPdoaValueCertiture(m_calculatorParameters, rawTdoas[i], rawPdoas[i], i,
                              tdoaProducedValue, pdoaProducedValue, pdoaCertitude);
    }

    getDecisionCertitude(rawTdoas, fallingSlopeCertitude, risingSlopeCertitude,
                         m_calculatorParameters);

    getPairAngle(pairResult, rawTdoasCertitude, pdoaProducedValue, pdoaCertitude,
                 fallingSlopeCertitude, risingSlopeCertitude);
    return getFinalAngle(pairResult);
}

float AngleCalculator::getRawTdoa(BspInterlocRawAngleData& rawData,
                                  uint8_t antennaPair,
                                  int32_t meanLength) {
    volatile float angleAccumulator = 0;
    uint32_t length = (meanLength <= 0) ? rawData.m_framesLength : (uint32_t)meanLength;

    const auto& antennaIds = m_calculatorParameters.m_antennaPairs[antennaPair];
    for (uint32_t i = 0; i < length; i++) {
        volatile int64_t timeDiff =
            (int64_t)(rawData.m_frames[i].m_frameInfos[antennaIds[0]].m_rxTimestamp -

                      rawData.m_frames[i].m_frameInfos[antennaIds[1]].m_rxTimestamp);

        volatile float tdoaDist = (float)timeDiff / UUS_TO_DWT_TIME * 299.792458;
        tdoaDist /= m_calculatorParameters.m_tdoaNormalizationFactors[antennaPair];

        if (tdoaDist > 1) {
            tdoaDist = 1;
        } else if (tdoaDist < -1) {
            tdoaDist = -1;
        }

        angleAccumulator += asin(tdoaDist) * 180 / M_PI;
    }
    float angle = angleAccumulator / length;
    while (angle > (float)360) {
        angle -= (float)360;
    }
    while (angle < -(float)360) {
        angle += (float)360;
    }
    return angle;
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
