#ifndef HIVE_MIND_ANGLECALCULATOR_H
#define HIVE_MIND_ANGLECALCULATOR_H

#include <bsp/AngleCalculatorParameters.h>
#include <bsp/BspInterlocAngleRawData.h>
#include <logger/ILogger.h>
#include <optional>

#define CONFUSION_LINE_DEGREES (30.0)
#define MINIMUM_ANGLE_MEAN (1)

/**
 * Helper class used to compute an angle from raw phase and timestamp values.
 * Should be initialized with setCalculatorParameters() before using.
 */
class AngleCalculator {
  public:
    AngleCalculator(ILogger& logger);

    void setCalculatorParameters(const AngleCalculatorParameters& parameters);

    std::tuple<std::optional<float>, std::optional<float>> calculateAngle(
        BspInterlocRawAngleData& rawData);

  private:
    AngleCalculatorParameters m_calculatorParameters;
    ILogger& m_logger;

    uint32_t m_allBetweenConfusionLinesErrors = 0;
    uint32_t m_conflictingSlopeDecisionsErrors = 0;

    bool m_parametersValid = false;

    float getRawPdoa(
        BspInterlocRawAngleData& rawData,
        std::array<std::array<float, NUM_ANTENNA_PAIRS>, MAX_ANGLE_FRAMES>& losConfidence,
        uint8_t antennaPair,
        int32_t meanLength);

    void computeLineOfSight(
        BspInterlocRawAngleData& rawData,
        std::array<std::array<float, NUM_ANTENNA_PAIRS>, MAX_ANGLE_FRAMES>& frameLosConfidence,
        std::array<float, NUM_ANTENNA_PAIRS>& meanLosConfidence);

    float fitPdoa(float pdValue, uint8_t pdSlopeId, uint8_t antennaPair);

    static void getPairAngle(
        std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS>& pairResult,
        std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS>& pdoaProducedValue,
        std::array<float, NUM_ANTENNA_PAIRS>& rawPdoaCertitude,
        std::array<float, NUM_ANTENNA_PAIRS>& fallingSlopeCertitude,
        std::array<float, NUM_ANTENNA_PAIRS>& risingSlopeCertitude,
        std::array<float, NUM_ANTENNA_PAIRS>& meanLosConfidence);

    static std::optional<float> getFinalAngle(
        std::array<std::array<float, 2>, NUM_ANTENNA_PAIRS>& pairResult);
};

#endif // HIVE_MIND_ANGLECALCULATOR_H
