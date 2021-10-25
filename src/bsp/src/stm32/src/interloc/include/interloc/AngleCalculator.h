#ifndef HIVE_MIND_ANGLECALCULATOR_H
#define HIVE_MIND_ANGLECALCULATOR_H

#include <array>
#include <bsp/BspInterlocAngleRawData.h>
#include <logger/ILogger.h>
#include <optional>
#include <tuple>

#define NUM_ANTENNA_PAIRS (3)
#define NUM_TDOA_SLOPES (2)
#define NUM_PDOA_SLOPES (10)
#define CONFUSION_LINE_DEGREES (30.0)
#define MINIMUM_ANGLE_MEAN (30)

/**
 * Structure containing all parameters used to calculate an angle. These parameters are calculated
 * using the Python tooling during calibration and then transferred to the board.
 */
struct AngleCalculatorParameters {
    std::array<float, NUM_ANTENNA_PAIRS> m_tdoaNormalizationFactors{};
    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS> m_tdoaSlopes{};
    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS> m_tdoaIntercepts{};

    std::array<float, NUM_ANTENNA_PAIRS> m_pdoaNormalizationFactors{};
    std::array<float, NUM_ANTENNA_PAIRS> m_pdoaSlopes{};
    std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS> m_pdoaIntercepts{};
    std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS> m_pdoaOrigins{};

    std::array<std::array<uint8_t, 2>, NUM_ANTENNA_PAIRS> m_antennaPairs{};
    std::array<std::array<uint8_t, NUM_ANTENNA_PAIRS>, NUM_ANTENNA_PAIRS> m_slopeDecisionMatrix{};
};

class AngleCalculator {
  public:
    AngleCalculator(ILogger& logger);

    void setCalculatorParameters(const AngleCalculatorParameters& parameters);
    std::optional<float> calculateAngle(BspInterlocRawAngleData& rawData);

  private:
    AngleCalculatorParameters m_calculatorParameters;
    ILogger& m_logger;

    uint32_t m_allBetweenConfusionLinesErrors = 0;
    uint32_t m_conflictingSlopeDecisionsErrors = 0;

    float getRawTdoa(BspInterlocRawAngleData& rawData, uint8_t antennaPair, int32_t meanLength);
    int8_t getTdoaSlopeIndex(std::array<float, NUM_ANTENNA_PAIRS> rawTdoas, uint8_t antennaPair);
    float getFittedTdoa(float rawTdoa, uint8_t tdoaSlopeIndex, uint8_t antennaPair);

    float getRawPdoa(BspInterlocRawAngleData& rawData, uint8_t antennaPair, int32_t meanLength);
    float getFittedPdoa(float rawPdoa,
                        float fittedTdoa,
                        uint8_t tdoaSlopeIndex,
                        uint8_t antennaPair);
};

#endif // HIVE_MIND_ANGLECALCULATOR_H
