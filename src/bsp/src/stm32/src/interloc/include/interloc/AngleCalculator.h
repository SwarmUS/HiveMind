#ifndef HIVE_MIND_ANGLECALCULATOR_H
#define HIVE_MIND_ANGLECALCULATOR_H

#include <array>
#include <bsp/BspInterlocAngleRawData.h>
#include <logger/ILogger.h>
#include <optional>
#include <tuple>

#define NUM_ANTENNA_PAIRS (3)
#define NUM_TDOA_SLOPES (2)
#define NUM_PDOA_SLOPES (12)
#define CONFUSION_LINE_DEGREES (30.0)
#define MINIMUM_ANGLE_MEAN (30)

/**
 * Structure containing all parameters used to calculate an angle. These parameters are calculated
 * using the Python tooling during calibration and then transferred to the board.
 */
struct AngleCalculatorParameters {
    /**
     * Normalization factors used to stretch TDOA angles between -90 and +90 degrees. (Applied by
     * dividing by the normalization before applying the asin() on the TDOA)
     */
    std::array<float, NUM_ANTENNA_PAIRS> m_tdoaNormalizationFactors{};
    /**
     * Slopes (a in y=ax+b) of the non-reciprocated TDOA functions (calculatedAngle = a*realAngle+b)
     */
    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS> m_tdoaSlopes{};
    /**
     * Intercepts (b in y=ax+b) of the non-reciprocated TDOA functions
     * (calculatedAngle = a*realAngle+b)
     */
    std::array<std::array<float, NUM_TDOA_SLOPES>, NUM_ANTENNA_PAIRS> m_tdoaIntercepts{};

    /**
     * Normalization factors used to stretch PDOA angles between -90 and +90 degrees. (Applied by
     * dividing by the normalization after applying the asin() on the PDOA)
     */
    std::array<float, NUM_ANTENNA_PAIRS> m_pdoaNormalizationFactors{};
    /**
     * Slopes (a in y=ax+b) of the non-reciprocated PDOA functions (calculatedAngle = a*realAngle+b)
     * A single (positive) slope is used for all PDOA curves. The sign of the slope is applied at
     * runtime.
     */
    std::array<float, NUM_ANTENNA_PAIRS> m_pdoaSlopes{};
    /**
     * Intercepts (b in y=ax+b) of the non-reciprocated PDOA functions for every curve
     * (calculatedAngle = a*realAngle+b)
     */
    std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS> m_pdoaIntercepts{};
    /**
     * Origins (value for angle=-90 degrees) for every PDOA curve. Used to decide which curve to use
     */
    std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS> m_pdoaOrigins{};

    /**
     * Lookup table used to know which antenna IDs are used in each pair ID
     */
    std::array<std::array<uint8_t, 2>, NUM_ANTENNA_PAIRS> m_antennaPairs{};

    /**
     * Decision matrix used to choose between two slopes for the TDOA.
     *
     * The first index represents the pair ID for which we are choosing the slope while the second
     * index represents the slope ID (0 or 1) that should be used if the TDOA of the current index
     * pair is OVER the confusion line (+30 degrees). The opposite of this value is used when UNDER
     * the line.
     *
     * In summary, decisionMatrix[i][j] tells which slope index to use for antenna pair i when the
     * TDOA of antenna pair j is over 30 degrees.
     */
    std::array<std::array<uint8_t, NUM_ANTENNA_PAIRS>, NUM_ANTENNA_PAIRS> m_slopeDecisionMatrix{};
};

/**
 * Helper class used to compute an angle from raw phase and timestamp values.
 * Should be initialized with setCalculatorParameters() before using.
 */
class AngleCalculator {
  public:
    AngleCalculator(ILogger& logger);

    /**
     * Copies a set of parameters to internal storage. A copy is preferred to a reference as to
     * optimize placement in RAM an therefore calculation speed.
     * @param parameters Source parameters to copy
     */
    void setCalculatorParameters(const AngleCalculatorParameters& parameters);

    /**
     * Tries to calculate an angle from raw phase and timestamp values. If not enough data present
     * in array, or an error occurred in the algorithm, a false optional value is returned.
     * @param rawData Array of raw data to use for calculation
     * @return The angle if it could be computed, false otherwise
     */
    std::optional<float> calculateAngle(BspInterlocRawAngleData& rawData);

  private:
    AngleCalculatorParameters m_calculatorParameters;
    ILogger& m_logger;

    uint32_t m_allBetweenConfusionLinesErrors = 0;
    uint32_t m_conflictingSlopeDecisionsErrors = 0;

    /**
     * Computes a TDOA angle (without applying the linear fit) using raw timestamps
     * @param rawData Array with timestamps of messages
     * @param antennaPair ID of the antenna pair for which to compute the TDOA
     * @param meanLength Number of samples to use for the mean (-1 to use them all)
     * @return The mean angle value normalized between -90 and +90 degrees
     */
    float getRawTdoa(BspInterlocRawAngleData& rawData, uint8_t antennaPair, int32_t meanLength);

    /**
     * Retrieves the index of the slope that should be used for a given antenna pair considering
     * TDOA values calculated for every pair.
     * @param rawTdoas Array of raw TDOA angles calculated for every antenna pair
     * @param antennaPair ID of the pair for which we want the slope to use
     * @return -1 if an error occurred, ID of the slope otherwise.
     */
    int8_t getTdoaSlopeIndex(std::array<float, NUM_ANTENNA_PAIRS> rawTdoas, uint8_t antennaPair);

    /**
     * Applies the linear fit to the raw TDOA value to get a "real" angle from it
     * @param rawTdoa Raw TDOA value
     * @param tdoaSlopeIndex Index of the slope to use
     * @param antennaPair ID of the antenna pair
     * @return Angle value between 0 and 360 degrees
     */
    float getFittedTdoa(float rawTdoa, uint8_t tdoaSlopeIndex, uint8_t antennaPair);

    /**
     * Computes a PDOA angle (without applying the linear fit) using raw phase data
     * @param rawData Array with phase values
     * @param antennaPair ID of the antenna pair for which to compute the PDOA
     * @param meanLength Number of samples to use for the mean (-1 to use them all)
     * @return The mean PDOA value
     */
    float getRawPdoa(BspInterlocRawAngleData& rawData, uint8_t antennaPair, int32_t meanLength);

    /**
     * Applies the linear fit on a raw PDOA value using the associated TDOA fitted value
     * @param rawPdoa Raw PDOA value
     * @param fittedTdoa TDOA value after applying the linear fit
     * @param tdoaSlopeIndex Index of the TDOA slope used
     * @param antennaPair ID of the antenna pair
     * @return Angle value between 0 and 360 degrees
     */
    float getFittedPdoa(float rawPdoa,
                        float fittedTdoa,
                        uint8_t tdoaSlopeIndex,
                        uint8_t antennaPair);
};

#endif // HIVE_MIND_ANGLECALCULATOR_H
