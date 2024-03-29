#ifndef HIVE_MIND_ANGLECALCULATORPARAMETERS_H
#define HIVE_MIND_ANGLECALCULATORPARAMETERS_H

#include <array>
#include <cstdint>

#define NUM_ANTENNA_PAIRS (3)
#define NUM_PDOA_SLOPES (2)

#define ANGLE_PARAMETERS_VALID_SECRET_NUMBER (42)

/**
 * Structure containing all parameters used to calculate an angle. These parameters are calculated
 * using the Python tooling during calibration and then transferred to the board.
 */
struct AngleCalculatorParameters {
    /**
     * Normalization factors used to stretch PDOA angles between -90 and +90 degrees. (Applied by
     * dividing by the normalization after applying the asin() on the PDOA)
     */
    std::array<float, NUM_ANTENNA_PAIRS> m_pdoaNormalizationFactors{};
    /**
     * Slopes (a in y=ax+b) of the non-reciprocated PDOA functions (calculatedAngle = a*realAngle+b)
     */
    std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS> m_pdoaSlopes{};
    /**
     * Intercepts (b in y=ax+b) of the non-reciprocated PDOA functions for every curve
     * (calculatedAngle = a*realAngle+b)
     */
    std::array<std::array<float, NUM_PDOA_SLOPES>, NUM_ANTENNA_PAIRS> m_pdoaIntercepts{};

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

    /**
     * Numbers used to verify if the data loaded from flash was written beforehand. If the numbers
     * equals the constant, we can consider everything is OK for a given pair.
     */
    std::array<uint8_t, NUM_ANTENNA_PAIRS> m_parametersValidSecretNumbers;

    /**
     * Offset (in degrees) of the BeeBoard assembly on the robot to allow placing it at any
     * orientation. Applied after interloc is calculated
     */
    float m_boardOrientationOffset;
};

#endif // HIVE_MIND_ANGLECALCULATORPARAMETERS_H
