#ifndef HIVE_MIND_TWOWAYRANGING_H
#define HIVE_MIND_TWOWAYRANGING_H

#include "UWBMessages.h"
#include "UWBRxFrame.h"
#include <cstdint>
#include <optional>

/**
 * @brief Class containing all information needed to calculate a distance using the TWR algorithm
 */
class TwoWayRanging {
  public:
    /**
     * @brief Calculates a distance based on the values of the class members
     * @param slotId The slot ID of the current device
     * @return The distance in meters
     */
    std::optional<double> calculateDistance(uint16_t slotId) const;

    /**
     * @brief Extracts all timestamps from a Final message
     * @param finalMessage A pointer to the message
     */
    void deserializeFinal(UWBMessages::TWRFinal* finalMessage);

    /**
     * @brief Constructs a final message based on the values of the class members and the given Tx
     * timestamp
     * @param finalMessage A pointer to the buffer to fill
     * @param finalTxTs The timestamp at which the final message will be sent
     */
    void constructFinal(UWBMessages::TWRFinal* finalMessage, uint64_t finalTxTs);

    uint64_t m_pollTxTs;
    uint64_t m_pollRxTs;

    uint64_t m_responseTxTs;
    uint64_t m_responseRxTs[MAX_INTERLOC_SUBFRAMES];

    uint64_t m_finalTxTs;
    uint64_t m_finalRxTs;
};

#endif // HIVE_MIND_TWOWAYRANGING_H
