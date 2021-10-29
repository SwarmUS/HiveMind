#ifndef __PERSISTEDSTORAGEMANAGER_H__
#define __PERSISTEDSTORAGEMANAGER_H__

#include "PersistantStorage.h"
#include <logger/ILogger.h>

class PersistantStorageManager {
  public:
    PersistantStorageManager(ILogger& logger);
    virtual ~PersistantStorageManager() = default;

    /**
     * @brief Loads all storage from flash into RAM
     */
    void loadFromFlash();

    /**
     * @brief Returns the board UUID
     * @return The UUID
     */
    uint16_t getUUID() const;

    /**
     * Returns a reference to the angle calculator parameters
     * @return Reference
     */
    AngleCalculatorParameters& getAngleCaculatorParameters();

    /**
     * Saves the storage back to flash
     * @return True if successfull, false otherwise.
     */
    bool saveToFlash();

  private:
    __attribute__((packed, aligned(8)))
    PersistedStorage m_storage {};

    ILogger& m_logger;

    bool setUUID(uint16_t uuid);
};

#endif //__PERSISTEDSTORAGEMANAGER_H__
