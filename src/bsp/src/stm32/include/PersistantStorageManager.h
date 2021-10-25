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
     * Updates the angle calculator parameters both in FLASH and in the interloc runtime
     * @param parameters New parameters
     */
    void setAngleCalculatorParameters(const AngleCalculatorParameters& parameters);

  private:
    PersistedStorage m_storage __attribute__((aligned(4))){};
    ILogger& m_logger;

    bool setUUID(uint16_t uuid);
    bool saveToFlash();
};

#endif //__PERSISTEDSTORAGEMANAGER_H__
