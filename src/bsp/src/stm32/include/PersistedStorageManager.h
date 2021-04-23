#ifndef __PERSISTEDSTORAGEMANAGER_H__
#define __PERSISTEDSTORAGEMANAGER_H__

#include "PersitedStorage.h"
#include <logger/ILogger.h>

class PersistedStorageManager {
  public:
    PersistedStorageManager(ILogger& logger);
    virtual ~PersistedStorageManager() = default;

    /**
     * @brief Loads all storage from flash into RAM
     */
    void loadFromFlash();

    /**
     * @brief Returns the board UUID
     * @return The UUID
     */
    uint16_t getUUID() const;

  private:
    PersistedStorage m_storage{};
    ILogger& m_logger;

    bool setUUID(uint16_t uuid);
    bool saveToFlash();
};

#endif //__PERSISTEDSTORAGEMANAGER_H__
