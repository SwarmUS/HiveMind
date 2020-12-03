#include "bsp/bsp_info.h"

namespace BSPInfo {
    // TODO: Store in EEPROM with a compiler flag to overwrite or something like that
    uint16_t get_id() { return 1; }
} // namespace BSPInfo
