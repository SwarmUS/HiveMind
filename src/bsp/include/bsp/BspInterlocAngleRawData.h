#ifndef HIVE_MIND_BSPINTERLOCANGLERAWDATA_H
#define HIVE_MIND_BSPINTERLOCANGLERAWDATA_H

#include <array>
#include <cstdint>

#define MAX_ANGLE_FRAMES 100
#define MAX_BEEBOARDS 6

struct BspInterlocFrameInfo {
    uint32_t m_beeboardPort;
    uint64_t m_rxTimestamp;
    float m_sfdAngle;
    float m_accumulatorAngle;
    uint32_t m_messageId;
};

struct BspInterlocFrameAngleRawData {
    uint8_t m_frameInfosLength;
    std::array<BspInterlocFrameInfo, MAX_BEEBOARDS> m_frameInfos;
};

struct BspInterlocRawAngleData {
    // TODO: Could add iterator functions to simplify reading the array
    uint32_t m_framesLength;
    std::array<BspInterlocFrameAngleRawData, MAX_ANGLE_FRAMES> m_frames;
};

#endif // HIVE_MIND_BSPINTERLOCANGLERAWDATA_H
