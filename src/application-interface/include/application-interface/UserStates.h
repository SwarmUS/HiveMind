#ifndef USERSTATES_H_
#define USERSTATES_H_

enum class SevenSegment {
    Zero = 0,
    One = 1,
    Two = 2,
    Three = 3,
    Four = 4,
    Five = 5,
    Six = 6,
    Seven = 7,
    Eight = 8,
    Nine = 9,
    A = 0x0A,
    B = 0x0B,
    C = 0x0C,
    D = 0x0D,
    E = 0x0E,
    F = 0x0F,
};

struct UserStates {
    bool m_userLed;
    SevenSegment m_userSegment;
};

#endif // USERSTATES_H_
