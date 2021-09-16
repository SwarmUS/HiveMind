#ifndef APPLICATIONSTATES_H_
#define APPLICATIONSTATES_H_

#include "SystemStates.h"
#include "UserStates.h"

struct ApplicationStates {
    SystemStates m_systemStates;
    UserStates m_userStates;
};

#endif // APPLICATIONSTATES_H_
