#ifndef __DTOMATCHERS_H_
#define __DTOMATCHERS_H_

#include <gmock/gmock.h>
#include <pheromones/MessageDTO.h>

// Simple matcher to check that DTO matches inside function call. If required
MATCHER_P(MessageGreetingDTOMatcher, expectedId, "Equality matcher for NetworkApiDTO)") {
    auto greet = std::get<GreetingDTO>(arg.getMessage());
    return greet.getId() == expectedId;
}

#endif // __DTOMATCHERS_H_
