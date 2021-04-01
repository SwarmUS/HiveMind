#ifndef __GREETHANDLER_H_
#define __GREETHANDLER_H_

#include "IGreetHandler.h"
#include <bsp/IBSP.h>
#include <pheromones/HiveMindHostDeserializer.h>
#include <pheromones/HiveMindHostSerializer.h>

class GreetHandler : public IGreetHandler {
  public:
    GreetHandler(IHiveMindHostSerializer& serializer,
                 IHiveMindHostDeserializer& deserializer,
                 IBSP& bsp);

    ~GreetHandler() override = default;

    bool greet() override;

  private:
    IHiveMindHostSerializer& m_serializer;
    IHiveMindHostDeserializer& m_deserializer;
    IBSP& m_bsp;
};

#endif // __GREETHANDLER_H_
