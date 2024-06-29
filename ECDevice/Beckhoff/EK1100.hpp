#ifndef SIMPLECAT_BECKHOFF_EK1100_H_
#define SIMPLECAT_BECKHOFF_EK1100_H_


#include <utility>

#include "ECAT_Device.hpp"


namespace ECDevice {


class Beckhoff_EK1100 : public Slave
{
public:
    explicit Beckhoff_EK1100(std::string name="Beckhoff_EK1100") : Slave(0x00000002, 0x044c2c52, std::move(name)) {}
    ~Beckhoff_EK1100() override = default;
};


}

#endif
