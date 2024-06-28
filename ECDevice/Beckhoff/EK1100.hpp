#ifndef SIMPLECAT_BECKHOFF_EK1100_H_
#define SIMPLECAT_BECKHOFF_EK1100_H_


#include "ECAT_Device.hpp"


namespace ECDevice {


class Beckhoff_EK1100 : public Slave
{
public:
    Beckhoff_EK1100() : Slave(0x00000002, 0x044c2c52) {}
    virtual ~Beckhoff_EK1100() {}
};


}

#endif
