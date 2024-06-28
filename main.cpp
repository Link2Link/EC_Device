/****************************************************************************/

#include <iostream>
#include "ECAT_Device.hpp"
#include "Beckhoff/Beckhoff.hpp"



ECDevice::Master master("/log/EC_Device.log");
ECDevice::Beckhoff_EK1100 bh_ek1100;
ECDevice::Beckhoff_EL1008 bh_el1008;

unsigned int control_frequency = 1000; // Hz

/****************************************************************************/

void ctrl_c_handler(int s)
{
    std::cout << "exiting" << std::endl;
    master.stop();
}

/****************************************************************************/

unsigned int loop_counter = 0;
void control_callback()
{
    if (loop_counter%500==0){


        std::cout << "value : ";
        for (int k = 0; k < 8; ++k)
            std::cout << bh_el1008.read_data_[k] << " ";
        std::cout << std::endl;

    }

    ++loop_counter;
}

/****************************************************************************/

int main(int argc, char **argv)
{
    master.printInfo("EC_Device start");


    master.setCtrlCHandler(ctrl_c_handler);

    master.addSlave(0,0,&bh_ek1100);
    master.addSlave(0,1,&bh_el1008);

//    master.setThreadHighPriority();
    master.setThreadRealTime();

    master.activate();

    master.run(control_callback, control_frequency);



    master.printInfo(fmt::format("run time : {}", master.elapsedTime()));
    master.printInfo(fmt::format("updates : {}", master.elapsedCycles()));
    master.printInfo(fmt::format("frequency : {}", master.elapsedCycles()/master.elapsedTime()));

    return 0;
}

/****************************************************************************/
