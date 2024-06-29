/****************************************************************************/

#include <iostream>
#include "ECAT_Device.hpp"
#include "Beckhoff/Beckhoff.hpp"
#include "AMX/AMX.hpp"
#include "Other/Other.hpp"

ECDevice::Master master("/log/EC_Device.log");
ECDevice::Beckhoff_EK1100 bh_ek1100;
ECDevice::Beckhoff_EL1008 bh_el1008;
ECDevice::AMX_EC2_IO8RA amx_ec2_io8ra;
ECDevice::CL3_E57H cl3_e57h("step_motor_1");

unsigned int control_frequency = 1000; // Hz

/****************************************************************************/

void ctrl_c_handler(int s)
{
    master.ctrl_c_exiting();
}

/****************************************************************************/

unsigned int loop_counter = 0;

uint8_t lastDI = 0;

const int encoder_resolution = 1024;
bool initialized = false;
int axis1_pos_offset = 0;
int axis1_last_pos = 0;


void control_callback()
{

    if (amx_ec2_io8ra.read_DI_ != lastDI)
        spdlog::get("console")->info(fmt::format("digital input : {}", amx_ec2_io8ra.read_DI_));
    lastDI = amx_ec2_io8ra.read_DI_;

    if (loop_counter%500==0){

    }

    // Intialize encoder offsets
    if (!initialized && cl3_e57h.initialized()){
        axis1_pos_offset = cl3_e57h.position_;
        axis1_last_pos = 0;
        initialized = true;

        std::cout << "================\n";
        std::cout << "initial position: " << cl3_e57h.position_ << '\n';
    }

    // Zeroed positions and velocities
    int axis1_pos = cl3_e57h.position_          - axis1_pos_offset;

    int axis1_vel = axis1_pos - axis1_last_pos;

    axis1_last_pos = axis1_pos;

    cl3_e57h.target_position_ = loop_counter;

    // print
    if (loop_counter%500==0)
    {
        std::cout << std::dec;
        std::cout << "axis 1  position : " << axis1_pos << '\n';
        std::cout << "axis 1  velocity : " << axis1_vel << '\n';
        std::cout << "target position :" << cl3_e57h.target_position_ << '\n';
        printf("current mode %d\n", cl3_e57h.mode_of_operation_display_);
        cl3_e57h.mode_of_operation_ = ECDevice::CL3_E57H::MODE_CYCLIC_SYNC_POSITION;
    }

    ++loop_counter;
}

/****************************************************************************/

int main(int argc, char **argv)
{
    ECDevice::Master::printInfo("EC_Device start");


    ECDevice::Master::setCtrlCHandler(ctrl_c_handler);

    master.addSlave(0,0,&bh_ek1100);
    master.addSlave(0,1,&bh_el1008);
    master.addSlave(0,2, &amx_ec2_io8ra);
    master.addSlave(0,3,&cl3_e57h);
//    master.setThreadHighPriority();
    ECDevice::Master::setThreadRealTime();

    master.activate();

    master.run(control_callback, control_frequency);



    ECDevice::Master::printInfo(fmt::format("run time : {}", master.elapsedTime()));
    ECDevice::Master::printInfo(fmt::format("updates : {}", master.elapsedCycles()));
    ECDevice::Master::printInfo(fmt::format("frequency : {}", master.elapsedCycles()/master.elapsedTime()));

    return 0;
}

/****************************************************************************/
