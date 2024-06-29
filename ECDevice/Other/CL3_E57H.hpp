#ifndef ECDEVICE_CL3_E57H_HPP
#define ECDEVICE_CL3_E57H_HPP


#include <utility>

#include "ECAT_Device.hpp"

namespace ECDevice {


    class CL3_E57H : public Slave
    {

    public:
        explicit CL3_E57H(std::string name = "CL3_E57H") : Slave(0x00000a79, 0x00001000, std::move(name)) {}
        ~CL3_E57H() override = default;

        //当驱动器使能时返回true
        bool initialized() const {return initialized_;}



        virtual void processData(size_t index, uint8_t* domain_address){
            switch (index)
            {
                case 0:
                    control_word_ = EC_READ_U16(domain_address);
                    control_word_ = transition(state_, control_word_);
                    EC_WRITE_U16(domain_address, control_word_);
                    break;
                case 1:
                    EC_WRITE_S32(domain_address, target_position_);
                    break;
                case 2:
                    EC_WRITE_S32(domain_address, target_velocity_);
                    break;
                case 3:
                    EC_WRITE_S8(domain_address, mode_of_operation_);
                    break;
                case 4:
                    status_word_ = EC_READ_U16(domain_address);
                    state_ = deviceState(status_word_);
                    break;
                case 5:
                    position_ = EC_READ_S32(domain_address);
                    break;
                case 6:
                    velocity_ = EC_READ_S32(domain_address);
                    break;
                case 7:
                    mode_of_operation_display_ = EC_READ_S8(domain_address);
                    break;
                default:
                    std::cout << "WARNING. CL3_E57H pdo index out of range." << std::endl;
            }

            // CHECK FOR STATE CHANGE
            if (index == 4)
            {
                if (status_word_ != last_status_word_){
                    state_ = deviceState(status_word_);
                    // status word change does not necessarily mean state change
                    // http://ftp.beckhoff.com/download/document/motion/ax2x00_can_manual_en.pdf
                    // std::bitset<16> temp(status_word_);
                    // std::cout << "STATUS WORD: " << temp << std::endl;
                    if (state_ != last_state_){
                        Master::printInfo(fmt::format("{} STATE : {}", name, device_state_str_[state_]));
                    }
                }
                if ((state_ == STATE_OPERATION_ENABLED)&&
                    (last_state_ == STATE_OPERATION_ENABLED)){
                    initialized_ = true;
                } else {
                    initialized_ = false;
                }
                last_status_word_ = status_word_;
                last_state_ = state_;
            }
        }

        virtual const ec_sync_info_t* syncs() { return &syncs_[0]; }

        virtual size_t syncSize() {
            return sizeof(syncs_)/sizeof(ec_sync_info_t);
        }

        virtual const ec_pdo_entry_info_t* channels() {
            return channels_;
        }

        virtual void domains(DomainMap& domains) const {
            domains = domains_;
        }

        //array to store the data to be read or sent
        int16_t read_data_[8] = {0}; //example

        uint16_t control_word_              = 0; // write
        int32_t  target_position_           = 0; // write
        int32_t  target_velocity_           = 0; // write
        int8_t   mode_of_operation_         = 0; // write (use enum ModeOfOperation for convenience)

        uint16_t status_word_               = 0; // read
        int32_t  position_                  = 0; // read
        int32_t  velocity_                  = 0; // read
        int8_t   mode_of_operation_display_ = 0; // read


        enum ModeOfOperation
        {
            MODE_NO_MODE                = 0,
            MODE_PROFILED_POSITION      = 1,
            MODE_PROFILED_VELOCITY      = 3,
            MODE_PROFILED_TORQUE        = 4,
            MODE_HOMING                 = 6,
            MODE_INTERPOLATED_POSITION  = 7,
            MODE_CYCLIC_SYNC_POSITION   = 8,
            MODE_CYCLIC_SYNC_VELEOCITY  = 9,
            MODE_CYCLIC_SYNC_TORQUE     = 10
        };

    private:



        ec_pdo_entry_info_t channels_[11] = {
                {0x6040, 0x00, 16},             // 0(write) 控制字(0x6040) 在 OP 状态才被从站读取
                {0x607a, 0x00, 32},             // 1(write) target_position（607Ah)
                {0x60ff, 0x00, 32},             // 2(write) target_velocity（60FFh）
                {0x6060, 0x00, 8},              // 3(write) Mode of Operation (6060h)
                {0x0000, 0x00, 8}, /* Gap */
                {0x6041, 0x00, 16},             // 5(read) 状态字(0x6041)在 OP 状态与 Safe-Op 下才被更新传送
                {0x6064, 0x00, 32},             // 6(read) 实际位置(编码器值)
                {0x606c, 0x00, 32},             // 7(read) 实际速度
                {0x6061, 0x00, 8},              // 8(read) Modes of Operation Display
                {0x0000, 0x00, 8}, /* Gap */

        };

        ec_pdo_info_t pdos_[2] = {
                {0x1600, 5, channels_ + 0},
                {0x1a00, 5, channels_ + 5},
        };

        ec_sync_info_t syncs_[5] = {
                {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
                {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
                {2, EC_DIR_OUTPUT, 1, pdos_ + 0, EC_WD_ENABLE},
                {3, EC_DIR_INPUT, 1, pdos_ + 1, EC_WD_DISABLE},
                {0xff}
        };

        DomainMap domains_ = {
                {0, {0, 1, 2, 3, 5, 6, 7, 8, 10} }  // 这些添加到映射中
        };

        enum DeviceState
        {
            STATE_UNDEFINED = 0,
            STATE_START = 1,
            STATE_NOT_READY_TO_SWITCH_ON,
            STATE_SWITCH_ON_DISABLED,
            STATE_READY_TO_SWITCH_ON,
            STATE_SWITCH_ON,
            STATE_OPERATION_ENABLED,
            STATE_QUICK_STOP_ACTIVE,
            STATE_FAULT_REACTION_ACTIVE,
            STATE_FAULT
        };

        std::map<DeviceState,std::string> device_state_str_ = {
                {STATE_START,                  "Start"},
                {STATE_NOT_READY_TO_SWITCH_ON, "Not Ready to Switch On"},
                {STATE_SWITCH_ON_DISABLED,     "Switch on Disabled"},
                {STATE_READY_TO_SWITCH_ON,     "Ready to Switch On"},
                {STATE_SWITCH_ON,              "Switch On"},
                {STATE_OPERATION_ENABLED,      "Operation Enabled"},
                {STATE_QUICK_STOP_ACTIVE,      "Quick Stop Active"},
                {STATE_FAULT_REACTION_ACTIVE,  "Fault Reaction Active"},
                {STATE_FAULT,                  "Fault"}
        };

        DeviceState deviceState(uint16_t status_word)
        {
            if      ((status_word & 0b01001111) == 0b00000000){
                return STATE_NOT_READY_TO_SWITCH_ON;
            }
            else if ((status_word & 0b01001111) == 0b01000000){
                return STATE_SWITCH_ON_DISABLED;
            }
            else if ((status_word & 0b01101111) == 0b00100001){
                return STATE_READY_TO_SWITCH_ON;
            }
            else if ((status_word & 0b01101111) == 0b00100011){
                return STATE_SWITCH_ON;
            }
            else if ((status_word & 0b01101111) == 0b00100111){
                return STATE_OPERATION_ENABLED;
            }
            else if ((status_word & 0b01101111) == 0b00000111){
                return STATE_QUICK_STOP_ACTIVE;
            }
            else if ((status_word & 0b01001111) == 0b00001111){
                return STATE_FAULT_REACTION_ACTIVE;
            }
            else if ((status_word & 0b01001111) == 0b00001000){
                return STATE_FAULT;
            }
            return STATE_UNDEFINED;
        }

        /** returns the control word that will take device from state to next desired state */
        uint16_t transition(DeviceState state, uint16_t control_word)
        {
            switch(state)
            {
                case STATE_START:                   // -> STATE_NOT_READY_TO_SWITCH_ON (automatic)
                    return control_word;
                case STATE_NOT_READY_TO_SWITCH_ON:  // -> STATE_SWITCH_ON_DISABLED (automatic)
                    return control_word;
                case STATE_SWITCH_ON_DISABLED:      // -> STATE_READY_TO_SWITCH_ON
                    return ((control_word & 0b01111110) | 0b00000110);
                case STATE_READY_TO_SWITCH_ON:      // -> STATE_SWITCH_ON
                    return ((control_word & 0b01110111) | 0b00000111);
                case STATE_SWITCH_ON:               // -> STATE_OPERATION_ENABLED
                    return ((control_word & 0b01111111) | 0b00001111);
                case STATE_OPERATION_ENABLED:       // -> GOOD
                    return control_word;
                case STATE_QUICK_STOP_ACTIVE:       // -> STATE_OPERATION_ENABLED
                    return ((control_word & 0b01111111) | 0b00001111);
                case STATE_FAULT_REACTION_ACTIVE:   // -> STATE_FAULT (automatic)
                    return control_word;
                case STATE_FAULT:                   // -> STATE_SWITCH_ON_DISABLED
                    return ((control_word & 0b11111111) | 0b10000000);
                default:
                    break;
            }
            return control_word;
        }

        int last_status_word_ = -1;
        DeviceState last_state_ = STATE_START;
        DeviceState state_ = STATE_START;
        bool initialized_ = false;
    };



}



#endif //ECDEVICE_CL3_E57H_HPP
