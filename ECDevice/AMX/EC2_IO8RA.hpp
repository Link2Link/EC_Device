#ifndef ECDEVICE_EC2_IO8RA_HPP
#define ECDEVICE_EC2_IO8RA_HPP


#include <utility>

#include "ECAT_Device.hpp"

namespace ECDevice {


    class AMX_EC2_IO8RA : public Slave
    {

    public:
        explicit AMX_EC2_IO8RA(std::string name="AMX_EC2_IO8RA") : Slave(0x00000b95, 0x00001003, std::move(name)) {}
        ~AMX_EC2_IO8RA() override = default;

        void processData(size_t index, uint8_t* domain_address) override{

            switch (index){
                case 0:
                    EC_WRITE_U8(domain_address, write_DO_);
                    break;
                case 1:
                    EC_WRITE_U16(domain_address, write_AO_[0]);
                    break;
                case 2:
                    EC_WRITE_U16(domain_address, write_AO_[1]);
                    break;
                case 3:
                    read_DI_ = EC_READ_U8(domain_address);
                    break;
                case 4:
                    read_AI_[0] = EC_READ_U16(domain_address);
                    break;
                case 5:
                    read_AI_[1] = EC_READ_U16(domain_address);
                    break;
                case 6:
                    read_AI_[2] = EC_READ_U16(domain_address);
                    break;
                case 7:
                    read_AI_[3] = EC_READ_U16(domain_address);
                    break;
                case 8:
                    read_AI_[4] = EC_READ_U16(domain_address);
                    break;
                case 9:
                    read_AI_[5] = EC_READ_U16(domain_address);
                    break;
                default: ;
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
        uint8_t write_DO_ = 0;
        uint8_t read_DI_ = 0;
        uint16_t write_AO_[2] = {};
        uint16_t read_AI_[6] = {};


    private:
        ec_pdo_entry_info_t channels_[12] = {
                {0x7000, 0x01, 8}, /* OUT_GEN_DO */         //0
                {0x0000, 0x00, 8}, /* Gap */
                {0x7020, 0x01, 16}, /* Analogoutput1 */     //1
                {0x7020, 0x02, 16}, /* Analogoutput2 */     //2
                {0x6000, 0x01, 8}, /* IN_GEN_DI */          //3
                {0x0000, 0x00, 8}, /* Gap */
                {0x6020, 0x01, 16}, /* Analoginput1 */      //4
                {0x6020, 0x02, 16}, /* Analoginput2 */      //5
                {0x6020, 0x03, 16}, /* Analoginput3 */      //6
                {0x6020, 0x04, 16}, /* Analoginput4 */      //7
                {0x6020, 0x05, 16}, /* Analoginput5 */      //8
                {0x6020, 0x06, 16}, /* Analoginput6 */      //9
        };

        ec_pdo_info_t pdos_[4] = {
                {0x1600, 2, channels_ + 0}, /* DOOutputs process data mapping */
                {0x1602, 2, channels_ + 2}, /* AOOutputs process data mapping */
                {0x1a00, 2, channels_ + 4}, /* DIInputs process data mapping */
                {0x1a02, 6, channels_ + 6}, /* AIInputs process data mapping */
        };

        ec_sync_info_t syncs_[5] = {
                {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
                {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
                {2, EC_DIR_OUTPUT, 2, pdos_ + 0, EC_WD_ENABLE},
                {3, EC_DIR_INPUT, 2, pdos_ + 2, EC_WD_DISABLE},
                {0xff}
        };

        DomainMap domains_ = {
                {0, {0, 2, 3, 4, 6, 7, 8, 9, 10, 11} }
        };
    };

}




#endif //ECDEVICE_EC2_IO8RA_HPP
