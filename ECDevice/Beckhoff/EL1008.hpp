#ifndef ECDEVICE_EL1008_HPP
#define ECDEVICE_EL1008_HPP

#include "ECAT_Device.hpp"

namespace ECDevice {


    class Beckhoff_EL1008 : public Slave
    {

    public:
        Beckhoff_EL1008() : Slave(0x00000002, 0x03f03052) {}
        virtual ~Beckhoff_EL1008() {}

        virtual void processData(size_t index, uint8_t* domain_address){
            read_data_[index] = EC_READ_U8(domain_address);
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

    private:
        ec_pdo_entry_info_t channels_[8] = {
                {0x6000, 0x01, 1}, /* Input */
                {0x6010, 0x01, 1}, /* Input */
                {0x6020, 0x01, 1}, /* Input */
                {0x6030, 0x01, 1}, /* Input */
                {0x6040, 0x01, 1}, /* Input */
                {0x6050, 0x01, 1}, /* Input */
                {0x6060, 0x01, 1}, /* Input */
                {0x6070, 0x01, 1}, /* Input */
        };

        ec_pdo_info_t pdos_[8] = {
                {0x1a00, 1, channels_ + 0}, /* Channel 1 */
                {0x1a01, 1, channels_ + 1}, /* Channel 2 */
                {0x1a02, 1, channels_ + 2}, /* Channel 3 */
                {0x1a03, 1, channels_ + 3}, /* Channel 4 */
                {0x1a04, 1, channels_ + 4}, /* Channel 5 */
                {0x1a05, 1, channels_ + 5}, /* Channel 6 */
                {0x1a06, 1, channels_ + 6}, /* Channel 7 */
                {0x1a07, 1, channels_ + 7}, /* Channel 8 */
        };

        ec_sync_info_t syncs_[2] = {
                {0, EC_DIR_INPUT, 8, pdos_ + 0, EC_WD_DISABLE},
                {0xff}
        };

        DomainMap domains_ = {
                {0, {0, 1, 2, 3, 4, 5, 6, 7} }
        };
    };

}

#endif //ECDEVICE_EL1008_HPP
