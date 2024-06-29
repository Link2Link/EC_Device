/*
 * EL1124
 *
 * */
#ifndef ECDEVICE_EL1124_HPP
#define ECDEVICE_EL1124_HPP

#include <utility>

#include "ECAT_Device.hpp"

namespace ECDevice {


    class Beckhoff_EL1124 : public Slave
    {

    public:
        explicit Beckhoff_EL1124(std::string name) : Slave(0x00000002, 0x04643052, std::move(name)) {}
        ~Beckhoff_EL1124() override = default;

        void processData(size_t index, uint8_t* domain_address) override{
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

        // digital input values
        uint8_t read_data_[4] = {};

    private:
        ec_pdo_entry_info_t channels_[4] = {
                {0x6000, 0x01, 1}, /* Input */
                {0x6010, 0x01, 1}, /* Input */
                {0x6020, 0x01, 1}, /* Input */
                {0x6030, 0x01, 1}, /* Input */
        };

        ec_pdo_info_t pdos_[4] = {
                {0x1a00, 1, channels_ + 0}, /* Channel 1 */
                {0x1a01, 1, channels_ + 1}, /* Channel 2 */
                {0x1a02, 1, channels_ + 2}, /* Channel 3 */
                {0x1a03, 1, channels_ + 3}, /* Channel 4 */
        };

        ec_sync_info_t syncs_[2] = {
                {0, EC_DIR_INPUT, 4, pdos_ + 0, EC_WD_ENABLE},
                {0xff}
        };

        DomainMap domains_ = {
                {0, {0,1,2,3} }
        };
    };


}

#endif //ECDEVICE_EL1124_HPP
