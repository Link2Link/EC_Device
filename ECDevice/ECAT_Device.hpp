#ifndef ECDEVICE_ECAT_DEVICE_HPP
#define ECDEVICE_ECAT_DEVICE_HPP


#include "ecrt.h"
#include <map>
#include <vector>
#include <ctime>
#include <string>
#include <chrono>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <sys/resource.h>
#include <pthread.h>
#include <sched.h>
#include <csignal>
#include <sys/mman.h>
#include <cstring>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>

namespace ECDevice {

    // 从站设备基类
    class Slave
    {
    public:

        Slave(uint32_t vendor_id, uint32_t product_id) :
                vendor_id_(vendor_id),
                product_id_(product_id) {}

        virtual ~Slave() {}

        /** read or write data to the domain */
        virtual void processData(size_t index, uint8_t* domain_address){
            //read/write macro needs to match data type. e.g.
            //read_data_[index] = EC_READ_U8(domain_address)
            //EC_WRITE_S16(domain_address, write_data_[index]);
        }

        /** a pointer to syncs. return &syncs[0] */
        virtual const ec_sync_info_t* syncs() {
            //return &syncs_[0];
            return nullptr;
        }

        /** number of elements in the syncs array. */
        virtual size_t syncSize() {
            //return sizeof(syncs_)/sizeof(ec_sync_info_t);
            return 0;
        }

        /** a pointer to all PDO entries */
        virtual const ec_pdo_entry_info_t* channels() {
            //return channels_;
            return nullptr;
        }

        /** a map from domain index to pdo indices in that domain.
         *  map<domain index, vector<channels_ indices> > */
        typedef std::map<unsigned int, std::vector<unsigned int> > DomainMap;

        virtual void domains(DomainMap& domains) const {
            //domains = domains;
        }

        const uint32_t vendor_id_;
        const uint32_t product_id_;

        // //array to store the data to be read or sent
        //uint8_t               read_data_[E]; //example
        //int16_t               write_data_[F]; //example

    protected:
        // //see SETUP_ETHERLAB.md for explanation
        //ec_pdo_entry_info_t   channels_[A];
        //ec_pdo_info_t         pdos_[B];
        //ec_sync_info_t        syncs_[C];
        //ec_pdo_entry_info_t   domain_regs_[D];

        //DomainMap domains_;
    };

    class Master
    {
    public:

        /** true if running */
        volatile bool running_ = false;

        /** start and current time */
        std::chrono::time_point<std::chrono::system_clock> start_t_, curr_t_;

        // EtherCAT Control

        /** register a domain of the slave */
        struct DomainInfo;
        void registerPDOInDomain(uint16_t alias, uint16_t position,
                                 std::vector<unsigned int>& channel_indices,
                                 DomainInfo* domain_info,
                                 Slave* slave)
        {
            // expand the size of the domain
            unsigned int num_pdo_regs = channel_indices.size();
            size_t start_index = domain_info->domain_regs.size()-1; //empty element at end
            domain_info->domain_regs.resize(domain_info->domain_regs.size()+num_pdo_regs);

            // create a new entry in the domain
            DomainInfo::Entry domain_entry;
            domain_entry.slave        = slave;
            domain_entry.num_pdos     = num_pdo_regs;
            domain_entry.offset       = new unsigned int[num_pdo_regs];
            domain_entry.bit_position = new unsigned int[num_pdo_regs];
            domain_info->entries.push_back(domain_entry);

            Slave::DomainMap domain_map;
            slave->domains(domain_map);

            // add to array of pdos registrations
            const ec_pdo_entry_info_t* pdo_regs = slave->channels();
            for (size_t i=0; i<num_pdo_regs; ++i)
            {
                // create pdo entry in the domain
                ec_pdo_entry_reg_t& pdo_reg = domain_info->domain_regs[start_index+i];
                pdo_reg.alias       = alias;
                pdo_reg.position    = position;
                pdo_reg.vendor_id   = slave->vendor_id_;
                pdo_reg.product_code= slave->product_id_;
                pdo_reg.index       = pdo_regs[channel_indices[i]].index;
                pdo_reg.subindex    = pdo_regs[channel_indices[i]].subindex;
                pdo_reg.offset      = &(domain_entry.offset[i]);
                pdo_reg.bit_position= &(domain_entry.bit_position[i]);


                // print the domain pdo entry

                printInfo((std::ostringstream()
                << "register domain entry {" << pdo_reg.alias <<", "<< pdo_reg.position
                << ", 0x" << std::hex << pdo_reg.vendor_id
                << ", 0x" << std::hex << pdo_reg.product_code
                << ", 0x" << std::hex << pdo_reg.index
                << ", 0x" << std::hex << (int)pdo_reg.subindex
                << "}").str());

            }

            // set the last element to null
            ec_pdo_entry_reg_t empty = {0};
            domain_info->domain_regs.back() = empty;
        }

        /** check for change in the domain state */
        void checkDomainState(unsigned int domain)
        {
            DomainInfo* domain_info = domain_info_[domain];

            ec_domain_state_t ds;
            ecrt_domain_state(domain_info->domain, &ds);

            if (ds.working_counter != domain_info->domain_state.working_counter){
                printInfo(fmt::format("Domain: WC %u.", ds.working_counter));
            }
            if (ds.wc_state != domain_info->domain_state.wc_state){
                printInfo(fmt::format("Domain: State %u.", ds.wc_state));
            }
            domain_info->domain_state = ds;
        }

        /** check for change in the master state */
        void checkMasterState()
        {
            char message[64] = {};
            ec_master_state_t ms;
            ecrt_master_state(master_, &ms);

            if (ms.slaves_responding != master_state_.slaves_responding){
                sprintf(message, "%u slave(s).", ms.slaves_responding);
                printInfo(message);
            }
            if (ms.al_states != master_state_.al_states){
                sprintf(message, "Master AL states: 0x%02X.", ms.al_states);
                printInfo(message);
            }
            if (ms.link_up != master_state_.link_up){
                sprintf(message, "Link is %s.", ms.link_up ? "up" : "down");
                printInfo(message);
            }
            master_state_ = ms;
        }

        /** check for change in the slave states */
        void checkSlaveStates()
        {
            char message[64] = {};
            for (SlaveInfo& slave : slave_info_)
            {
                memset(message, 0, sizeof(message)/sizeof(char));
                ec_slave_config_state_t s;
                ecrt_slave_config_state(slave.config, &s);

                if (s.al_state != slave.config_state.al_state){
                    //this spams the terminal at initialization.
                    sprintf(message, "Slave: State 0x%02X.", s.al_state);
                    printInfo(message);
                }
                if (s.online != slave.config_state.online){
                    sprintf(message, "Slave: %s.", s.online ? "online" : "offline");
                    printInfo(message);
                }
                if (s.operational != slave.config_state.operational){
                    sprintf(message, "Slave: %soperational.", s.operational ? "" : "Not ");
                    printInfo(message);
                }
                slave.config_state = s;
            }
        }

        static void printError(const std::string& message)
        {
            spdlog::get("console")->error(message);
            spdlog::get("file_logger")->error(message);
        }

        /** print warning message to terminal */
        static void printWarning(const std::string& message)
        {
            spdlog::get("console")->warn(message);
            spdlog::get("file_logger")->warn(message);
        }


        static void printInfo(const std::string& message)
        {
            spdlog::get("console")->info(message);
            spdlog::get("file_logger")->info(message);
        }

        /** EtherCAT master data */
        ec_master_t *master_ = nullptr;
        ec_master_state_t master_state_ = {};

        /** data for a single domain */
        struct DomainInfo
        {
            DomainInfo(ec_master_t* master)
            {
                domain = ecrt_master_create_domain(master);
                if (domain==nullptr){
                    printWarning("Failed to create domain");
                    return;
                }

                const ec_pdo_entry_reg_t empty = {0};
                domain_regs.push_back(empty);
            }

            ~DomainInfo()
            {
                for (Entry& entry : entries){
                    delete [] entry.offset;
                    delete [] entry.bit_position;
                }
            }

            ec_domain_t *domain = NULL;
            ec_domain_state_t domain_state = {};
            uint8_t *domain_pd = NULL;

            /** domain pdo registration array.
             *  do not modify after active(), or may invalidate */
            std::vector<ec_pdo_entry_reg_t> domain_regs;

            /** slave's pdo entries in the domain */
            struct Entry {
                Slave* slave               = NULL;
                int num_pdos               = 0;
                unsigned int* offset       = NULL;
                unsigned int* bit_position = NULL;
            };

            std::vector<Entry> entries;
        };

        /** map from domain index to domain info */
        std::map<unsigned int, DomainInfo*> domain_info_;

        /** data needed to check slave state */
        struct SlaveInfo {
            Slave*                  slave               = NULL;
            ec_slave_config_t*      config              = NULL;
            ec_slave_config_state_t config_state        = {0};
        };

        std::vector<SlaveInfo> slave_info_;

        /** counter of control loops */
        unsigned long long update_counter_ = 0;

        /** frequency to check for master or slave state change.
         *  state checked every frequency_ control loops */
        unsigned int check_state_frequency_ = 100;


    public:

        Master(const std::string & log_file_name = "EC_Device.log")
        {
            auto consol_logger = spdlog::stdout_logger_mt("console");
            auto file_logger = spdlog::rotating_logger_mt("file_logger", log_file_name, 1024 * 128, 5);

            spdlog::set_level(spdlog::level::debug);
            spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");

            master_ = ecrt_request_master(0);
            if (master_== nullptr) {
                printError("Failed to obtain master.");
                return;
            }
        }

        virtual ~Master()
        {
            for (SlaveInfo& slave : slave_info_){
                //
            }
            for (auto& domain : domain_info_){
                delete domain.second;
            }

            // 关闭和销毁日志记录器
            spdlog::drop("console");
            spdlog::drop("file_logger");
        }

        void addSlave(uint16_t alias, uint16_t position, Slave* slave)
        {
            // configure slave in master
            SlaveInfo slave_info;
            slave_info.slave = slave;
            slave_info.config = ecrt_master_slave_config(master_, alias, position,
                                                         slave->vendor_id_,
                                                         slave->product_id_);
            if (slave_info.config==NULL){
                printError("Add slave. Failed to get slave configuration.");
                return;
            }
            slave_info_.push_back(slave_info);

            // check if slave has pdos
            size_t num_syncs = slave->syncSize();
            const ec_sync_info_t* syncs = slave->syncs();
            if (num_syncs>0)
            {
                // configure pdos in slave
                int pdos_status = ecrt_slave_config_pdos(slave_info.config, num_syncs, syncs);
                if (pdos_status){
                    printError("Add slave. Failed to configure PDOs");
                    return;
                }
            } else {
                if (slave->product_id_ == 0x044c2c52 && slave->vendor_id_ == 0x00000002){} // EK1100 does not have sync
                else
                    printWarning("Add slave. Sync size is zero for "
                    + (std::ostringstream() << alias).str() + ":"
                    + (std::ostringstream() << position).str()
                    );
            }

            // check if slave registered any pdos for the domain
            Slave::DomainMap domain_map;
            slave->domains(domain_map);
            for (auto& iter : domain_map){

                // get the domain info, create if necessary
                unsigned int domain_index = iter.first;
                DomainInfo* domain_info = domain_info_[domain_index];
                if (domain_info== nullptr){
                    domain_info = new DomainInfo(master_);
                    domain_info_[domain_index] = domain_info;
                }

                registerPDOInDomain(alias, position,
                                    iter.second, domain_info,
                                    slave);
            }
        }

        void activate()
        {
            // register domain
            for (auto& iter : domain_info_){
                DomainInfo* domain_info = iter.second;
                bool domain_status = ecrt_domain_reg_pdo_entry_list(
                        domain_info->domain,
                        &(domain_info->domain_regs[0]));
                if (domain_status){
                    printError("Activate. Failed to register domain PDO entries.");
                    return;
                }
            }
            // activate master
            bool activate_status = ecrt_master_activate(master_);
            if (activate_status){
                printError("Activate. Failed to activate master.");
                return;
            }

            // retrieve domain data
            for (auto& iter : domain_info_){
                DomainInfo* domain_info = iter.second;
                domain_info->domain_pd = ecrt_domain_data(domain_info->domain);
                if (domain_info->domain_pd== nullptr){
                    printError("Activate. Failed to retrieve domain process data.");
                    return;
                }
            }
        }

        virtual void update(unsigned int domain = 0)
        {
            // receive process data
            ecrt_master_receive(master_);

            DomainInfo* domain_info = domain_info_[domain];

            ecrt_domain_process(domain_info->domain);

            // check process data state (optional)
            checkDomainState(domain);

            // check for master and slave state change
            if (update_counter_ % check_state_frequency_ == 0){
                checkMasterState();
                checkSlaveStates();
            }

            // read and write process data
            for (DomainInfo::Entry& entry : domain_info->entries){
                for (int i=0; i<entry.num_pdos; ++i){
                    (entry.slave)->processData(i, domain_info->domain_pd + entry.offset[i]);
                }
            }

            // send process data
            ecrt_domain_queue(domain_info->domain);
            ecrt_master_send(master_);

            ++update_counter_;
        }

        typedef void (*ECAT_CONTRL_CALLBACK)(void);
        virtual void run(ECAT_CONTRL_CALLBACK user_callback, double frequency)
        {
            spdlog::get("console")->info("Running loop at [%.1f] Hz", frequency);
            spdlog::get("file_logger")->info("Running loop at [%.1f] Hz", frequency);

            unsigned int interval = 1000000000.0/frequency;

            // start after one second
            struct timespec t;
            clock_gettime(CLOCK_MONOTONIC ,&t);
            t.tv_sec++;

            running_ = true;
            start_t_ = std::chrono::system_clock::now();
            while(running_)
            {
                // wait until next shot
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

                // update EtherCAT bus
                this->update();

                // get actual time
                curr_t_ = std::chrono::system_clock::now();

                // user callback
                user_callback();

                // calculate next shot. carry over nanoseconds into microseconds.
                t.tv_nsec += interval;
                while (t.tv_nsec >= 1000000000){
                    t.tv_nsec -= 1000000000;
                    t.tv_sec++;
                }
            }
        }

        /** stop the control loop. use within callback, or from a seperate thread. */
        virtual void stop() {running_ = false;}

        /** time of last ethercat update, since calling run. stops if stop called.
     *  returns actual time. use elapsedCycles()/frequency for discrete time at last update. */
        virtual double elapsedTime()
        {
            std::chrono::duration<double> elapsed_seconds = curr_t_ - start_t_;
            return elapsed_seconds.count()-1.0; // started after 1 second
        }

        /** number of EtherCAT updates since calling run. */
        virtual unsigned long long elapsedCycles()
        {
            return update_counter_;
        }

        /** add ctr-c exit callback.
            * default exits the run loop and prints timing */
        typedef void (*ECDEVICE_EXIT_CALLBACK)(int);
        static void setCtrlCHandler(ECDEVICE_EXIT_CALLBACK user_callback = NULL)
        {
            // ctrl c handler
            struct sigaction sigIntHandler;
            sigIntHandler.sa_handler = user_callback;
            sigemptyset(&sigIntHandler.sa_mask);
            sigIntHandler.sa_flags = 0;
            sigaction(SIGINT, &sigIntHandler, NULL);
        }

        /** set the thread to a priority of -19
            *  priority range is -20 (highest) to 19 (lowest) */
        static void setThreadHighPriority()
        {
            pid_t pid = getpid();
            int priority_status = setpriority(PRIO_PROCESS, pid, -19);
            if (priority_status){
                printWarning("setThreadHighPriority. Failed to set priority.");
                return;
            }
        }

        /** set the thread to real time (FIFO)
         *  thread cannot be preempted.
         *  set priority as 49 (kernel and interrupts are 50) */
        static void setThreadRealTime()
        {
            /* Declare ourself as a real time task, priority 49.
               PRREMPT_RT uses priority 50
               for kernel tasklets and interrupt handler by default */
            struct sched_param param;
            param.sched_priority = 49;
            //pthread_t this_thread = pthread_self();
            if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
                printError("sched_setscheduler failed");
                exit(-1);
            }

            /* Lock memory */
            if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
                printError("mlockall failed");
                exit(-2);
            }

            /* Pre-fault our stack
               8*1024 is the maximum stack size
               which is guaranteed safe to access without faulting */
            int MAX_SAFE_STACK = 8*1024;
            unsigned char dummy[MAX_SAFE_STACK];
            memset(dummy, 0, MAX_SAFE_STACK);
        }

    };


}


#endif //ECDEVICE_ECAT_DEVICE_HPP
