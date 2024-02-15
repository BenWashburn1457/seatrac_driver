#include <iostream>
using namespace std;

#include <seatrac_driver/AsyncService.h>
#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/commands.h>
using namespace narval::seatrac;

narval::seatrac::messages::Status get_next_status(SeatracDriver& seatrac)
{
    narval::seatrac::messages::Status status;

    if(!seatrac.wait_for_message(CID_STATUS, &status, 1000)) {
        throw TimeoutReached();
    }

    return status;
}

class MyDriver : public narval::seatrac::SeatracDriver
{
    public:


    narval::seatrac::messages::PingResp lastResponse_;

    MyDriver(const IoServicePtr& ioService,
             const std::string& port = "/dev/narval_usbl") :
        narval::seatrac::SeatracDriver(ioService, port)
    {}

    void on_message(CID_E msgId, const std::vector<uint8_t>& msgData)
    {
        switch(msgId) {
            default:
                std::cout << "Got message : " << msgId << std::endl << std::flush;
                break;
            case CID_PING_ERROR:
                {
                    narval::seatrac::messages::PingError response;
                    response = msgData;
                    std::cout << response << std::endl;
                }
                break;
            case CID_PING_RESP:
                // std::cout << "Got a Ping Response" << std::endl << std::flush;
                {
                    //messages::PingResp response;
                    lastResponse_ = msgData;
                    std::cout << lastResponse_ << std::endl;
                }
                break;
            //case CID_STATUS:
            //    break;
        }
    }
};

int main(int argc, char** argv)
{
    const char* device = "/dev/ttyUSB0";
    if(argc > 1) device = argv[1];

    AsyncService service;
    MyDriver seatrac(service.io_service(), device);
    seatrac.enable_io_dump();

    service.start();



    //cout << "Type Enter for reboot" << endl;
    //getchar();
    //cout << command::sys_reboot(seatrac) << endl;

    //cout << "Type enter to configure status message" << endl;
    //getchar();
    //cout << command::status_config_set(seatrac,
    //    ENVIRONMENT | ATTITUDE | MAG_CAL | ACC_CAL | AHRS_RAW_DATA | AHRS_COMP_DATA,
    //    STATUS_MODE_1HZ 
    //) << endl;
    
    // // changing beacon id to 1
    // auto settings = command::settings_get(seatrac).settings;
    // cout << "Current settings : " << settings << endl;
    // cout << "Type enter to change settings" << endl;
    // getchar();
    // settings.xcvrBeaconId = BEACON_ID_1;
    // cout << command::settings_set(seatrac, settings) << endl;
    // settings = command::settings_get(seatrac).settings;
    // cout << "New settings : " << settings << endl;

    // cout << "Type enter to save settings" << endl;
    // getchar();
    // cout << command::settings_save(seatrac) << endl;
    // settings = command::settings_get(seatrac).settings;
    // cout << "New settings : " << settings << endl;

    // // reloading saved settings
    // cout << "Type enter to reload settings" << endl;
    // getchar();
    // cout << command::settings_load(seatrac) << endl;
    // settings = command::settings_get(seatrac).settings;
    // cout << "New settings : " << settings << endl;
    
    // // factory reset settings
    // cout << "Type enter to reset settings" << endl;
    // getchar();
    // cout << command::settings_reset(seatrac) << endl;
    // cout << "New settings : " << command::settings_get(seatrac).settings << endl;

    for(int i = 0; i < 100; i++) {
        getchar();
        try {
            //cout << command::sys_alive(seatrac) << endl;
            //cout << command::sys_info(seatrac) << endl;
            //cout << command::status_config_get(seatrac) << endl;
            //cout << command::settings_get(seatrac) << endl;
            cout << command::ping_send(seatrac, BEACON_ID_1) << endl;
            //cout << command::ping_send(seatrac, BEACON_ID_15, MSG_REQU) << endl;
            //cout << command::background_noise(seatrac) << endl;
            //cout << command::xcvr_status(seatrac) << endl;
        }
        catch(const TimeoutReached& e) {
            cout << "Timeout reached" << endl;
        }
    }

    // for(int i = 0; i < 100; i++) {
    //     try {
    //         cout << get_next_status(seatrac) << endl;
    //     }
    //     catch(const TimeoutReached& e) {
    //         cout << "Timeout reached" << endl;
    //     }
    // }

    service.stop();

    return 0;
}

