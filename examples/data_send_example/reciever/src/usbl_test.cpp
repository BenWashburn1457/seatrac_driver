#include <iostream>
#include <stdio.h>

#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/messages/Messages.h>
#include <seatrac_driver/commands.h>
using namespace narval::seatrac;


class MyDriver : public SeatracDriver
{
    public:

    MyDriver(const std::string& serialPort = "/dev/ttyUSB0") :
        SeatracDriver(serialPort)
    {}

    // this method is called on any message returned by the beacon.
    void on_message(CID_E msgId, const std::vector<uint8_t>& data) {
        //replace code in this method by your own
        switch(msgId) {
            default:
                std::cout << "Got message : " << msgId << std::endl << std::flush;
                break;

            case CID_DAT_RECEIVE:
                {
                    std::cout << "Got message : " << msgId << std::endl << std::flush;
                
                    messages::DataReceive response;
                    response = data;
                    std::cout << response << std::endl;
                }
                break;
            case CID_DAT_ERROR:
                {
                    messages::DataError response;
                    response = data;
                    std::cout << response << std::endl;
                }
                break;

            case CID_PING_ERROR:
                {
                    messages::PingError response;
                    response = data;
                    std::cout << response << std::endl;
                }
                break;
            case CID_PING_RESP:
                {
                    messages::PingResp response;
                    response = data;
                    std::cout << response << std::endl;
                }
                break;

            case CID_STATUS:
                // too many STATUS messages so bypassing display.
                break;
        }
    }
};

int main(int argc, char *argv[])
{
    
    std::string serial_port;
    if (argc == 1) { serial_port = "/dev/ttyUSB0"; }
    else { serial_port = argv[1]; }

    MyDriver seatrac(serial_port);

    std::cout << "waiting for message" << std::endl;

    getchar();

    return 0;
}
