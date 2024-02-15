#include <iostream>
#include <stdio.h>

#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/messages/Messages.h>
#include <seatrac_driver/commands.h>
using namespace narval::seatrac;


struct CidDatRecieveMessage {
    CID_E msgId;
    ACOFIX_T acoFix;
    bool ackFlag;
    uint8_t packetLen;
    uint8_t packetData[31];
}__attribute__((packed));

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
            case CID_PING_ERROR:
                {
                    messages::PingError response;
                    response = data;
                    std::cout << response << std::endl;
                    //this->ping_beacon(response.beaconId);
                }
                break;
            case CID_PING_RESP:
                // std::cout << "Got a Ping Response" << std::endl << std::flush;
                {
                    messages::PingResp response;
                    response = data;
                    std::cout << response << std::endl;
                    //this->ping_beacon(response.acoFix.srcId);
                }
                break;
            case CID_DAT_RECEIVE:
                {
                    std::cout << "Got message : " << msgId << std::endl;
                    CidDatRecieveMessage message;
                    
                    //copy the bytes (chars) from data into our message structure
                    //std::memcpy((uint8_t*)&message, data.data(), std::min(sizeof(data), sizeof(message)));

                    std::cout << "message: ";
                    for (auto byte: data) {
                        printf("%x ", byte);
                    }
                    std::cout << std::endl;
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
    
    //command::ping_send(seatrac, BEACON_ID_10);

    getchar();

    return 0;
}
