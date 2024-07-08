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

    void nav_query_beacon(BID_E target) {
        messages::NavQuerySend::Request req;
        req.destId   = target;
        req.queryFlags = (NAV_QUERY_E)(QRY_DEPTH | QRY_SUPPLY | QRY_TEMP | QRY_ATTITUDE);
        req.packetLen = 0;

        this->send(sizeof(req), (const uint8_t*)&req);
    }

    // this method is called on any message returned by the beacon.
    void on_message(CID_E msgId, const std::vector<uint8_t>& data) {
        //replace code in this method by your own
        switch(msgId) {
            default:
                std::cout << "Got message : " << msgId << std::endl << std::flush;
                break;

            case CID_NAV_QUERY_SEND: {
                messages::NavQuerySend msg;
                msg = data;
                std::cout << msg << std::endl;
            } break;
            case CID_NAV_QUERY_REQ: {
                messages::NavQueryReq msg;
                msg = data;
                std::cout << msg << std::endl;
            } break;
            case CID_NAV_QUERY_RESP: {
                messages::NavQueryResp msg;
                msg = data;
                std::cout << msg << std::endl;

                this->nav_query_beacon(msg.acoFix.srcId);
            } break;
            case CID_NAV_ERROR: {
                messages::NavError msg;
                msg = data;
                std::cout << msg << std::endl;

                this->nav_query_beacon(msg.beaconId);
            } break;

            case CID_STATUS:
                // too many STATUS messages so bypassing display.
                break;
        }

    }
};

int main(int argc, char *argv[])
{

    std::string serial_port;
    if (argc == 1) serial_port = "/dev/ttyUSB0";
    else serial_port = argv[1];

    MyDriver seatrac(serial_port);    

    seatrac.nav_query_beacon(BEACON_ID_15);

    getchar();

    return 0;
}
