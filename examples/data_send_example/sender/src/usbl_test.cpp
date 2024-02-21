#include <iostream>

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

    void send_data(const BID_E destId, 
                   const AMSGTYPE_E msgType, 
                   const uint8_t data_length, 
                   const uint8_t* data) {

        messages::DataSend message;   //create a message packet to send
        
        message.destId = BEACON_ALL;  //send it to all beacons regardless of id
        message.msgType = MSG_OWAYU;  //send the data away to other beacons with usbl information
        message.packetLen = std::min(data_length, (uint8_t)31); //the length of data packet

        std::memcpy(message.packetData, data, message.packetLen); //copy the bytes (chars) from data into our message structure

        this->send(sizeof(message), (const uint8_t*)&message); 

    }

    // this method is called on any message returned by the beacon.
    void on_message(CID_E msgId, const std::vector<uint8_t>& data) {
        //replace code in this method by your own
        switch(msgId) {
            default:
                std::cout << "Got message : " << msgId << std::endl << std::flush;
                break;
            case CID_PING_ERROR:
                {
                    std::cout << "case ping error" << std::endl;
                    messages::PingError response;
                    response = data;
                    std::cout << response << std::endl;
                    //this->ping_beacon(response.beaconId);
                }
                break;
            case CID_PING_RESP:
                // std::cout << "Got a Ping Response" << std::endl << std::flush;
                {
                    std::cout << "case ping response" << std::endl;
                    messages::PingResp response;
                    response = data;
                    std::cout << response << std::endl;
                    //this->ping_beacon(response.acoFix.srcId);
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
    //set serial to command line input if given
    std::string serial_port;
    if (argc == 1) { serial_port = "/dev/ttyUSB0"; }
    else { serial_port = argv[1]; }

    MyDriver seatrac(serial_port);

    std::string message_txt(31, 'x');

    for (int i=0; i<50; i++) {
        std::cout << "message text: " << std::flush;
        getline(std::cin, message_txt);
        if (message_txt.length() == 0) break;

        seatrac.send_data(
            BEACON_ALL,
            MSG_OWAYU, //other valid types: MSG_OWAY, MSG_REQ, MSG_REQU, MSG_REQX
            (uint8_t)message_txt.size(),
            (const uint8_t*)message_txt.data()
          );
    }

    //getchar();

    return 0;
}
