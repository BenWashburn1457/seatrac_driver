
#include <iostream>
#include <chrono>
#include <thread>

#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/messages/Messages.h>
#include <seatrac_driver/commands.h>

using namespace narval::seatrac;
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds


class MyDriver : public SeatracDriver
{
    public:

    MyDriver(const std::string& serialPort = "/dev/ttyUSB0") :
        SeatracDriver(serialPort)
    {}

    void send_echo(const BID_E destId, 
                   const AMSGTYPE_E msgType, 
                   const uint8_t data_length, 
                   const uint8_t* data) {

        messages::EchoSend::Request message;                      //create a message to send
        
        message.destId    = destId;                               //beacon it's sending the data to
        message.msgType   = msgType;                              //type of message being sent
        message.packetLen = std::min(data_length, (uint8_t)sizeof(message.packetData));   //the length of data packet (must be no more than 30 bytes)

        std::memcpy(message.packetData, data, message.packetLen); //copy the bytes (chars) from data into our message structure

        this->send(sizeof(message), (const uint8_t*)&message);    //prepares data to send over serial line

    }

    // this method is called on any message returned by the beacon.
    void on_message(CID_E msgId, const std::vector<uint8_t>& data) {
        //replace code in this method by your own
        switch(msgId) {
            default:
                std::cout << "Got message : " << msgId << std::endl << std::flush;
                break;

            case CID_ECHO_REQ:
                {
                    messages::EchoReq response;         //struct that contains response fields
                    response = data;                    //operator overload fills in response struct with correct data
                    std::cout << response << std::endl; //operator overload prints out response data
                }
                break;
            case CID_ECHO_RESP:
                {
                    messages::EchoResp response;        //struct that contains response fields
                    response = data;                    //operator overload fills in response struct with correct data
                    std::cout << response << std::endl; //operator overload prints out response data
                }
                break;
            case CID_ECHO_ERROR:
                {
                    messages::EchoError response;
                    response = data;
                    std::cout << response << std::endl;
                }
                break;
            case CID_ECHO_SEND:
                {
                    messages::EchoSend response;
                    response = data;
                    std::cout << response << std::endl;
                } break;

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
    if (argc == 1) serial_port = "/dev/ttyUSB0";
    else           serial_port = argv[1];

    MyDriver seatrac(serial_port);

    std::string message_txt;

    for (int i=0; i<50; i++) {
        std::cout << "message text: " << std::flush;
        getline(std::cin, message_txt);
        if (message_txt.length() == 0) break;

        seatrac.send_echo(
            BEACON_ID_15,
            MSG_REQU,
            (uint8_t)message_txt.size(),
            (const uint8_t*)message_txt.data()
          );
        sleep_for(milliseconds(25));
    }

    //getchar();

    return 0;
}
