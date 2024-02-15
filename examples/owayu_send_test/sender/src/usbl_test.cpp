#include <iostream>

#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/messages/Messages.h>
#include <seatrac_driver/commands.h>
using namespace narval::seatrac;




/*
https://www.seascapesubsea.com/downloads/Blueprint-Subsea-SeaTrac-Developer-Guide.pdf
Seatrac dev manual page 116
*/
struct CidDatSendMessage {
    CID_E msgId;
    BID_E destId;
    AMSGTYPE_E msgType;
    uint8_t packetLen;
    uint8_t packetData[31];
}__attribute__((packed));


class MyDriver : public SeatracDriver
{
    public:

    MyDriver(const std::string& serialPort = "/dev/ttyUSB0") :
        SeatracDriver(serialPort)
    {}

    void send_data(const uint8_t* data, uint8_t length) {
        std::cout << "ping_beacon start" << std::endl;

        //create a message packet to send
        CidDatSendMessage message;
        
        message.msgId = CID_DAT_SEND; //message type is send data
        message.destId = BEACON_ALL;  //send it to all beacons regardless of id
        message.msgType = MSG_OWAYU;  //send the data away to other beacons with usbl information
        message.packetLen = std::min(length, (uint8_t)31); //31; //the length of data packet

        //copy the bytes (chars) from data into our message structure
        std::memcpy(message.packetData, data, 31);//std::min(length, 31));

        //fill the remaining bytes with 0x00. Might not be necessary
        //std::memset(message.packetData + sizeof(data), 0x00, sizeof(message.packetData) - sizeof(data));

        this->send(sizeof(message), (const uint8_t*)&message);

        std::cout << "Data to be sent:";
        for(int i=0; i<length; i++) {
            printf("%x ", data[i]);
        }
        printf("\n");

        std::cout << "Message being sent to modem:";
        uint8_t* msgdata = (uint8_t*)&message;
        for(int i=0; i<sizeof(message); i++) {
            printf("%x ", msgdata[i]);
        }
        std::cout << std::endl;


        std::cout << "ping_beacon end" << std::endl;
    }

    void on_receive(const std::vector<uint8_t>& data) {
        if (data[0] != CID_STATUS) { // if the data is not a regular status message. Too many status messages
            //std::cout << sizeof(data) << std::endl;
            std::cout << "on recieve";
            for(int i=0; i<sizeof(data); i++) {
                printf("%x ", data[i]);
            }
            std::cout << std::endl;
        }

        SeatracDriver::on_receive(data);
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
    
    std::string serial_port;
    if (argc == 1) { serial_port = "/dev/ttyUSB0"; }
    else { serial_port = argv[1]; }

    MyDriver seatrac(serial_port);

    std::string message_txt(31, 'x');

    for (int i=0; i<50; i++) {
        std::cout << "message text: " << std::flush;
        getline(std::cin, message_txt);
        if (message_txt.length() == 0) { break; }

        seatrac.send_data((const uint8_t*)message_txt.data(), message_txt.size());
    }

    //getchar();

    return 0;
}
