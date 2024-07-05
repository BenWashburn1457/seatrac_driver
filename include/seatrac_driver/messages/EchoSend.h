#ifndef _DEF_SEATRAC_DRIVER_MESSAGES_ECHO_SEND_H_
#define _DEF_SEATRAC_DRIVER_MESSAGES_ECHO_SEND_H_

#include <seatrac_driver/SeatracTypes.h>
#include <seatrac_driver/messages/MessageBase.h>

namespace narval { namespace seatrac { namespace messages {

struct EchoSend : public Message<EchoSend> {

    using Message<EchoSend>::operator=;

    static const CID_E Identifier = CID_ECHO_SEND;
    struct Request : public Message<Request> {
        static const CID_E Identifier = CID_ECHO_SEND;
        BID_E destId;
        AMSGTYPE_E msgType;
        uint8_t packetLen;
        uint8_t packetData[30];
    }__attribute__((packed));

    CST_E status;
    BID_E beaconId;

}__attribute__((packed));

}; //namespace messages
}; //namespace seatrac
}; //namespace narval

inline std::ostream& operator<<(std::ostream& os,
                                const narval::seatrac::messages::EchoSend::Request& msg)
{
    os << "EchoSend Request: " 
       << "\n- target:\t" << msg.destId
       << "\n- msg type:\t" << msg.msgType
       << "\n- packet length:\t" << (int)msg.packetLen
       << "\nPacket Data:\n- (char):\t";
    for(uint8_t i=0; i<msg.packetLen; i++) {
        os << msg.packetData[i];
    }
    os << "\n- (hex):\t";
    for(uint8_t i=0; i<msg.packetLen; i++) {
        printf("%02X ", msg.packetData[i]);
    }
    os << "\n- (int):\t";
    for(uint8_t i=0; i<msg.packetLen; i++) {
        printf("%d ", msg.packetData[i]);
    }
    return os;
}

inline std::ostream& operator<<(std::ostream& os,
                                const narval::seatrac::messages::EchoSend& msg)
{
    os << "EchoSend Response: " << std::endl
       << "- status: " << msg.status << std::endl
       << "- beaconId: " << msg.beaconId << std::endl;
    return os;
}


#endif //_DEF_SEATRAC_DRIVER_MESSAGES_ECHO_SEND_H_