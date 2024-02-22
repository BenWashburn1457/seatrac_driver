#ifndef _DEF_SEATRAC_DRIVER_MESSAGES_DATA_SEND_H_
#define _DEF_SEATRAC_DRIVER_MESSAGES_DATA_SEND_H_

namespace narval { namespace seatrac { namespace messages {

struct DataSend : public Message<DataSend> {
    static const CID_E Identifier = CID_DAT_SEND;
    BID_E destId;
    AMSGTYPE_E msgType;
    uint8_t packetLen;
    uint8_t packetData[31];

}__attribute__((packed));

}; //namespace messages
}; //namespace seatrac
}; //namespace narval

inline std::ostream& operator<<(std::ostream& os,
                                const narval::seatrac::messages::DataSend& msg)
{
    os << "DataSend: " 
       << "\n- target:\t" << msg.destId
       << "\n- msg type:\t" << msg.msgType
       << "\nPacket Length: " << (int)msg.packetLen
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

#endif //_DEF_SEATRAC_DRIVER_MESSAGES_DATA_SEND_H_