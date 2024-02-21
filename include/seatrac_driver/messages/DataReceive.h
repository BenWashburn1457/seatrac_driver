#ifndef _DEF_SEATRAC_DRIVER_MESSAGES_DATA_RECEIVE_H_
#define _DEF_SEATRAC_DRIVER_MESSAGES_DATA_RECEIVE_H_

#include <seatrac_driver/SeatracTypes.h>
#include <seatrac_driver/messages/MessageBase.h>

//TODO: Put Data Receive decoder here


namespace narval { namespace seatrac { namespace messages {

struct DataReceive : public Message<DataReceive>
{
    static const CID_E Identifier = CID_DAT_RECEIVE;
    ACOFIX_T acoFix;
    bool ackFlag;   // Flag is true if this message has been generated as a
                    // response to a CID_DAT_SEND command which
                    // requested an ACK – in which case, remotely queued
                    // data may have also been transmitted back and in
                    // included in this message.
    uint8_t packetLen;  // The number of bytes sent in the DAT acoustic
                        // packet. Valid values are from 0 to 31.
                        // A value of 0 indicate no data is present.
    uint8_t packetData[31]; // The array of data received in the DAT acoustic packet

    DataReceive& operator=(const std::vector<uint8_t>& other)
    {
        if(other[0] != this->msgId) {
            throw std::runtime_error("Wrong message for decoding.");
        }
        uint8_t acoFixSize = acoFix.assign(other.size() - 1, other.data() + 1);

        const uint8_t* p   = other.data() + 1 + acoFixSize; //pointer to start of rest of message
        this->ackFlag = p[0];
        this->packetLen = p[1];
        std::memcpy(this->packetData, p+2, this->packetLen);

        return *this;
    }

}__attribute__((packed));


}; //namespace messages
}; //namespace seatrac
}; //namespace narval

inline std::ostream& operator<<(std::ostream& os,
                                const narval::seatrac::messages::DataReceive& msg)
{
    os << "\nDataReceive: \n" 
       << msg.acoFix
       << "\nRequires Acknowledgement: " << msg.ackFlag
       << "\nPacket Length: " << (int)msg.packetLen
       << "\nPacket Data: ";
    for(uint8_t i=0; i<msg.packetLen; i++) {
        os << msg.packetData[i];
    }
    return os;
}

#endif //_DEF_SEATRAC_DRIVER_MESSAGES_DATA_RECEIVE_H_









