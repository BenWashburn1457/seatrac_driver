#ifndef _DEF_SEATRAC_DRIVER_MESSAGES_DATA_SEND_H_
#define _DEF_SEATRAC_DRIVER_MESSAGES_DATA_SEND_H_

#include <seatrac_driver/SeatracTypes.h>
#include <seatrac_driver/messages/MessageBase.h>

namespace narval { namespace seatrac { namespace messages {

struct DataQueueStatus : public Message<DataQueueStatus> {

    using Message<DataQueueStatus>::operator=;

    static const CID_E Identifier = CID_DAT_QUEUE_STATUS;
    struct Request : public Message<Request> {
        static const CID_E Identifier = CID_DAT_QUEUE_STATUS;
    }__attribute__((packed));

    uint8_t packetLengths[16];

}__attribute__((packed));

}; //namespace messages
}; //namespace seatrac
}; //namespace narval


inline std::ostream& operator<<(std::ostream& os,
                                const narval::seatrac::messages::DataQueueStatus& msg)
{
    os << "DataQueueStatus Response: " << std::endl;
    for(int i=1; i<16; i++) {
        os << " - id " << i << " queued packet length: " << msg.packetLengths[i] << std::endl;
    }
    return os;
}


#endif //_DEF_SEATRAC_DRIVER_MESSAGES_DATA_SEND_H_