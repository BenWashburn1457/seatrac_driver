#ifndef _DEF_SEATRAC_DRIVER_MESSAGES_DATA_SEND_H_
#define _DEF_SEATRAC_DRIVER_MESSAGES_DATA_SEND_H_

#include <seatrac_driver/SeatracTypes.h>
#include <seatrac_driver/messages/MessageBase.h>

namespace narval { namespace seatrac { namespace messages {

struct DataQueueClear : public Message<DataQueueClear> {

    using Message<DataQueueClear>::operator=;

    static const CID_E Identifier = CID_DAT_QUEUE_CLR;
    struct Request : public Message<Request> {
        static const CID_E Identifier = CID_DAT_QUEUE_CLR;
        BID_E destId;
    }__attribute__((packed));

    CST_E status;
    BID_E destId;

}__attribute__((packed));

}; //namespace messages
}; //namespace seatrac
}; //namespace narval

inline std::ostream& operator<<(std::ostream& os,
                                const narval::seatrac::messages::DataQueueClear::Request& msg)
{
    os << "DataQueueClear Request: " 
       << "\n- Id of queue cleared:\t" << msg.destId;
    return os;
}

inline std::ostream& operator<<(std::ostream& os,
                                const narval::seatrac::messages::DataQueueClear& msg)
{
    os << "DataQueueClear Response: " << std::endl
       << "- status: " << msg.status << std::endl
       << "- Id of queue cleared: " << msg.destId << std::endl;
    return os;
}


#endif //_DEF_SEATRAC_DRIVER_MESSAGES_DATA_SEND_H_