#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/print_utils.h>

#include <seatrac_driver/messages/Messages.h>

namespace narval { namespace seatrac {

SeatracDriver::SeatracDriver(rtac::asio::Stream::Ptr stream,
                             std::size_t bufferSize) :
    SeatracClient(stream, bufferSize)
{}

SeatracDriver::SeatracDriver(const std::string& port,
                             std::size_t bufferSize) :
    SeatracClient(port, bufferSize)
{}

void SeatracDriver::on_receive(const std::vector<uint8_t>& data)
{
    // main dispatch function

    CID_E msgId = (CID_E)data[0]; // TODO check validity
    {
        // Iterating on waiters and setting data when msgId match.
        // If there is a match, waiter is to be deleted.
        std::unique_lock<std::mutex> lock(waitersMutex_);
        auto it = waiters_.begin();
        while(it != waiters_.end()) {
            if((*it)->msg_id() == msgId) {
                (*it)->set_data(data);
                waiters_.erase(it++);
            }
            else {
                it++;
            }
        }
    }
    this->on_message(msgId, data);
}

void SeatracDriver::on_message(CID_E msgId, const std::vector<uint8_t>& data)
{
    switch(msgId) {
        default:
            std::cout << "Got message : " << msgId << std::endl << std::flush;
            break;
        case CID_PING_ERROR:
            {
                messages::PingError response;
                response = data;
                std::cout << response << std::endl;
            }
            break;
        case CID_PING_RESP:
            // std::cout << "Got a Ping Response" << std::endl << std::flush;
            {
                messages::PingResp response;
                response = data;
                std::cout << response << std::endl;
            }
            break;
        //case CID_STATUS:
        //    break;
    }
}

}; //namespace seatrac
}; //namespace narval
