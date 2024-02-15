#include <seatrac_driver/DataWaiter.h>

namespace narval { namespace seatrac {

DataWaiterBase::DataWaiterBase(CID_E msgId) :
    msgId_(msgId),
    wasCalled_(false)
{}

CID_E DataWaiterBase::msg_id() const
{
    return msgId_;
}

bool DataWaiterBase::wait_for_data(int64_t timeout)
{
    std::unique_lock<std::mutex> lock(mutex_);

    if(wasCalled_)
        return true;

    if(timeout < 0) {
        cv_.wait(lock, [&]{ return wasCalled_; });
    }
    else {
        if(!cv_.wait_for(lock, std::chrono::milliseconds(timeout),
                         [&]{ return wasCalled_; })) {
            // Timeout reached, data was not set
            return false;
        }
    }
    return true;
}

}; //namespace seatrac
}; //namespace narval
