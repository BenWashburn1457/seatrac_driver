#ifndef _DEF_SEATRAC_DRIVER_DATA_WAITER_H_
#define _DEF_SEATRAC_DRIVER_DATA_WAITER_H_

#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>

#include <seatrac_driver/SeatracEnums.h>

namespace narval { namespace seatrac {

struct TimeoutReached : public std::exception
{
    const char* what() const throw() {
        return "Timeout reached while waiting for message.";
    }
};
    
class DataWaiterBase
{
    public:
    
    //using Ptr = std::unique_ptr<Waiter>;
    using Ptr = std::shared_ptr<DataWaiterBase>;

    protected:
    
    CID_E                   msgId_;
    std::vector<uint8_t>*   dataDestination_;
    std::mutex              mutex_;
    std::condition_variable cv_;
    bool                    wasCalled_;

    public:

    DataWaiterBase(CID_E msgId);

    CID_E msg_id() const;

    bool wait_for_data(int64_t timeout = 5000);
    virtual void set_data(const std::vector<uint8_t>& data) = 0;
};

template <typename T>
class DataWaiter : public DataWaiterBase
{
    public:

    using Ptr = std::shared_ptr<DataWaiter<T>>;

    protected:

    T* dataDestination_;

    public:

    DataWaiter(CID_E msgId, T* dataDestination);

    virtual void set_data(const std::vector<uint8_t>& data);
};

template <typename T>
DataWaiter<T>::DataWaiter(CID_E msgId, T* dataDestination) :
    DataWaiterBase(msgId),
    dataDestination_(dataDestination)
{
    assert(dataDestination_ != nullptr);
}

template <typename T>
void DataWaiter<T>::set_data(const std::vector<uint8_t>& data)
{
    {
        std::unique_lock<std::mutex> lock(this->mutex_);
        *dataDestination_ = data;
        this->wasCalled_ = true;
    }
    this->cv_.notify_all();
}

}; //namespace seatrac
}; //namespace narval

#endif //_DEF_SEATRAC_DRIVER_DATA_WAITER_H_
