#ifndef _DEF_SEATRAC_DRIVER_SEATRAC_DRIVER_H_
#define _DEF_SEATRAC_DRIVER_SEATRAC_DRIVER_H_

#include <memory>
#include <functional>
#include <list>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include <seatrac_driver/SeatracEnums.h>
#include <seatrac_driver/SeatracTypes.h>
#include <seatrac_driver/SeatracClient.h>
#include <seatrac_driver/DataWaiter.h>

namespace narval { namespace seatrac {

class SeatracDriver : public SeatracClient
{
    public:

    using IoService    = boost::asio::io_service;
    using IoServicePtr = std::shared_ptr<boost::asio::io_service>;
    using SerialPort   = boost::asio::serial_port;
    using ReadBuffer   = boost::asio::streambuf;

    using Waiters = std::list<DataWaiterBase::Ptr>;

    protected:
    
    Waiters    waiters_;
    std::mutex waitersMutex_;
    
    virtual void on_receive(const std::vector<uint8_t>& data);
    virtual void on_message(CID_E msgId, const std::vector<uint8_t>& data);

    public:

    SeatracDriver(rtac::asio::Stream::Ptr stream,
                  std::size_t bufferSize = 8192);
    SeatracDriver(const std::string& port = "/dev/ttyUSB0",
                  std::size_t bufferSize = 8192);
    
    template <typename T>
    bool send_request(unsigned int cmdSize, const uint8_t* cmdData,
                      T* respData, int64_t timeout = 5000);
    
    template <typename T>
    bool wait_for_message(CID_E msgId, T* data, int64_t timeout = 5000);
};

/**
 * Sends a command and waits for an answer (synchronous call)
 */
template <typename T>
bool SeatracDriver::send_request(unsigned int cmdSize, const uint8_t* cmdData,
                                 T* respData, int64_t timeout)
{
    CID_E msgId = (CID_E)cmdData[0]; // TODO check validity
    auto waiter = std::make_shared<DataWaiter<T>>(msgId, respData);
    {
        std::unique_lock<std::mutex> lock(waitersMutex_);
        waiters_.push_back(waiter);
    }

    // sending data when waiter is set, not before (to avoid missing the
    // response)
    this->send(cmdSize, cmdData);
    if(!waiter->wait_for_data(timeout)) {
        std::cerr << "Timeout reached while waiting for request response (cmd_id : " 
                  << msgId << ")" << std::endl;
        return false;
    }
    return true;
}

/**
 * Waits for a specific message identified by its CID_E (with a timeout).
 */
template <typename T>
bool SeatracDriver::wait_for_message(CID_E msgId, T* data, int64_t timeout)
{
    auto waiter = std::make_shared<DataWaiter<T>>(msgId, data);
    {
        std::unique_lock<std::mutex> lock(waitersMutex_);
        waiters_.push_back(waiter);
    }
    
    // TODO delete waiter if timeout reached
    return waiter->wait_for_data(timeout);
}

}; //namespace seatrac
}; //namespace narval

#endif //_DEF_SEATRAC_DRIVER_SEATRAC_DRIVER_H_

