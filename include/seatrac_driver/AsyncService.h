#ifndef _DEF_SEATRAC_DRIVER_ASYNC_SERVICE_H_
#define _DEF_SEATRAC_DRIVER_ASYNC_SERVICE_H_

#include <iostream>
#include <thread>
#include <memory>

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/asio.hpp>
#include <boost/bind.hpp>

namespace narval { namespace seatrac {

class AsyncService
{
    public:

    using IoService    = boost::asio::io_service;
    using IoServicePtr = std::shared_ptr<IoService>;

    protected:
    
    IoServicePtr service_;
    std::thread  thread_;
    bool         isRunning_;

    public:

    AsyncService();
    ~AsyncService();

    IoServicePtr io_service();

    bool is_running() const;
    void start();
    void stop();
};

}; //namespace seatrac
}; //namespace narval

#endif //_DEF_SEATRAC_DRIVER_ASYNC_SERVICE_H_
