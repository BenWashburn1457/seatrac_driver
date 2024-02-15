#include <iostream>
#include <functional>
using namespace std;
using namespace std::placeholders;

#include <seatrac_driver/SeatracClient.h>
using namespace narval::seatrac;

void request_sys_info(boost::asio::steady_timer* timer,
                      SeatracClient* serial,
                      const boost::system::error_code& err)
{
    if(err) {
        std::ostringstream oss;
        oss << "Error waiting for timer : " << err << endl;
        throw std::runtime_error(oss.str());
    }
    
    uint8_t cid = 0x02;
    serial->send(1, &cid);
    
    timer->expires_from_now(boost::asio::chrono::seconds(2));
    timer->async_wait(std::bind(&request_sys_info, timer, serial, _1)); 
}

int main()
{
    SeatracClient serial("/dev/ttyUSB0");
    
    boost::asio::steady_timer timer(serial.stream()->service()->service(),
                                    boost::asio::chrono::seconds(2));
    timer.async_wait(std::bind(&request_sys_info, &timer, &serial, _1)); 

    getchar();

    return 0;
}

