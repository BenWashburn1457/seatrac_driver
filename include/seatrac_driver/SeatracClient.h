#ifndef _DEF_SEATRAC_DRIVER_SEATRAC_CLIENT_H_
#define _DEF_SEATRAC_DRIVER_SEATRAC_CLIENT_H_

#include <chrono>
#include <memory>

#include <rtac_asio/Stream.h>

namespace narval { namespace seatrac {

/**
 * The purpose of this class is to handle a binary connection with a Blueprint
 * Seatrac USBL device through a serial port (UDP and TCP connections are also
 * available. See rtac-asio docs for more info.
 *
 * This class handles the reception of encoded binary messages. It decodes the
 * received messages and pass the decoded binary data to a virtual method to be
 * reimplemented in a subclass (see the SeatracDriver class).
 *
 * This class can be directly used by the user. However, the SeatracDriver
 * class offers a higher level interface more used for most user.
 */
class SeatracClient
{
    public:

    using SteadyClock = std::chrono::steady_clock;
    using SteadyTime  = std::chrono::time_point<SteadyClock>;
    using SystemClock = std::chrono::system_clock;
    using SystemTime  = std::chrono::time_point<SystemClock>;
    struct TimePoint {
        SteadyTime steadyTime;
        SystemTime systemTime;

        TimePoint() :
            steadyTime(SteadyClock::now()),
            systemTime(SystemClock::now())
        {}
    };
    static TimePoint now() { return TimePoint(); }

    static constexpr char Delimiter = '\n';

    static void    crc16_update(uint16_t& checksum, uint8_t v);
    static uint8_t hexascii_to_value(char c);
    static char    value_to_hexascii(uint8_t v);

    private:
    
    rtac::asio::Stream::Ptr stream_;
    std::vector<uint8_t>    receiveBuffer_;
    std::vector<uint8_t>    decodedData_;

    TimePoint currentTime_;

    // These define the state machine which reads the serial port.
    void initiate_read();
    void on_first_read(const boost::system::error_code& err, size_t byteCount);
    void read_callback(const boost::system::error_code& err, size_t byteCount);
    void decode_received(std::size_t receivedCount);
    
    // These are called after decoding/sent and can be reimplemented in a subclass to
    // handle the received data or write error.
    virtual void on_receive(const std::vector<uint8_t>& data);
    virtual void on_sent(const boost::system::error_code& err, size_t byteCount);

    public:

    SeatracClient(rtac::asio::Stream::Ptr stream,
                  std::size_t bufferSize = 8192);
    SeatracClient(const std::string& port,
                  std::size_t bufferSize = 8192);
    ~SeatracClient();

    rtac::asio::Stream::Ptr stream() const { return stream_; }

    void reset();

    void send(size_t size, const uint8_t* data);

    void enable_io_dump(const std::string& rxFile="serial_rx.dump",
                        const std::string& txFile="serial_tx.dump") const;
    void disable_io_dump() const;

    TimePoint current_time() const;
};

} //namespace seatrac
} //namespace narval

#endif //_DEF_SEATRAC_DRIVER_SEATRAC_CLIENT_H_
