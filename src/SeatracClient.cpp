#include <seatrac_driver/SeatracClient.h>

using namespace std::placeholders;

namespace narval { namespace seatrac {

/**
 * This compute a single step of a CRC16 checksum.
 */
void SeatracClient::crc16_update(uint16_t& checksum, uint8_t v)
{
    const uint16_t poly = 0xA001;
    checksum = ((v    & 0x01) ^ (checksum & 0x01)) ? (checksum>>1) ^ poly : checksum>>1;
    checksum = ((v>>1 & 0x01) ^ (checksum & 0x01)) ? (checksum>>1) ^ poly : checksum>>1;
    checksum = ((v>>2 & 0x01) ^ (checksum & 0x01)) ? (checksum>>1) ^ poly : checksum>>1;
    checksum = ((v>>3 & 0x01) ^ (checksum & 0x01)) ? (checksum>>1) ^ poly : checksum>>1;
    checksum = ((v>>4 & 0x01) ^ (checksum & 0x01)) ? (checksum>>1) ^ poly : checksum>>1;
    checksum = ((v>>5 & 0x01) ^ (checksum & 0x01)) ? (checksum>>1) ^ poly : checksum>>1;
    checksum = ((v>>6 & 0x01) ^ (checksum & 0x01)) ? (checksum>>1) ^ poly : checksum>>1;
    checksum = ((v>>7 & 0x01) ^ (checksum & 0x01)) ? (checksum>>1) ^ poly : checksum>>1;
}

/**
 * Converts a single hexadecimal synbol in ascii into its decimal value.
 *
 * Only valid for ascii digit characters [0,...,9] and characters ABCDEF.
 * ('0'=0, '9'=9, 'A'=10, 'B'=11, 'F'=15).
 *
 * Caution : only uppercase letters are valid.
 *
 * @param c a character representing an hexadecimal digit.
 *
 * @return a value between [0,15].
 */
uint8_t SeatracClient::hexascii_to_value(char c)
{
    return (c >= 'A') ? (c - 'A' + 10) : (c - '0');
}

/**
 * Converts a decimal value between [0,15] into its ascii hexadecimal
 * representation.
 *
 * @param v a value between [0,15] (only 4 bits are used)
 *
 * @return an hexadecimal symbol ['0',...,'9'] or ABCDEF (guaranteed
 *         uppercase).
 */
char SeatracClient::value_to_hexascii(uint8_t v)
{
    return (v >= 10) ? (v - 10 + 'A') : (v + '0');
}

SeatracClient::SeatracClient(rtac::asio::Stream::Ptr stream, std::size_t bufferSize) :
    stream_(stream),
    receiveBuffer_(bufferSize)
{
    stream_->start(); // forcing asynchronous for now
    this->initiate_read();
}


SeatracClient::SeatracClient(const std::string& port, std::size_t bufferSize) :
    SeatracClient(rtac::asio::Stream::CreateSerial(port, 115200), bufferSize)
{}

SeatracClient::~SeatracClient()
{
    stream_->stop();
}

void SeatracClient::reset()
{
    stream_->reset();
}

void SeatracClient::initiate_read()
{
    stream_->async_read_until(receiveBuffer_.size(), receiveBuffer_.data(), Delimiter,
        std::bind(&SeatracClient::read_callback, this, _1, _2));
}

void SeatracClient::read_callback(const boost::system::error_code& err, size_t byteCount)
{
    currentTime_ = SeatracClient::now();
    if(err || byteCount == 0) {
        std::ostringstream oss;
        oss << "Reading error : " << err << ", bytecount : " << byteCount;
        std::cerr << oss.str() << std::endl;
    }
    else {
        if(receiveBuffer_[0] == '$') {
            this->decode_received(byteCount);
        }
        else {
            std::cerr << "Start of message missing. Ignoring." << std::endl;
        }
    }
    
    stream_->async_read_until(receiveBuffer_.size(), receiveBuffer_.data(), Delimiter,
        std::bind(&SeatracClient::read_callback, this, _1, _2));
}

void SeatracClient::decode_received(std::size_t receivedCount)
{
    // msg should be a complete message (starting with '$' and ending with delimiter)

    decodedData_.resize((receivedCount - 7) / 2);
    int processed_ = 1; // starting at 1 to discard the '$' character
    uint16_t checksum = 0;
    for(int i = 0; i < decodedData_.size(); i++) {
        decodedData_[i]  = hexascii_to_value(receiveBuffer_[processed_++]    ) << 4;
        decodedData_[i] += hexascii_to_value(receiveBuffer_[processed_++]);
        crc16_update(checksum, decodedData_[i]);
    }

    // Decoding transmitted checksum
    uint16_t transmittedChecksum = 0;
    transmittedChecksum  = hexascii_to_value(receiveBuffer_[processed_++]) << 4;
    transmittedChecksum += hexascii_to_value(receiveBuffer_[processed_++]);
    transmittedChecksum += hexascii_to_value(receiveBuffer_[processed_++]) << 12;
    transmittedChecksum += hexascii_to_value(receiveBuffer_[processed_++]) << 8;

    if(checksum != transmittedChecksum) {
        std::ostringstream oss;
        oss << "Invalid data line (wrong checksum) : '";
        for(std::size_t i = 0; i < receivedCount; i++) {
            oss << receiveBuffer_[i];
        }
        std::cerr << oss.str() << std::endl;
        return;
    }
    
    this->on_receive(decodedData_);
}

void SeatracClient::on_receive(const std::vector<uint8_t>& data)
{
    std::cout << "Received " << data.size() << " bytes." << std::endl;
}

void SeatracClient::send(size_t size, const uint8_t* data)
{
    std::ostringstream oss;
    uint16_t checksum = 0;

    oss << '#';
    for(int i = 0; i < size; i++) {
        crc16_update(checksum, data[i]);
        oss << value_to_hexascii((data[i] & 0xf0) >> 4);
        oss << value_to_hexascii( data[i] & 0x0f);
    }
    oss << value_to_hexascii((checksum & 0x00f0) >> 4);
    oss << value_to_hexascii((checksum & 0x000f));
    oss << value_to_hexascii((checksum & 0xf000) >> 12);
    oss << value_to_hexascii((checksum & 0x0f00) >> 8);
    oss << "\r\n";

    std::string str = oss.str();
    stream_->async_write(str.size(), (const uint8_t*)str.data(),
        std::bind(&SeatracClient::on_sent, this, _1, _2));
}

void SeatracClient::on_sent(const boost::system::error_code& err, size_t byteCount)
{
    if(err) {
        std::ostringstream oss;
        oss << "Write error : " << err;
        std::cerr << oss.str() << std::endl;
    }
}

void SeatracClient::enable_io_dump(const std::string& rxFile,
                                   const std::string& txFile) const
{
    stream_->enable_io_dump();
}

void SeatracClient::disable_io_dump() const
{
    stream_->disable_io_dump();
}

SeatracClient::TimePoint SeatracClient::current_time() const
{
    return currentTime_;
}

}; //namespace seatrac
}; //namespace narval

