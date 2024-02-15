#include <iostream>
using namespace std;

#include <seatrac_driver/SeatracClient.h>
using namespace narval::seatrac;

// This is the code found in seatrac documentation.
uint16_t CalcCRC16(uint8_t* buf, uint16_t len)
{
    uint16_t poly = 0xA001;
    uint16_t crc = 0;
    for(uint16_t b = 0; b < len; b++) {
        uint8_t v = *buf;
        for(uint8_t i = 0; i < 8; i++) {
            if((v & 0x01) ^ (crc & 0x01)) {
                crc >>= 1;
                crc ^= poly;
            }
            else {
                crc >>= 1;
            }
            v >>= 1;
        }
        buf++;
    }
    return crc;
}

std::vector<uint8_t> from_string(const std::string& msg)
{
    std::vector<uint8_t> data(msg.size() / 2);
    auto fromHex = [](char c) { return (uint8_t)(c >= 'A') ? (c - 'A' + 10) : (c - '0'); };
    for(int i = 0; i < data.size(); i++) {
        data[i] = (fromHex(msg[2*i]) << 4) + fromHex(msg[2*i + 1]);
    }
    return data;
}

int main()
{
    std::string msg1 = "02";
    std::string msg2 = "15";
    std::string msg3 = "1000";
    std::string msg4 = "4002";
    std::string msg5 = "3102010400000000";
    std::string msg6 = "0282330000011B0301690E000000000000FF900301006901B7FAC5BFFF910301007A07750463A9";
    
    std::vector<uint8_t> data;

    uint16_t checksum = 0;
    for(auto c : from_string(msg1)) {
        SeatracClient::crc16_update(checksum, c);
    }
    cout << "msg1 : " << std::hex << checksum << endl;
    data = from_string(msg1);
    cout << "msg1 : " << CalcCRC16(data.data(), data.size()) << endl;

    checksum = 0;
    for(auto c : from_string(msg2)) {
        SeatracClient::crc16_update(checksum, c);
    }
    cout << "msg2 : " << std::hex << checksum << endl;
    data = from_string(msg2);
    cout << "msg2 : " << CalcCRC16(data.data(), data.size()) << endl;

    checksum = 0;
    for(auto c : from_string(msg3)) {
        SeatracClient::crc16_update(checksum, c);
    }
    cout << "msg3 : " << std::hex << checksum << endl;
    data = from_string(msg3);
    cout << "msg3 : " << CalcCRC16(data.data(), data.size()) << endl;

    checksum = 0;
    for(auto c : from_string(msg4)) {
        SeatracClient::crc16_update(checksum, c);
    }
    cout << "msg4 : " << std::hex << checksum << endl;
    data = from_string(msg4);
    cout << "msg4 : " << CalcCRC16(data.data(), data.size()) << endl;


    checksum = 0;
    for(auto c : from_string(msg5)) {
        SeatracClient::crc16_update(checksum, c);
    }
    cout << "msg5 : " << std::hex << checksum << endl;
    data = from_string(msg5);
    cout << "msg5 : " << CalcCRC16(data.data(), data.size()) << endl;


    checksum = 0;
    for(auto c : from_string(msg6)) {
        SeatracClient::crc16_update(checksum, c);
    }
    cout << "msg6 : " << std::hex << checksum << endl;
    data = from_string(msg6);
    cout << "msg6 : " << CalcCRC16(data.data(), data.size()) << endl;


    return 0;
}

