#include <iostream>

#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/messages/Messages.h>
#include <seatrac_driver/commands.h>
#include <seatrac_driver/Calibration.h>
using namespace narval::seatrac;


class MyDriver : public SeatracDriver
{
    public:

    MyDriver(const std::string& serialPort = "/dev/ttyUSB0") :
        SeatracDriver(serialPort)
    {}

    // this method is called on any message returned by the beacon.
    void on_message(CID_E msgId, const std::vector<uint8_t>& data) {
        switch(msgId) {
            default:
                std::cout << "Got message : " << msgId << std::endl << std::flush;
                break;

            case CID_STATUS: {
                messages::Status status;
                status = data;
                calibration::printCalFeedback(std::cout, status);
            } break;
        }
    }
};

int main(int argc, char *argv[])
{
    //set serial to command line input if given
    std::string serial_port;
    if (argc == 1) { serial_port = "/dev/ttyUSB0"; }
    else { serial_port = argv[1]; }

    MyDriver seatrac(serial_port);

    getchar();
    calibration::turnOnAccCalFeedback(seatrac);
    getchar();
    calibration::turnOffCalFeedback(seatrac);
    getchar();
    calibration::turnOnMagCalFeedback(seatrac);
    getchar();
    calibration::turnOffCalFeedback(seatrac);

    return 0;
}
