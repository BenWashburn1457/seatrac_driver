#ifndef _DEF_SEATRAC_DRIVER_CALIBRATION_H_
#define _DEF_SEATRAC_DRIVER_CALIBRATION_H_

#include <iostream>
#include <cstdio>
#include <seatrac_driver/SeatracTypes.h>
#include <seatrac_driver/messages/MessageBase.h>
#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/messages/Status.h>
#include <seatrac_driver/messages/StatusConfigSet.h>

using namespace narval::seatrac;

/*
    Contains helper functions to quickly calibrate the acoustic beacon.
*/

namespace narval { namespace seatrac { namespace calibration {

    struct CalibrationActionMsg {
        CID_E msgId;
        CAL_ACTION_E action;
    }__attribute__((packed));

    struct CalibrationResponse {
        CID_E msgId;
        CST_E status;
    }__attribute__((packet));


    class CalibrationDriver: public SeatracDriver  {
    public:

        MyDriver(std::ostream& out, const std::string& serialPort = "/dev/ttyUSB0") :
            SeatracDriver(serialPort)
        {
            print_out = out;
        }

        bool printCalData = false;

        // this method is called on any message returned by the beacon.
        void on_message(CID_E msgId, const std::vector<uint8_t>& data) {
            //replace code in this method by your own
            switch(msgId) {
                default:
                    std::cout << "Got message : " << msgId << std::endl << std::flush;
                    break;

                case CID_STATUS:
                    if(printCalData) {
                    messages::Status status;
                    status = data;
                    
                    } break;
            }
        }
    private:
        std::ostream* print_out;
    }


    //Call this function to run a calibration command
    //Function walks user through a terminal calibration procedure
    //Take note that this is a blocking function
    bool calibrateAccelerometer(std::ostream& out, std::istream& in, std::string serial_port) {

        CalibrationDriver calSeatrac(serial_port);

        messages::StatusConfigSet::Request statusSet;
        statusSet.statusOutput = STATUS_BITS_E::ACC_CAL + STATUS_BITS_E::AHRS_RAW_DATA; //status set to report the accelerometer calibration


        char input[50];
        out << "--- Seatrac USBL Modem Accelerometer Calibration ---" << std::endl
            << "This calibration procedure may be performed out of water" //TODO: check if true
            << "Please hold the modem in an upright position (with the modem cable facing down)" << std::endl
            << "When ready, Press Enter to continue";
        in.get();
        out << "Slowly rotate the beacon around the verticle axis"
        //TODO: finish calibration sequence
    }

    bool calibrateMagnetometer(std::ostream& out, std::istream& in, std::string serial_port) {

        CalibrationDriver calSeatrac(serial_port);


        char input[50];
        out << "--- Seatrac USBL Modem Magnetometer Calibration ---" << std::endl
            << "This calibration procedure may be performed out of water" //TODO: check if true
            << "Please hold the modem in an upright position (with the modem cable facing down)" << std::endl
            << "When ready, Press Enter to continue";
        in.get();
        out << "Slowly rotate the beacon around the verticle axis"
        //TODO: finish calibration sequence
    }

}; //namespace calibration
}; //namespace seatrac
}; //namespace narval

#endif //_DEF_SEATRAC_DRIVER_CALIBRATION_H_
