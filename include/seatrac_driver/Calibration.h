#ifndef _DEF_SEATRAC_DRIVER_CALIBRATION_H_
#define _DEF_SEATRAC_DRIVER_CALIBRATION_H_

#include <iostream>
#include <cstdio>
#include <seatrac_driver/SeatracTypes.h>
#include <seatrac_driver/messages/MessageBase.h>
#include <seatrac_driver/SeatracDriver.h>

/*
    Contains helper functions to quickly calibrate the acoustic beacon.
*/

namespace narval { namespace seatrac { namespace command {

    struct CalibrationActionMsg {
        CID_E msgId;
        CAL_ACTION_E action;
    }__attribute__((packed));

    struct CalibrationResponse {
        CID_E msgId;
        CST_E status;
    }__attribute__((packet));



    //Function walks user through a terminal calibration procedure
    //Take note that this is a blocking function
    bool TerminalCalibration(SeatracDriver& driver, std::ostream& out, std::istream& in) {
        char input[50];
        out << "--- Seatrac Modem Accelerometer Calibration ---" << std::endl
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