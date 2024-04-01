#ifndef _DEF_SEATRAC_DRIVER_CALIBRATION_H_
#define _DEF_SEATRAC_DRIVER_CALIBRATION_H_

#include <ostream>
#include <istream>
#include <seatrac_driver/SeatracTypes.h>
#include <seatrac_driver/messages/MessageBase.h>

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
    bool TerminalCalibration(std::ostream& out, std::istream& in) {
        out << "--- Seatrac Modem Calibration ---" << std::endl;
        out << "Please hold the modem in an upright position (with the modem cable facing down)" << std::endl
            << "When ready, Press Enter to continue";
        //TODO: finish calibration sequence
    }

}; //namespace calibration
}; //namespace seatrac
}; //namespace narval

#endif //_DEF_SEATRAC_DRIVER_CALIBRATION_H_