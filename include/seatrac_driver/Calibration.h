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
    }__attribute__((packed));

    inline void turnOnAccCalFeedback(SeatracDriver& seatrac, STATUS_BITS_E prevStatusBits=static_cast<STATUS_BITS_E>(0x0), STATUSMODE_E statusMode=STATUS_MODE_10HZ) {
        messages::StatusConfigSet::Request statusSet;
        
        statusSet.statusOutput = ACC_CAL | AHRS_RAW_DATA | prevStatusBits; 
        statusSet.statusMode   = statusMode;

        seatrac.send(sizeof(statusSet), (const uint8_t*)&statusSet);
    }
    inline void turnOnMagCalFeedback(SeatracDriver& seatrac, STATUS_BITS_E prevStatusBits=static_cast<STATUS_BITS_E>(0x0), STATUSMODE_E statusMode=STATUS_MODE_10HZ) {
        messages::StatusConfigSet::Request statusSet;
        
        statusSet.statusOutput = MAG_CAL | AHRS_RAW_DATA | prevStatusBits; 
        statusSet.statusMode   = statusMode;

        seatrac.send(sizeof(statusSet), (const uint8_t*)&statusSet);
    }
    inline void turnOffCalFeedback(SeatracDriver& seatrac, STATUS_BITS_E prevStatusBits=static_cast<STATUS_BITS_E>(0x0), STATUSMODE_E statusMode=STATUS_MODE_10HZ) {
        messages::StatusConfigSet::Request statusSet;
        
        statusSet.statusOutput = static_cast<STATUS_BITS_E>(prevStatusBits & ~ACC_CAL & ~MAG_CAL & ~AHRS_RAW_DATA); 
        statusSet.statusMode   = statusMode;

        seatrac.send(sizeof(statusSet), (const uint8_t*)&statusSet);
    }

    //call this function in SeatracDriver::on_message under the CID_STATUS case to
    //print calibration data if calibration fields exist

    inline void printCalFeedback(std::ostream& out, const messages::Status& status) {
        if (status.contentType & ACC_CAL) {
            out << "\tXmin:" << status.accCalibration.accLimMinX
                << "\tXmax:" << status.accCalibration.accLimMaxX
                << "\tYmin:" << status.accCalibration.accLimMinY
                << "\tYmax:" << status.accCalibration.accLimMaxY
                << "\tZmin:" << status.accCalibration.accLimMinZ
                << "\tZmax:" << status.accCalibration.accLimMaxZ
                << std::endl;
        }
        else if (status.contentType & MAG_CAL) {
            uint8_t mag_progress = status.magCalibration.magCalBuf;
            std::string progressBar(' ',50);
            for(uint8_t i=0; i<mag_progress/2; i++) progressBar[i] = '=';
            out << "MagCal: " << mag_progress << "%\t" 
                << "|" << progressBar << "|" << std::endl;
        }
    }

    
    //Call this function to run a calibration command
    //Function walks user through a terminal calibration procedure
    //Take note that this is a blocking function
    bool calibrateAccelerometer(SeatracDriver& seatrac, std::ostream& out, std::istream& in) {

        char input[50];
        out << "--- Seatrac USBL Modem Accelerometer Calibration ---" << std::endl
            << "This calibration procedure may be performed out of water" //TODO: check if true
            << "Please hold the modem in an upright position (with the modem cable facing down)" << std::endl
            << "When ready, Press Enter to continue";
        in.get();
        out << "Slowly rotate the beacon around the verticle axis";
        //TODO: finish calibration sequence
        return true;
    }

    bool calibrateMagnetometer(std::ostream& out, std::istream& in, std::string serial_port) {

        char input[50];
        out << "--- Seatrac USBL Modem Magnetometer Calibration ---" << std::endl
            << "This calibration procedure may be performed out of water" //TODO: check if true
            << "Please hold the modem in an upright position (with the modem cable facing down)" << std::endl
            << "When ready, Press Enter to continue";
        in.get();
        out << "Slowly rotate the beacon around the verticle axis";
        //TODO: finish calibration sequence
        return true;
    }

}; //namespace calibration
}; //namespace seatrac
}; //namespace narval

#endif //_DEF_SEATRAC_DRIVER_CALIBRATION_H_
