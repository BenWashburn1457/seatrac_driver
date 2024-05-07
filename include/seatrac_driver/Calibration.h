#ifndef _DEF_SEATRAC_DRIVER_CALIBRATION_H_
#define _DEF_SEATRAC_DRIVER_CALIBRATION_H_

#include <iostream>
#include <cstdio>
#include <chrono>
#include <thread>
#include <seatrac_driver/SeatracTypes.h>
#include <seatrac_driver/messages/MessageBase.h>
#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/messages/Status.h>
#include <seatrac_driver/messages/StatusConfigSet.h>

using namespace narval::seatrac;
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds

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

    inline void turnOnAccCalFeedback(SeatracDriver& seatrac, 
                        STATUS_BITS_E prevStatusBits=static_cast<STATUS_BITS_E>(0x0), 
                        STATUSMODE_E statusMode=STATUS_MODE_10HZ) {
        messages::StatusConfigSet::Request statusSet;
        
        statusSet.statusOutput = ACC_CAL | AHRS_RAW_DATA | prevStatusBits; 
        statusSet.statusMode   = statusMode;

        seatrac.send(sizeof(statusSet), (const uint8_t*)&statusSet);
    }
    inline void turnOnMagCalFeedback(SeatracDriver& seatrac, 
                        STATUS_BITS_E prevStatusBits=static_cast<STATUS_BITS_E>(0x0), 
                        STATUSMODE_E statusMode=STATUS_MODE_10HZ) {
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
            out << "\tXmin:  " << status.accCalibration.accLimMinX
                << "\tXmax:  " << status.accCalibration.accLimMaxX
                << "\tYmin:  " << status.accCalibration.accLimMinY
                << "\tYmax:  " << status.accCalibration.accLimMaxY
                << "\tZmin:  " << status.accCalibration.accLimMinZ
                << "\tZmax:  " << status.accCalibration.accLimMaxZ
                << std::endl;
        }
        else if (status.contentType & MAG_CAL) {
            uint8_t mag_progress = status.magCalibration.magCalBuf;
            std::string progressBar(50, ' ');
            for(uint8_t i=0; i<mag_progress/2; i++) progressBar[i] = '=';
            out << "MagCal: " << int(mag_progress) << "%\t" 
                << "|" << progressBar << "|" << std::endl;
        }
    }

    
    //Call this function to run a calibration command
    //Function walks user through a terminal calibration procedure
    //Ensure that printCalFeedback is being called in the status 
    //This is a blocking function
    inline void calibrateAccelerometer(SeatracDriver& seatrac, std::ostream& out, std::istream& in, bool saveToEEPROM = false) {
        
        turnOffCalFeedback(seatrac);
        sleep_for(nanoseconds(2000));
        out << "---\tSeatrac Accelerometer Calibration\t---" << std::endl
            << "Press enter to begin. Once the X Y and Z limits are found, press enter again to finish and apply changes." << std::endl;        
        in.get();

        //reset the calibration values
        CalibrationActionMsg resetCal;
        resetCal.msgId = CID_CAL_ACTION;
        resetCal.action = CAL_ACC_RESET;
        seatrac.send(sizeof(resetCal), (const uint8_t*)&resetCal);
        sleep_for(nanoseconds(2000));

        //print cal values and wait for user to finish calibration procedure
        turnOnAccCalFeedback(seatrac);
        sleep_for(nanoseconds(2000));
        in.get();
        turnOffCalFeedback(seatrac);
        sleep_for(nanoseconds(2000));

        //calculate the new calibration parameters & save to seatrac RAM
        CalibrationActionMsg calculateCal;
        calculateCal.msgId = CID_CAL_ACTION;
        calculateCal.action = CAL_ACC_CALC;
        seatrac.send(sizeof(calculateCal), (const uint8_t*)&calculateCal);
        out << "Calibration values calculated and saved to RAM." << std::endl;
        sleep_for(nanoseconds(2000));

        //save the calibration settings to perminant EEPROM
        if(saveToEEPROM) {
            messages::SettingsSave::Request saveSettings;
            seatrac.send(sizeof(saveSettings), (const uint8_t*)&saveSettings);
            out << "Calibration values saved to EEPROM." << std::endl;
            sleep_for(nanoseconds(2000));
        } else {
            out << "Calibration values have not been saved to EEPROM." << std::endl;
        }

        out << "Accelerometer calibration complete. Settings saved." << std::endl;

    }


    inline void calibrateMagnetometer(SeatracDriver& seatrac, std::ostream& out, std::istream& in, bool saveToEEPROM = false) {
        turnOffCalFeedback(seatrac);
        sleep_for(nanoseconds(2000));
        out << "---\tSeatrac Magnetometer Calibration\t---" << std::endl
            << "Press enter to begin. Once the progress has reached 100%%, press enter again to finish and apply changes." << std::endl;        
        in.get();

       //reset the calibration values
        CalibrationActionMsg resetCal;
        resetCal.msgId = CID_CAL_ACTION;
        resetCal.action = CAL_MAG_RESET;
        seatrac.send(sizeof(resetCal), (const uint8_t*)&resetCal);
        sleep_for(nanoseconds(2000));

        //print cal values and wait for user to finish calibration procedure
        turnOnMagCalFeedback(seatrac);
        sleep_for(nanoseconds(2000));
        in.get();
        turnOffCalFeedback(seatrac);
        sleep_for(nanoseconds(2000));

        //calculate the new calibration parameters & save to seatrac RAM
        CalibrationActionMsg calculateCal;
        calculateCal.msgId = CID_CAL_ACTION;
        calculateCal.action = CAL_MAG_CALC;
        seatrac.send(sizeof(calculateCal), (const uint8_t*)&calculateCal);
        out << "Calibration values calculated and saved to RAM." << std::endl;
        sleep_for(nanoseconds(2000));

        //save the calibration settings to perminant EEPROM
        if(saveToEEPROM) {
            messages::SettingsSave::Request saveSettings;
            seatrac.send(sizeof(saveSettings), (const uint8_t*)&saveSettings);
            out << "Calibration values saved to EEPROM." << std::endl;
            sleep_for(nanoseconds(2000));
        } else {
            out << "Calibration values have not been saved to EEPROM." << std::endl;
        }

        out << "Magnetometer calibration complete. Settings saved." << std::endl;

    }

}; //namespace calibration
}; //namespace seatrac
}; //namespace narval

#endif //_DEF_SEATRAC_DRIVER_CALIBRATION_H_
