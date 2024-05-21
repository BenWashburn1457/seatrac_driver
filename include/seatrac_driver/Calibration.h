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
#include <seatrac_driver/messages/MessageBase.h>
#include <seatrac_driver/commands.h>

using namespace narval::seatrac;
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds

/*
    Contains helper functions to quickly calibrate the acoustic beacon.
*/

namespace narval { namespace seatrac { namespace calibration {


    struct CalAction : public messages::Message<CalAction>{

        using Message<CalAction>::operator=;

        static const CID_E Identifier = CID_CAL_ACTION;
        struct Request : public messages::Message<Request>{
            static const CID_E Identifier = CID_CAL_ACTION;
            CAL_ACTION_E action;
        }__attribute__((packed));

        CST_E status;

    }__attribute__((packed));

    inline std::ostream& operator<<(std::ostream& os,
                                const narval::seatrac::calibration::CalAction& msg)
    {
        os << "CalAction : " << msg.status << std::endl;
        return os;
    }

    // inline bool turnOnAccCalFeedback(SeatracDriver& seatrac, 
    //                     STATUS_BITS_E prevStatusBits=static_cast<STATUS_BITS_E>(0x0), 
    //                     STATUSMODE_E statusMode=STATUS_MODE_10HZ) {

    //     command::status_config_set()

    //     messages::StatusConfigSet::Request statusSet;
        
    //     statusSet.statusOutput = ACC_CAL | AHRS_RAW_DATA | prevStatusBits; 
    //     statusSet.statusMode   = statusMode;

    //     messages::StatusConfigSet statusSetResponse;
    //     if (seatrac.send_request(sizeof(statusSet), (const uint8_t*)&statusSet, &statusSetResponse)) {
    //             if (statusSetResponse.statusCode == CST_OK) return true;
    //             else {
    //                 if (err_out_ptr != nullptr) 
    //                 *err_out_ptr << "Error sending status set request: " 
    //                              << statusSetResponse 
    //                              << std::endl;
    //                 return false;
    //             }
    //     } else {
    //          if (err_out_ptr != nullptr) *err_out_ptr << "Error sending status set request: Timeout reached" << std::endl;
    //          return false;
    //     }
    // }
    // inline bool turnOnMagCalFeedback(SeatracDriver& seatrac, 
    //                     STATUS_BITS_E prevStatusBits=static_cast<STATUS_BITS_E>(0x0), 
    //                     STATUSMODE_E statusMode=STATUS_MODE_10HZ,
    //                     std::ostream* err_out_ptr=nullptr) {
    //     messages::StatusConfigSet::Request statusSet;
        
    //     statusSet.statusOutput = MAG_CAL | AHRS_RAW_DATA | prevStatusBits; 
    //     statusSet.statusMode   = statusMode;

    //     messages::StatusConfigSet statusSetResponse;
    //     if (seatrac.send_request(sizeof(statusSet), (const uint8_t*)&statusSet, &statusSetResponse)) {
    //             if (statusSetResponse.statusCode == CST_OK) return true;
    //             else {
    //                 if (err_out_ptr != nullptr) 
    //                 *err_out_ptr << "Error sending status set request: " 
    //                              << statusSetResponse 
    //                              << std::endl;
    //                 return false;
    //             }
    //     } else {
    //          if (err_out_ptr != nullptr) *err_out_ptr << "Error sending status set request: Timeout reached" << std::endl;
    //          return false;
    //     }
    // }
    // inline bool turnOffCalFeedback(SeatracDriver& seatrac, 
    //                     STATUS_BITS_E prevStatusBits=static_cast<STATUS_BITS_E>(0x0),
    //                     STATUSMODE_E statusMode=STATUS_MODE_10HZ,
    //                     std::ostream* err_out_ptr=nullptr) {
    //     messages::StatusConfigSet::Request statusSet;
        
    //     statusSet.statusOutput = static_cast<STATUS_BITS_E>(prevStatusBits & ~ACC_CAL & ~MAG_CAL & ~AHRS_RAW_DATA); 
    //     statusSet.statusMode   = statusMode;

    //     messages::StatusConfigSet statusSetResponse;
    //     if (seatrac.send_request(sizeof(statusSet), (const uint8_t*)&statusSet, &statusSetResponse)) {
    //             if (statusSetResponse.statusCode == CST_OK) return true;
    //             else {
    //                 if (err_out_ptr != nullptr) 
    //                 *err_out_ptr << "Error sending status set request: " 
    //                              << statusSetResponse 
    //                              << std::endl;
    //                 return false;
    //             }
    //     } else {
    //          if (err_out_ptr != nullptr) *err_out_ptr << "Error sending status set request: Timeout reached" << std::endl;
    //          return false;
    //     }
    // }

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
    inline bool calibrateAccelerometer(SeatracDriver& seatrac, std::ostream& out, std::istream& in, bool saveToEEPROM = false) {
        
        command::status_config_set(seatrac, (STATUS_BITS_E)0x0);
        out << "---\tSeatrac Accelerometer Calibration\t---" << std::endl
            << "Press enter to begin. Once the X Y and Z limits are found, press enter again to finish and apply changes." << std::endl;        
        in.get();

        //reset the calibration values
        CalAction::Request resetCal;
        resetCal.action = CAL_ACC_RESET;
        CalAction resetCalResp;
        seatrac.send_request(sizeof(resetCal), (const uint8_t*)&resetCal, &resetCalResp);
        //sleep_for(milliseconds(5));

        //print cal values and wait for user to finish calibration procedure
        command::status_config_set(seatrac, ACC_CAL);
        sleep_for(milliseconds(5));
        in.get();
        command::status_config_set(seatrac, (STATUS_BITS_E)0x0);
        sleep_for(milliseconds(5));

        //calculate the new calibration parameters & save to seatrac RAM
        CalAction::Request calculateCal;
        calculateCal.action = CAL_ACC_CALC;
        CalAction calculateCalResp;
        if(seatrac.send_request(sizeof(calculateCal), (const uint8_t*)&calculateCal, &calculateCalResp)) {
            if(calculateCalResp.status == CST_OK) out << "Calibration values calculated and saved to RAM." << std::endl;
            else {
                out << "Error saving Calibration Values to RAM";// << calculateCalResp;
                return false;
            }
        } else {
            out << "Error saving Calibration Values: time out reached";
            return false;
        }

        //save the calibration settings to perminant EEPROM
        if(saveToEEPROM) {
            messages::SettingsSave::Request saveSettings;
            messages::SettingsSave saveSettingsResp;
            if(seatrac.send_request(sizeof(saveSettings), (const uint8_t*)&saveSettings, &saveSettingsResp)) {
                if(saveSettingsResp.statusCode == CST_OK) out << "Calibration values saved to EEPROM." << std::endl;
                else out << "Error saving Calibration Values to EEPROM";// << saveSettingsResp;
            } else {
                out << "Error saving Calibration Values: time out reached";
                return false;
            }
        } else {
            out << "Calibration values have not been saved to EEPROM." << std::endl;
            return false;
        }

        out << "Accelerometer calibration complete" << std::endl;
        return true;
    }


    inline void calibrateMagnetometer(SeatracDriver& seatrac, std::ostream& out, std::istream& in, bool saveToEEPROM = false) {
        command::status_config_set(seatrac, (STATUS_BITS_E)0x0);
        sleep_for(milliseconds(5));
        out << "---\tSeatrac Magnetometer Calibration\t---" << std::endl
            << "Press enter to begin. Once the progress has reached 100%, press enter again to finish and apply changes." << std::endl;        
        in.get();

       //reset the calibration values
        CalAction::Request resetCal;
        resetCal.action = CAL_MAG_RESET;
        seatrac.send(sizeof(resetCal), (const uint8_t*)&resetCal);

        //print cal values and wait for user to finish calibration procedure
        command::status_config_set(seatrac, MAG_CAL);
        in.get();
        command::status_config_set(seatrac, (STATUS_BITS_E)0x0);

        //calculate the new calibration parameters & save to seatrac RAM
        CalAction::Request calculateCal;
        calculateCal.action = CAL_MAG_CALC;
        seatrac.send(sizeof(calculateCal), (const uint8_t*)&calculateCal);
        out << "Calibration values calculated and saved to RAM." << std::endl;
        sleep_for(milliseconds(5));

        //save the calibration settings to perminant EEPROM
        if(saveToEEPROM) {
            messages::SettingsSave::Request saveSettings;
            seatrac.send(sizeof(saveSettings), (const uint8_t*)&saveSettings);
            out << "Calibration values saved to EEPROM." << std::endl;
            sleep_for(milliseconds(5));
        } else {
            out << "Calibration values have not been saved to EEPROM." << std::endl;
        }

        out << "Magnetometer calibration complete. Settings saved." << std::endl;

    }

}; //namespace calibration
}; //namespace seatrac
}; //namespace narval

#endif //_DEF_SEATRAC_DRIVER_CALIBRATION_H_
