#ifndef _DEF_SEATRAC_DRIVER_CALIBRATION_H_
#define _DEF_SEATRAC_DRIVER_CALIBRATION_H_

#include <iostream>
#include <cstdio>
#include <seatrac_driver/SeatracTypes.h>
#include <seatrac_driver/messages/MessageBase.h>
#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/messages/Status.h>
#include <seatrac_driver/messages/StatusConfigSet.h>
#include <seatrac_driver/messages/MessageBase.h>
#include <seatrac_driver/commands.h>

using namespace narval::seatrac;

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

    
    //Call this function to calibrate the accelerometer.
    //Function walks user through a terminal calibration procedure
    //Ensure that printCalFeedback is being called in SeatracDriver::on_message
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

        //print cal values and wait for user to finish calibration procedure
        command::status_config_set(seatrac, ACC_CAL);
        in.get();
        command::status_config_set(seatrac, (STATUS_BITS_E)0x0);

        //calculate the new calibration parameters & save to seatrac RAM
        CalAction::Request calculateCal;
        calculateCal.action = CAL_ACC_CALC;
        CalAction calculateCalResp;
        if(seatrac.send_request(sizeof(calculateCal), (const uint8_t*)&calculateCal, &calculateCalResp)) {
            if(calculateCalResp.status == CST_OK) out << "Calibration values calculated and saved to RAM." << std::endl;
            else {
                out << "Error saving Calibration Values to RAM. Seatrac Response Status: " 
                << calculateCalResp.status;
                return false;
            }
        } else {
            out << "Error saving Calibration Values to RAM: time out reached";
            return false;
        }

        //save the calibration settings to perminant EEPROM
        if(saveToEEPROM) {
            messages::SettingsSave::Request saveSettings;
            messages::SettingsSave saveSettingsResp;
            if(seatrac.send_request(sizeof(saveSettings), (const uint8_t*)&saveSettings, &saveSettingsResp)) {
                if(saveSettingsResp.statusCode == CST_OK) out << "Calibration values saved to EEPROM." << std::endl;
                else {
                    out << "Error saving Calibration Values to EEPROM. Seatrac Response Status: " 
                        << saveSettingsResp.statusCode;
                    return false;
                }
            } else {
                out << "Error saving Calibration Values: time out reached";
                return false;
            }
        } else {
            out << "Calibration values have not been saved to EEPROM." << std::endl;
        }

        out << "Accelerometer calibration complete" << std::endl;
        return true;
    }

    //Call this function to calibrate the magnetometer.
    //Function walks user through a terminal calibration procedure
    //Ensure that printCalFeedback is being called in SeatracDriver::on_message
    inline bool calibrateMagnetometer(SeatracDriver& seatrac, std::ostream& out, std::istream& in, bool saveToEEPROM = false) {
        command::status_config_set(seatrac, (STATUS_BITS_E)0x0);
        out << "---\tSeatrac Magnetometer Calibration\t---" << std::endl
            << "Press enter to begin. Once the progress has reached 100%, press enter again to finish and apply changes." << std::endl;        
        in.get();

       //reset the calibration values
        CalAction::Request resetCal;
        resetCal.action = CAL_MAG_RESET;
        CalAction resetCalResp;
        seatrac.send_request(sizeof(resetCal), (const uint8_t*)&resetCal, &resetCalResp);

        //print cal values and wait for user to finish calibration procedure
        command::status_config_set(seatrac, MAG_CAL);
        in.get();
        command::status_config_set(seatrac, (STATUS_BITS_E)0x0);

        //calculate the new calibration parameters & save to seatrac RAM
        CalAction::Request calculateCal;
        calculateCal.action = CAL_MAG_CALC;
        CalAction calculateCalResp;
        if(seatrac.send_request(sizeof(calculateCal), (const uint8_t*)&calculateCal, &calculateCalResp)) {
            if(calculateCalResp.status == CST_OK) out << "Calibration values calculated and saved to RAM." << std::endl;
            else {
                out << "Error saving Calibration Values to RAM. Seatrac Response Status: " 
                    << calculateCalResp.status;
                return false;
            }
        } else {
            out << "Error saving Calibration Values to RAM: time out reached";
            return false;
        }

        //save the calibration settings to perminant EEPROM
        if(saveToEEPROM) {
            messages::SettingsSave::Request saveSettings;
            messages::SettingsSave saveSettingsResp;
            if(seatrac.send_request(sizeof(saveSettings), (const uint8_t*)&saveSettings, &saveSettingsResp)) {
                if(saveSettingsResp.statusCode == CST_OK) out << "Calibration values saved to EEPROM." << std::endl;
                else {
                    out << "Error saving Calibration Values to EEPROM. Seatrac Response Status: : " 
                    << saveSettingsResp.statusCode;
                    return false;
                }
            } else {
                out << "Error saving Calibration Values to EEPROM: time out reached";
                return false;
            }
        } else {
            out << "Calibration values have not been saved to EEPROM." << std::endl;
        }

        out << "Magnetometer calibration complete." << std::endl;
        return true;
    }

}; //namespace calibration
}; //namespace seatrac
}; //namespace narval

#endif //_DEF_SEATRAC_DRIVER_CALIBRATION_H_
