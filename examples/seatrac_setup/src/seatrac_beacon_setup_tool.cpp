#include <iostream>

#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/messages/Messages.h>
#include <seatrac_driver/commands.h>
#include <seatrac_driver/Calibration.h>

using namespace std::chrono_literals;
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
                //std::cout << "Got message : " << msgId << std::endl << std::flush;
                break;

            case CID_STATUS: {
                messages::Status status;
                status = data;
                calibration::printCalFeedback(std::cout, status);
            } break;
        }
    }
};




void skip_cin_line() {
    while(std::cin.get() != '\n');
}
bool yn_answer() {
    while(true) {
        char yorn;
        if(scanf("%c", &yorn)) {
            if(yorn == 'y') {skip_cin_line(); return true;}
            if(yorn == 'n') {skip_cin_line(); return false;}
        }
        skip_cin_line();
        std::cout << "Invalid response. Please enter 'y' or 'n': ";
        }
}

int main(int argc, char *argv[]) {

    std::cout << "=== Seatrac Beacon Setup Tool ==="    << std::endl << std::endl;
            //   << "Procedure:"                           << std::endl
            //   << " - Connect to Beacon"                 << std::endl
            //   << " - Set Beacon Id"                     << std::endl
            //   << " - Set Beacon Settings"               << std::endl
            //   << " - Calibrate Magnetometer"            << std::endl;

    bool cont = true;
    while(cont) {
        std::cout << "Enter Serial Port (or blank for default '/dev/ttyUSB0'): ";
        char serial_port[20];
        fgets(serial_port, sizeof(serial_port), stdin);
        serial_port[strlen(serial_port)-1] = 0x00;
        if(strlen(serial_port) == 0) strcpy(serial_port, "/dev/ttyUSB0");

        {
        std::cout << "Connecting to Beacon... ";
        MyDriver seatrac(serial_port);
        SETTINGS_T origional_settings = command::settings_get(seatrac).settings;
        SETTINGS_T settings = origional_settings;
        command::status_config_set(seatrac, (STATUS_BITS_E)0x0); 
        std::cout << "Done" << std::endl;

        std::cout << "View current settings (y/n)? ";
        if(yn_answer()) std::cout << settings << std::endl << std::endl;

        // Change beacon id
        std::cout << "Current Beacon Id: " << (int)settings.xcvrBeaconId << std::endl
                  << "Change Beacon Id (y/n)? ";
        if(yn_answer()) {
            int bid;
            while(true) {
                std::cout << "Enter New Beacon Id (integer between 1 and 15 inclusive): ";
                if(scanf("%d", &bid) && bid<=15 && bid>=1) break;
                skip_cin_line();
                std::cout << "Invalid Beacon Id. Id should be an integer between 1 and 15 inclusive." << std::endl;
            }
            skip_cin_line();
            std::cout << "Setting Beacon Id to " << bid << "... ";
            settings.xcvrBeaconId = (BID_E) bid;
            command::settings_set(seatrac, settings);
            std::cout << "done" << std::endl << std::endl;
        }

        // Change Env Salinity Settings
        std::cout << "Current Water Salinity Setting: " << settings.envSalinity/10.0 << " ppt" << std::endl
                  << "Fresh water has salinity of 0 ppt. Salt water has salinity of 35 ppt." << std::endl
                  << "Change Water Salinity Setting (y/n)? ";
        if(yn_answer()) {
            float sal;
            while(true) {
                std::cout << "Enter New Salinity (float): ";
                if(scanf("%f", &sal)) break;
                skip_cin_line();
                std::cout << "Invalid Salinity. Salinity should be a float." << std::endl;
            }
            skip_cin_line();
            std::cout << "Setting Salinity to " << sal << " ppt... ";
            settings.envSalinity = (int)(sal*10);
            command::settings_set(seatrac, settings);
            std::cout << "done" << std::endl << std::endl;
        }

        // Change Status Report Settings
        std::cout << "View and modify serial report settings (y/n)? ";
        if(yn_answer()) {
            std::cout << "Current Status Report Frequency: " << settings.statusFlags << std::endl
                      << "Change Status Report frequency (y/n)? ";
            if(yn_answer()) {
                int val;
                while(true) {
                    std::cout << "Select Status frequency from list:" << std::endl
                            << "\t1) Manual - 0 Hz" << std::endl
                            << "\t2) 1 Hz" << std::endl
                            << "\t3) 2.5 Hz" << std::endl
                            << "\t4) 5 Hz" << std::endl
                            << "\t5) 10 Hz" << std::endl
                            << "\t6) 25 Hz" << std::endl
                            << "Enter a number from 1 to 6: ";
                    if(scanf("%d", &val) && val<=6 && val>=1) break;
                    skip_cin_line();
                    std::cout << "Invalid Selection. Options are from 1 to 6." << std::endl;
                }
                skip_cin_line();
                settings.statusFlags = (STATUSMODE_E)(val-1);
            }
            std::cout << "Current fields included in status output: " << settings.status_output << std::endl
                      << "Modify status output (y/n)? ";
            if(yn_answer()) {
                settings.status_output = (STATUS_BITS_E)0x00;
                std::cout << "Include temperature, pressure, depth and velocity in status report (y/n)? ";
                settings.status_output = settings.status_output | (STATUS_BITS_E)(yn_answer() * ENVIRONMENT);
                std::cout << "Include yaw pitch and roll in status report (y/n)? ";
                settings.status_output = settings.status_output | (STATUS_BITS_E)(yn_answer() * ATTITUDE);
                std::cout << "Include accelerometer, magnetometer, and gyroscope sensor values in status report (y/n)? ";
                settings.status_output = settings.status_output | (STATUS_BITS_E)(yn_answer() * AHRS_COMP_DATA);        
            }

            std::cout << "Transciever serial reports: " << std::endl
                      << "  - transceiver usbl messages: " << (bool)(settings.xcvrFlags & XCVR_USBL_MSGS) << std::endl
                      << "  - transceiver fix messages: " << (bool)(settings.xcvrFlags & XCVR_FIX_MSGS) << std::endl
                      << "  - transceiver diagnostic messages: " << (bool)(settings.xcvrFlags & XCVR_DIAG_MSGS) << std::endl
                      << "If true, these reports will be sent in addition to the higher protocol reports such as PING or DAT." << std::endl
                      << "Modify transceiver serial report settings (y/n)? ";
            if(yn_answer()) {
                settings.xcvrFlags = (XCVR_FLAGS_E)(settings.xcvrFlags & 0x1F);
                std::cout << "report transceiver usbl messages (y/n)? ";
                if(yn_answer()) settings.xcvrFlags = (XCVR_FLAGS_E)(settings.xcvrFlags | XCVR_USBL_MSGS);
                std::cout << "report transceiver fix messages (y/n)? ";
                if(yn_answer()) settings.xcvrFlags = (XCVR_FLAGS_E)(settings.xcvrFlags | XCVR_FIX_MSGS);
                std::cout << "report transceiver diagnostic messages (y/n)? ";
                if(yn_answer()) settings.xcvrFlags = (XCVR_FLAGS_E)(settings.xcvrFlags | XCVR_DIAG_MSGS);
            }

            std::cout << "Saving serial report settings" << std::endl;
            command::settings_set(seatrac, settings);
            std::cout << "done" << std::endl << std::endl;
        }

        std::cout << "View and modify transciever and sensor settings (y/n)? ";
        if(yn_answer()) {
            std::cout << "Current transceiver flags: " << settings.xcvrFlags << std::endl
                      << "Currently automatic mag calibration is set to " << (bool)settings.ahrsFlags
                      << "Modify transceiver flags and auto mag cal? ";
            if(yn_answer()) {
                settings.xcvrFlags = (XCVR_FLAGS_E)0x00;
                std::cout << "Use Mag, Accel and Gyro to calculate usbl position (y/n)? ";
                settings.xcvrFlags = (XCVR_FLAGS_E)(settings.xcvrFlags | yn_answer() * USBL_USE_AHRS);

                std::cout << "Automatic Mag Calibration (y/n)? ";
                settings.ahrsFlags = (AHRS_FLAGS_E)yn_answer();

                std::cout << "Saving settings... ";
                command::settings_set(seatrac, settings);
                std::cout << "done" << std::endl << std::endl;   
            }
        }

        // Other settings (ones that shouldn't change but good to check to make sure they're not corrupted)
        // settings.status_output = ENVIRONMENT | ATTITUDE;
        // settings.xcvrFlags     = (XCVR_FLAGS_E)(USBL_USE_AHRS | XCVR_POSFLT_ENABLE);
        // settings.envFlags      = (ENV_FLAGS_E)(AUTO_VOS | AUTO_PRESSURE_OFS);
        // settings.ahrsFlags     = AUTO_CAL_MAG;

        settings.xcvrRangeTmo  = 1000;
        settings.xcvrRespTime  = 10;
        settings.xcvrPosfltVel = 3;
        settings.xcvrPosfltAng = 10;
        settings.xcvrPosfltTmo = 60;

        command::settings_set(seatrac, settings);

        std::cout << "Calibrate Magnetometer (y/n)? ";
        if(yn_answer()) {
            calibration::calibrateMagnetometer(seatrac, std::cout, std::cin, false);
        }
        std::cout << "Calibrate Accelerometer (y/n)? ";
        if(yn_answer()) {
            calibration::calibrateAccelerometer(seatrac, std::cout, std::cin, false);
        }

        std::cout << "Beacon setup complete" << std::endl
                  << "Review changes to settings (y/n)? ";
        if(yn_answer()) {
            std::cout << std::endl << "== Origional Settings ==" << std::endl
                      << origional_settings << std::endl << std::endl
                      << "== New Settings ==" << std::endl
                      << command::settings_get(seatrac).settings << std::endl << std::endl;
        }

        std::cout << "Save Settings to permanent EEPROM memory? " << std::endl
                  << "If you chose not to, settings will still be saved in beacon ram. (y/n)? ";
        if(yn_answer()) {
            std::cout << "Saving Settings... ";
            command::settings_save(seatrac);
            std::cout << "done" << std::endl;
        }

        std::cout << std::endl << "Beacon setup complete" << std::endl;
        }

        std::cout << std::endl << "Setup another beacon (y/n)? ";
        cont = yn_answer();
    }
}


        // std::cout << "Beacon Settings:";
        // std::cout << "Change Beacon Settings (y/n): ";
        // std::cout << "Upload from config file (u) or enter manually (m)? ";
        // std::cout << "Path to Config File (blank for default './seatrac_logger_config.toml'): ";

