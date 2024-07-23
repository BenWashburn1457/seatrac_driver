
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <time.h>

#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/messages/Messages.h>
#include <seatrac_driver/commands.h>

#include "toml.hpp"

using namespace narval::seatrac;

void upload_config_settings(SeatracDriver& seatrac, std::string config_file_path="./seatrac_config.toml") {

    std::cout << "starting upload settings to seatrac beacon" << std::endl;

    auto config = toml::parse_file(config_file_path);
    auto seatrac_config = config["SeatracConfig"];

    SETTINGS_T settings = command::settings_get(seatrac).settings;
    std::cout << "retrieved current beacon settings. Old settings: " << std::endl
              << settings << std::endl;

    switch((int)(seatrac_config["status_report_frequency_hertz"].value_or(0.0)*10)) {
        case 0:   settings.statusFlags = STATUS_MODE_MANUAL; break;
        case 10:  settings.statusFlags = STATUS_MODE_1HZ;    break;
        case 25:  settings.statusFlags = STATUS_MODE_2HZ5;   break;
        case 50:  settings.statusFlags = STATUS_MODE_5HZ;    break;
        case 100: settings.statusFlags = STATUS_MODE_10HZ;   break;
        case 250: settings.statusFlags = STATUS_MODE_25HZ;   break;
        default:  std::cout << "Seatrac Config Error: value of status_report_frequency_hertz is invalid" << std::endl;
    };
    settings.status_output = (STATUS_BITS_E)(
          ENVIRONMENT    * seatrac_config["status_include_temp_pressure_depth_vos"].value_or(false)
        | ATTITUDE       * seatrac_config["status_include_yaw_pitch_roll"].value_or(false)
        | MAG_CAL        * seatrac_config["status_include_mag_cal_data"].value_or(false)
        | ACC_CAL        * seatrac_config["status_include_accel_cal_data"].value_or(false)
        | AHRS_RAW_DATA  * seatrac_config["status_include_uncompensated_accel_mag_gyro"].value_or(false)
        | AHRS_COMP_DATA * seatrac_config["status_include_compensated_accel_mag_gyro"].value_or(false)
    );

    settings.envFlags = (ENV_FLAGS_E)(
          AUTO_VOS          * seatrac_config["auto_calc_velocity_of_sound"].value_or(true)
        | AUTO_PRESSURE_OFS * seatrac_config["auto_calc_pressure_offset"].value_or(true)
    );
    // settings.envPressureOfs = 0; //value will be overwritten if auto_calc_pressure_offset is true
    // settings.envVos = 0; //value will be overwritten if auto_calc_velocity_of_sound is true

    settings.envSalinity = (uint8_t)(10*seatrac_config["env_salinity_ppt"].value_or(0.0));

    settings.ahrsFlags = (AHRS_FLAGS_E)seatrac_config["automatic_mag_calibration"].value_or(false);
    
    settings.ahrsYawOfs   = 0;
    settings.ahrsPitchOfs = 0;
    settings.ahrsRollOfs  = 0;

    XCVR_TXMSGCTRL_E msgctrl = 
    seatrac_config["transceiver_block_send_all"].value_or(false)? 
        XCVR_TXMSG_BLOCK_ALL
        : (seatrac_config["transceiver_block_send_response"].value_or(false)? 
            XCVR_TXMSG_BLOCK_RESP
            : XCVR_TXMSG_ALLOW_ALL);

    settings.xcvrFlags = (XCVR_FLAGS_E)(
          USBL_USE_AHRS      //* seatrac_config["usbl_use_AHRS"].value_or(true)
        | XCVR_POSFLT_ENABLE * seatrac_config["position_filter_enabled"].value_or(true)
        | XCVR_USBL_MSGS     * seatrac_config["report_transceiver_usbl_msgs"].value_or(false)
        | XCVR_FIX_MSGS      * seatrac_config["report_transceiver_fix_msgs"].value_or(false)
        | XCVR_DIAG_MSGS     * seatrac_config["report_transceiver_msg_diagnostics"].value_or(false)
        | (msgctrl << 3) //XCVR_TXMSGCTRL_E takes up the 3rd and 4th bits of 
    );

    settings.xcvrBeaconId = (BID_E)seatrac_config["beacon_id"].value_or(0);
    if(settings.xcvrBeaconId<1 || settings.xcvrBeaconId>15)
        throw; //"Seatrac Config Error: beacon_id must be between 0 and 15";

    settings.xcvrRangeTmo = (uint16_t)seatrac_config["transceiver_range_timeout_meters"].value_or(1000);
    if(settings.xcvrRangeTmo>3000 || settings.xcvrRangeTmo<1000)
        std::cout << "Seatrac Config Error: transceiver_range_timeout_meters must be between 1000 and 3000 m" << std::endl;
    settings.xcvrRespTime = (uint16_t)seatrac_config["transceiver_response_delay_milliseconds"].value_or(10);
    if(settings.xcvrRespTime>1000 || settings.xcvrRespTime<10)
        std::cout << "Seatrac Config Error: transceiver_response_delay_milliseconds must be between 10 and 1000 ms" << std::endl;

    settings.xcvrPosfltVel = (uint8_t)(seatrac_config["pos_filter_velocity_limit_meters_per_sec"].value_or(3));
    settings.xcvrPosfltAng = (uint8_t)(seatrac_config["pos_filter_angle_limit_degrees"].value_or(10));
    settings.xcvrPosfltTmo = (uint8_t)(seatrac_config["pos_filter_timeout_seconds"].value_or(60));

    std::cout << settings << std::endl;

    std::cout << "uploading settings to beacon";
    messages::SettingsSet resp = command::settings_set(seatrac, settings);
    if(resp.statusCode != CST_OK)
        throw "Error saving settings to seatrac beacon";

    return;
}


void download_config_settings(SeatracDriver& seatrac, std::string output_file) {

}


int main(int argc, char *argv[]) {

    std::string serial_port;
    if (argc == 1) { serial_port = "/dev/ttyUSB0"; }
    else { serial_port = argv[1]; }

    SeatracDriver seatrac(serial_port);

    upload_config_settings(seatrac);

    return 0;
}
