#ifndef _DEF_SEATRAC_DRIVER_COMMANDS_H_
#define _DEF_SEATRAC_DRIVER_COMMANDS_H_

#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/messages/Messages.h>

namespace narval { namespace seatrac { namespace command {

inline messages::SysAlive sys_alive(SeatracDriver& seatrac, int64_t timeout=1000)
{
    messages::SysAlive response;
    CID_E msgId = CID_SYS_ALIVE;

    if(!seatrac.send_request(1, (const uint8_t*)&msgId, &response, timeout)) {
        throw TimeoutReached();
    }

    return response;
}

inline messages::SysInfo sys_info(SeatracDriver& seatrac, int64_t timeout=1000)
{
    messages::SysInfo response;
    CID_E msgId = CID_SYS_INFO;

    if(!seatrac.send_request(1, (const uint8_t*)&msgId, &response, timeout)) {
        throw TimeoutReached();
    }

    return response;
}

inline messages::SysReboot sys_reboot(SeatracDriver& seatrac, int64_t timeout=1000)
{
    messages::SysReboot response;
    messages::SysReboot::Request request;

    if(!seatrac.send_request(sizeof(request), (const uint8_t*)&request, &response, timeout)) {
        throw TimeoutReached();
    }

    return response;
}

inline messages::StatusConfigGet status_config_get(SeatracDriver& seatrac, int64_t timeout=1000)
{
    messages::StatusConfigGet response;
    messages::StatusConfigGet::Request request;

    if(!seatrac.send_request(sizeof(request), (const uint8_t*)&request, &response, timeout)) {
        throw TimeoutReached();
    }

    return response;
}

inline messages::StatusConfigSet status_config_set(SeatracDriver& seatrac,
    STATUS_BITS_E statusOutput = ENVIRONMENT | ATTITUDE | MAG_CAL | ACC_CAL | AHRS_RAW_DATA,
    STATUSMODE_E  statusMode   = STATUS_MODE_10HZ,
    int64_t timeout=1000)
{
    messages::StatusConfigSet          response;
    messages::StatusConfigSet::Request request;

    //std::cout << "Filling parameters" << std::endl << std::flush;
    request.statusOutput = statusOutput;
    request.statusMode   = statusMode;

    //std::cout << "send request" << std::endl << std::flush;
    if(!seatrac.send_request(sizeof(request), (const uint8_t*)&request, &response, timeout)) {
        throw TimeoutReached();
    }
    //std::cout << "Got result" << std::endl << std::flush;

    return response;
}

inline messages::SettingsGet settings_get(SeatracDriver& seatrac, int64_t timeout=1000)
{
    messages::SettingsGet response;
    messages::SettingsGet::Request request;

    if(!seatrac.send_request(sizeof(request), (const uint8_t*)&request, &response, timeout)) {
        throw TimeoutReached();
    }

    return response;
}

inline messages::SettingsSet settings_set(SeatracDriver& seatrac, 
                                          SETTINGS_T settings,
                                          int64_t timeout=1000)
{
    messages::SettingsSet response;
    messages::SettingsSet::Request request;

    request.settings = settings;

    if(!seatrac.send_request(sizeof(request), (const uint8_t*)&request, &response, timeout)) {
        throw TimeoutReached();
    }

    return response;
}

inline messages::SettingsLoad settings_load(SeatracDriver& seatrac, int64_t timeout=1000)
{
    messages::SettingsLoad response;
    messages::SettingsLoad::Request request;

    if(!seatrac.send_request(sizeof(request), (const uint8_t*)&request, &response, timeout)) {
        throw TimeoutReached();
    }

    return response;
}

inline messages::SettingsSave settings_save(SeatracDriver& seatrac, int64_t timeout=1000)
{
    messages::SettingsSave response;
    messages::SettingsSave::Request request;

    if(!seatrac.send_request(sizeof(request), (const uint8_t*)&request, &response, timeout)) {
        throw TimeoutReached();
    }

    return response;
}

inline messages::SettingsReset settings_reset(SeatracDriver& seatrac, int64_t timeout=1000)
{
    messages::SettingsReset response;
    messages::SettingsReset::Request request;

    if(!seatrac.send_request(sizeof(request), (const uint8_t*)&request, &response, timeout)) {
        throw TimeoutReached();
    }

    return response;
}

inline bool set_beacon_id(SeatracDriver& seatrac, BID_E newId,
                          bool saveSettings = false, int64_t timeout=1000)
{
    auto settings = settings_get(seatrac, timeout).settings;
    settings.xcvrBeaconId = newId;
    auto setOk = settings_set(seatrac, settings, timeout);
    if(setOk.statusCode != CST_OK) {
        std::cerr << "Error while changing settings : " << setOk << std::endl;
        return false;
    }

    if(saveSettings) {
        auto saveOk = settings_save(seatrac, timeout);
        if(saveOk.statusCode != CST_OK) {
            std::cerr << "Error while saving settings : " << saveOk << std::endl;
            return false;
        }
    }

    return true;
}

inline messages::PingSend ping_send(SeatracDriver& seatrac, 
    BID_E      target,
    AMSGTYPE_E pingType = MSG_REQ, // MSG_REQ simple ping without USBL info
    int64_t timeout=1000)
{
    messages::PingSend response;
    messages::PingSend::Request request;

    request.target   = target;
    request.pingType = pingType;

    if(!seatrac.send_request(sizeof(request), (const uint8_t*)&request, &response, timeout)) {
        throw TimeoutReached();
    }

    return response;
}
inline messages::DataSend data_send(SeatracDriver& seatrac,
    BID_E      target,
    AMSGTYPE_E msgType,
    uint8_t    data_length,
    uint8_t*   data,
    int64_t    timeout=1000) 
{
    messages::DataSend response;
    messages::DataSend::Request request;

    request.destId     = target;
    request.msgType    = msgType;
    request.packetLen  = std::min(data_length, (uint8_t)sizeof(request.packetData));
    std::memcpy(request.packetData, data, request.packetLen);

    if(!seatrac.send_request(sizeof(request), (const uint8_t*)&request, &response, timeout)) {
        throw TimeoutReached();
    }

    return response;
}

inline messages::XcvrAnalyse background_noise(SeatracDriver& seatrac, int64_t timeout=1000)
{
    messages::XcvrAnalyse response;
    messages::XcvrAnalyse::Request request;

    if(!seatrac.send_request(sizeof(request), (const uint8_t*)&request, &response, timeout)) {
        throw TimeoutReached();
    }

    return response;
}

inline messages::XcvrStatus xcvr_status(SeatracDriver& seatrac, int64_t timeout=1000)
{
    messages::XcvrStatus response;
    messages::XcvrStatus::Request request;

    if(!seatrac.send_request(sizeof(request), (const uint8_t*)&request, &response, timeout)) {
        throw TimeoutReached();
    }

    return response;
}

}; //namespace command
}; //namespace seatrac
}; //namespace narval

#endif //_DEF_SEATRAC_DRIVER_COMMANDS_H_
