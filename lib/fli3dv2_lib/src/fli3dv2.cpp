/** Fli3dv2 - Library
 *
 * This library provides generic functionality for Fli3dv2:
 * - CCSDS packet handling
 * - ESP-NOW interfaces
 * - SerialTransfer interfaces
 * - C1101 radio interface
 * - TM/TC handling (including buffering)
 *
 */

#include <fli3dv2.h>
#include <fli3d_secrets.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

char buffer[BUFFER_MAX_SIZE];
SerialTransfer serialtransfer;
LinkedList<buffer_t*> *ccsds_rx_fifo = new LinkedList<buffer_t*>();
LinkedList<buffer_t*> *ccsds_tx_fifo = new LinkedList<buffer_t*>();
LinkedList<index_t*> *ccsds_fs_archive = new LinkedList<index_t*>();
index_t archive;
SmartRC_CC1101 radio;
//FtpServer wifiTCP_FTP;

// Functions not exposed in fli3dv2.h
extern bool save_config_bank (const uint8_t bank, const char* tag, const cfg_packet_t *cfg_ptr);
extern bool check_config_bank (const uint8_t bank);
extern bool clear_config_bank (const uint8_t bank);
extern bool set_boot_bank (const uint8_t bank);
extern bool set_parameter (const char* parameter, const char* value);
extern void OnDataSent_espnow(const esp_now_peer_info_t *info, esp_now_send_status_t status);
extern void OnDataRecv_espnow(const esp_now_peer_info_t *info, const uint8_t *espnow_rx_buffer, int len);
extern void register_espnow_peer (esp_now_peer_info_t peerInfo);
extern bool send_packet_through_espnow (ccsds_t* ccsds_ptr);
extern bool send_packet_through_serial (ccsds_t* ccsds_ptr);
extern bool flush_fs ();
extern void sync_archive_file ();
extern void move_archive_file ();
extern void set_next_archive_path (char* archive_path);
extern bool save_packet_to_archive (ccsds_t* ccsds_ptr);
extern bool send_packet_through_radio (ccsds_t* ccsds_ptr);
extern bool add_packet_to_memory_buffer (LinkedList<buffer_t*> *ccsds_fifo_ptr, ccsds_t* ccsds_ptr, uint8_t source, uint8_t comms);
extern buffer_t* get_packet_from_memory_buffer (LinkedList<buffer_t*> *ccsds_fifo_ptr, uint16_t index);
extern bool delete_packet0_from_memory_buffer (LinkedList<buffer_t*> *ccsds_fifo_ptr);
extern void update_packet (ccsds_t* ccsds_ptr);
extern void reset_packet (ccsds_t* ccsds_ptr);
extern bool get_routing(uint32_t *routing, uint8_t PID);
extern void set_routing(uint32_t *routing, uint8_t PID, bool status);
extern void init_ccsds_hdr (ccsds_t* ccsds_ptr, uint16_t APID, uint8_t pkt_type, uint16_t pkt_len);
extern bool valid_ccsds_hdr (ccsds_t* ccsds_ptr, bool pkt_type);
extern uint16_t get_ccsds_apid (ccsds_t* ccsds_ptr);
extern uint16_t get_ccsds_packet_ctr (ccsds_t* ccsds_ptr);
extern uint16_t get_ccsds_packet_len (ccsds_t* ccsds_ptr);
extern bool get_ccsds_packet_type (ccsds_t* ccsds_ptr);
extern uint32_t get_ccsds_seconds (ccsds_t* ccsds_ptr);
extern uint16_t get_ccsds_subseconds (ccsds_t* ccsds_ptr);
extern void set_ccsds_payload_len (ccsds_t* ccsds_ptr, uint16_t len);
extern uint8_t get_ccsds_pid (ccsds_t* ccsds_ptr);
extern bool execute_tc ();
extern bool cmd_reboot (const uint8_t system);
extern bool cmd_set_opsmode (const uint8_t opsmode);
extern bool cmd_set_parameter (const char* parameter, const char* value);
extern bool cmd_load_config (const uint8_t bank);
extern bool cmd_save_config (const uint8_t bank, const char* tag);
extern bool cmd_flush_fs ();
extern bool cmd_list_fs ();
extern void print_ccsds_data (ccsds_t* ccsds_ptr);

tc_packet_t         tc_esp32;
tc_packet_t         tc_esp32cam;
tc_packet_t         tc_gndctrl;
sts_packet_t        sts_esp32;
sts_packet_t        sts_esp32cam;
sts_packet_t        sts_gndctrl;
tm_esp32_t          tm_esp32;
tm_gps_t            tm_gps;
tm_motion_t         tm_motion;
tm_pressure_t       tm_pressure;
tm_radio_t          tm_radio;
tm_esp32cam_t       tm_esp32cam;
tm_camera_t         tm_camera;
tm_gndctrl_t        tm_gndctrl;
tmr_esp32_t         tmr_esp32;
tmr_esp32cam_t      tmr_esp32cam;
tmr_gndctrl_t       tmr_gndctrl;
cfg_packet_t        cfg_esp32;
cfg_packet_t        cfg_esp32cam;
cfg_packet_t        cfg_gndctrl;
var_t               var;


packet_properties_t packet[] = { // also update #define in fli3dv2.h and xtce.fli3d.xml
    // PID           APID  ccdsd_ptr                   size                       name               source       destination  type    subtype
    { TC_ESP32,        42, (ccsds_t*)&tc_esp32,        sizeof(tc_packet_t),       "tc_esp32",        SS_ANY,      SS_ESP32,    PKT_TC, PKT_TC },   // PID: 0
    { TC_ESP32CAM,     43, (ccsds_t*)&tc_esp32cam,     sizeof(tc_packet_t),       "tc_esp32cam",     SS_ANY,      SS_ESP32CAM, PKT_TC, PKT_TC },   // PID: 1 
    { TC_GNDCTRL,      44, (ccsds_t*)&tc_gndctrl,      sizeof(tc_packet_t),       "tc_gndctrl",      SS_ANY,      SS_GNDCTRL,  PKT_TC, PKT_TC },   // PID: 2
    { STS_ESP32,       45, (ccsds_t*)&sts_esp32,       sizeof(sts_packet_t),      "sts_esp32",       SS_ESP32,    SS_ANY,      PKT_TM, PKT_STS },   // PID: 3
    { STS_ESP32CAM,    46, (ccsds_t*)&sts_esp32cam,    sizeof(sts_packet_t),      "sts_esp32cam",    SS_ESP32CAM, SS_ANY,      PKT_TM, PKT_STS },   // PID: 4
    { STS_GNDCTRL,     47, (ccsds_t*)&sts_gndctrl,     sizeof(sts_packet_t),      "sts_gndctrl",     SS_GNDCTRL,  SS_ANY,      PKT_TM, PKT_STS },   // PID: 5
    { TM_ESP32,        48, (ccsds_t*)&tm_esp32,        sizeof(tm_esp32_t),        "tm_esp32",        SS_ESP32,    SS_ANY,      PKT_TM, PKT_TM },   // PID: 6
    { TM_GPS,          49, (ccsds_t*)&tm_gps,          sizeof(tm_gps_t),          "tm_gps",          SS_ESP32,    SS_ANY,      PKT_TM, PKT_TM },   // PID: 7
    { TM_MOTION,       50, (ccsds_t*)&tm_motion,       sizeof(tm_motion_t),       "tm_motion",       SS_ESP32,    SS_ANY,      PKT_TM, PKT_TM },   // PID: 8
    { TM_PRESSURE,     51, (ccsds_t*)&tm_pressure,     sizeof(tm_pressure_t),     "tm_pressure",     SS_ESP32,    SS_ANY,      PKT_TM, PKT_TM },   // PID: 9
    { TM_RADIO,        52, (ccsds_t*)&tm_radio,        sizeof(tm_radio_t),        "tm_radio",        SS_ESP32,    SS_ANY,      PKT_TM, PKT_TM },   // PID: 10
    { TM_ESP32CAM,     53, (ccsds_t*)&tm_esp32cam,     sizeof(tm_esp32cam_t),     "tm_esp32cam",     SS_ESP32CAM, SS_ANY,      PKT_TM, PKT_TM },   // PID: 11
    { TM_CAMERA,       54, (ccsds_t*)&tm_camera,       sizeof(tm_camera_t),       "tm_camera",       SS_ESP32CAM, SS_ANY,      PKT_TM, PKT_TM },   // PID: 12
    { TM_GNDCTRL,      55, (ccsds_t*)&tm_gndctrl,      sizeof(tm_gndctrl_t),      "tm_gndctrl",      SS_GNDCTRL,  SS_ANY,      PKT_TM, PKT_TM },   // PID: 13
    { TIMER_ESP32,     56, (ccsds_t*)&tmr_esp32,       sizeof(tmr_esp32_t),       "tmr_esp32",       SS_ESP32,    SS_ANY,      PKT_TM, PKT_TIMER },   // PID: 14
    { TIMER_ESP32CAM,  57, (ccsds_t*)&tmr_esp32cam,    sizeof(tmr_esp32cam_t),    "tmr_esp32cam",    SS_ESP32CAM, SS_ANY,      PKT_TM, PKT_TIMER },   // PID: 15
    { TIMER_GNDCTRL,   58, (ccsds_t*)&tmr_gndctrl,     sizeof(tmr_gndctrl_t),     "tmr_gndctrl",     SS_GNDCTRL,  SS_ANY,      PKT_TM, PKT_TIMER },   // PID: 16
    { CONFIG_ESP32,    59, (ccsds_t*)&cfg_esp32,       sizeof(cfg_packet_t),      "cfg_esp32",       SS_ESP32,    SS_ANY,      PKT_TM, PKT_CONFIG },   // PID: 17
    { CONFIG_ESP32CAM, 60, (ccsds_t*)&cfg_esp32cam,    sizeof(cfg_packet_t),      "cfg_esp32cam",    SS_ESP32CAM, SS_ANY,      PKT_TM, PKT_CONFIG },   // PID: 18
    { CONFIG_GNDCTRL,  61, (ccsds_t*)&cfg_gndctrl,     sizeof(cfg_packet_t),      "cfg_gndctrl",     SS_GNDCTRL,  SS_ANY,      PKT_TM, PKT_CONFIG }    // PID: 19
};

name_t subsystem[] = { // also update #define in fli3dv2.h and xtce.fli3d.xml
    { SS_POWER,      "power" },           // ID: 0 
    { SS_ESP32,      "esp32" },           // ID: 1
    { SS_GPS,        "neo6mv2 gps" },     // ID: 2
    { SS_MOTION,     "icm20948 motion" }, // ID: 3
    { SS_PRESSURE,   "bmp388 pressure" }, // ID: 4
    { SS_RADIO,      "cc1101 radio" },    // ID: 5
    { SS_SEPARATION, "separation" },      // ID: 6
    { SS_PARACHUTE,  "parachute" },       // ID: 7
    { SS_BUZZER,     "buzzer" },          // ID: 8
    { SS_ESP32CAM,   "esp32cam" },        // ID: 9
    { SS_CAMERA,     "ov2640 camera" },   // ID: 10
    { SS_FS,         "little_fs" },       // ID: 11
    { SS_SD,         "SD" },              // ID: 12
    { SS_ESPNOW,     "ESP-NOW" },         // ID: 13
    { SS_SERIAL,     "Serial" },          // ID: 14
    { SS_TCTM,       "TC/TM" },           // ID: 15
    { SS_ARCHIVE,    "archive" },         // ID: 16
    { SS_TIMER,      "timer" },           // ID: 17
    { SS_CONFIG,     "config" },          // ID: 18
    { SS_GNDCTRL,    "gndctrl" },         // ID: 19
    { SS_YAMCS,      "yamcs" },           // ID: 20
    { SS_ANY,        "any" },             // ID: 21
    { SS_NONE,       "none" }             // ID: 22
};

name_t command[] = { // also update #define in fli3dv2.h and xtce.fli3d.xml
    { CMD_REBOOT,        "reboot" },        // ID: 0
    { CMD_SET_OPSMODE,   "set_opsmode" },   // ID: 1
    { CMD_SET_PARAMETER, "set_parameter" }, // ID: 2
    { CMD_LOAD_CONFIG,   "load_config" },   // ID: 3
    { CMD_SAVE_CONFIG,   "save_config" },   // ID: 4
    { CMD_FLUSH_FS,      "flush_fs" },      // ID: 5
    { CMD_LIST_FS,       "list_fs" }        // ID: 6
};

name_t event[] = { // also update #define in fli3dv2.h and xtce.fli3d.xml
    { EVENT_INIT,     "init" },     // ID: 0
    { EVENT_INFO,     "info" },     // ID: 1
    { EVENT_WARNING,  "warning" },  // ID: 2
    { EVENT_ERROR,    "error" },    // ID: 3
    { EVENT_CMD,      "cmd" },      // ID: 4
    { EVENT_CMD_ACK,  "cmd_ack" },  // ID: 5
    { EVENT_CMD_RESP, "cmd_resp" }, // ID: 6
    { EVENT_CMD_FAIL, "cmd_fail" }  // ID: 7
};

name_t comms[] = { // also update #define in fli3dv2.h and xtce.fli3d.xml
    { COMMS_WIFI,   "wifi" },      // ID: 0
    { COMMS_ESPNOW, "esp-now" },   // ID: 1
    { COMMS_SERIAL, "serial" },    // ID: 2  
    { COMMS_RADIO,  "radio" },     // ID: 3
    { COMMS_FS,     "little_fs" }, // ID: 4
    { COMMS_SD,     "sd" },        // ID: 5
    { COMMS_ANY,    "any" }        // ID: 6
};

name_t opsmode[] = { // also update #define in fli3dv2.h and xtce.fli3d.xml
    { MODE_INIT,        "init" },       // ID: 0
    { MODE_CHECKOUT,    "checkout" },   // ID: 1
    { MODE_NOMINAL,     "nominal" },    // ID: 2
    { MODE_MAINTENANCE, "maintenance" } // ID: 3
};

name_t filesystem[] = { // also update #define in fli3dv2.h and xtce.fli3d.xml
    { FS_NONE,     "none" },  // ID: 0
    { FS_LITTLEFS, "flash" }, // ID: 1
    { FS_SD_MMC,   "SD" },    // ID: 2
    { FS_EEPROM,   "EEPROM" } // ID: 3
};


//const char stateName[4][10] =             
//			{ "static", "thrust", "freefall", "parachute" };
//const char cameraModeName[4][7] =         
//			{ "init", "idle", "single", "stream" };
//const char cameraResolutionName[11][10] = 
//			{ "160x120", "invalid1", "invalid2", "240x176", "320x240", "400x300", "640x480", "800x600", "1024x768", "1280x1024", "1600x1200" };
//const char gpsStatusName[9][11] =         
//			{ "none", "est", "time_only", "std", "dgps", "rtk_float", "rtk_fixed", "status_pps", "waiting" }; 


// Configuration Functionality

void init_config () {
    // Set default configuration values
    cfg_this->wifi_channel = default_wifi_channel;
    strcpy(cfg_this->rocket_name, default_rocket_name); 
    strcpy(cfg_this->password, default_password); 
    cfg_this->radio_baud = 2000;
    cfg_this->serial_baud = 115200;
    cfg_this->ota_enable = true;
    cfg_this->espnow_longrange = false;   
    cfg_this->battery_voltage_min = 3300; // mV
    cfg_this->battery_voltage_max = 4200; // mV 
    switch(SS_THIS) {
    case SS_ESP32:
        cfg_this->magic_number = 'e';
        cfg_this->target_opsmode = MODE_CHECKOUT;
        memcpy(&cfg_this->my_mac, &default_mac_esp32, 6);
        memcpy(&cfg_this->peer_mac, &default_mac_gndctrl, 6);  
        cfg_this->wifi_ap_enable = false;
        cfg_this->wifi_sta_enable = true;
        cfg_this->espnow_rx_enable = true;
        cfg_this->espnow_tx_enable = true;
        cfg_this->serial_rx_enable = true;
        cfg_this->serial_tx_enable = true;
        cfg_this->radio_rx_enable = false;
        cfg_this->radio_tx_enable = false;
        cfg_this->espnow_broadcast = false;
        cfg_this->archive_enable = true;
        cfg_this->espnow_buffer_enable = true;
        cfg_this->serial_buffer_enable = false;
        cfg_this->radio_buffer_enable = false;
        cfg_this->archive_buffer_enable = true;
        cfg_this->fs_enable = false;
        cfg_this->flush_fs_enable = true;
        cfg_this->sd_enable = false;
        cfg_this->ftp_enable = true;
        cfg_this->ftp_fs = FS_LITTLEFS;
        cfg_this->archive_fs = FS_LITTLEFS;
        cfg_esp32.pressure_tm_rate = 1;
        cfg_esp32.motion_tm_rate = 1;
        cfg_esp32.gps_tm_rate = 1;
        cfg_esp32.mpu_accel_sensitivity = 595;
        cfg_esp32.mpu_accel_offset_x = 0;
        cfg_esp32.mpu_accel_offset_y = 0;
        cfg_esp32.mpu_accel_offset_z = 0;
        cfg_esp32.pressure_enable = true;
        cfg_esp32.motion_enable = true;
        cfg_esp32.gps_enable = true;
        tm_motion.accel_range = 3;
        tm_motion.gyro_range = 3;
        break;
    case SS_ESP32CAM:
        cfg_esp32cam.magic_number = 'c';
        cfg_esp32cam.target_opsmode = MODE_CHECKOUT;
        memcpy(&cfg_esp32cam.my_mac, &default_mac_esp32cam, 6);
        memcpy(&cfg_esp32cam.peer_mac, &default_mac_gndctrl, 6);  
        cfg_esp32cam.wifi_ap_enable = false;
        cfg_esp32cam.wifi_sta_enable = true;
        cfg_esp32cam.espnow_rx_enable = false;
        cfg_esp32cam.espnow_tx_enable = false;        
        cfg_esp32cam.serial_rx_enable = true;
        cfg_esp32cam.serial_tx_enable = true;
        cfg_esp32cam.espnow_broadcast = false;
        cfg_esp32cam.archive_enable = true;
        cfg_esp32cam.espnow_buffer_enable = false;
        cfg_esp32cam.serial_buffer_enable = true;
        cfg_esp32cam.archive_buffer_enable = false;
        cfg_esp32cam.fs_enable = false;
        cfg_esp32cam.flush_fs_enable = false;
        cfg_esp32cam.sd_enable = true;
        cfg_esp32cam.ftp_enable = true;
        cfg_esp32cam.ftp_fs = FS_SD_MMC;
        cfg_esp32cam.archive_fs = FS_SD_MMC;
        cfg_esp32cam.camera_enable = true;
        break;
    case SS_GNDCTRL:
        cfg_this->magic_number = 'g';
        cfg_this->target_opsmode = MODE_NOMINAL;
        memcpy(&cfg_this->my_mac, &default_mac_gndctrl, 6);
        memcpy(&cfg_this->peer_mac, &default_mac_esp32, 6);  
        cfg_this->wifi_ap_enable = false;
        cfg_this->wifi_sta_enable = true;
        cfg_this->espnow_rx_enable = true;
        cfg_this->espnow_tx_enable = true;
        cfg_this->serial_rx_enable = true;
        cfg_this->serial_tx_enable = true;
        cfg_this->radio_rx_enable = false;
        cfg_this->radio_tx_enable = false;
        cfg_this->espnow_broadcast = false;
        cfg_this->archive_enable = false;
        cfg_this->espnow_buffer_enable = false;
        cfg_this->serial_buffer_enable = false;
        cfg_this->radio_buffer_enable = false;
        cfg_this->archive_buffer_enable = false;
        cfg_this->fs_enable = false;
        cfg_this->flush_fs_enable = false;
        cfg_this->sd_enable = false;
        cfg_this->ftp_enable = false;
        cfg_this->ftp_fs = FS_NONE;
        cfg_this->archive_fs = FS_NONE;
        break;
    }
	for(uint8_t PID=0; PID<NUMBER_OF_PID; PID++) {
	    set_routing(&cfg_this->routing_espnow, PID, default_routing_espnow[PID]);
	    set_routing(&cfg_this->routing_serial, PID, default_routing_serial[PID]);
	    set_routing(&cfg_this->routing_radio, PID, default_routing_radio[PID]);
	    set_routing(&cfg_this->routing_archive, PID, default_routing_archive[PID]);
	}
}

bool init_boot_config () {
    cfg_boot_t stored_cfg_boot;

    // Check that EEPROM is correctly sized for boot configuration table and configuration tables, and addressable
    if(cfg_this->cfg_boot.size_cfg_boot < sizeof(cfg_boot_t)) {
        sprintf (buffer, "EEPROM bank for boot configuration table is too small (%u bytes, expected %u). Please correct code.", cfg_this->cfg_boot.size_cfg_boot, sizeof(cfg_boot_t));
        publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
        return false;
    }
    if(cfg_this->cfg_boot.size_config < sizeof(cfg_packet_t)) {
        sprintf (buffer, "EEPROM bank for configuration tables is too small (%u bytes, expected %u). Please correct code.", cfg_this->cfg_boot.size_config, sizeof(cfg_packet_t));
        publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
        return false;
    }
    if(!EEPROM.begin(cfg_this->cfg_boot.size_cfg_boot + 3 * cfg_this->cfg_boot.size_config)) {
        sprintf (buffer, "Cannot address %u bytes in %s EEPROM", cfg_this->cfg_boot.size_cfg_boot + 3 * cfg_this->cfg_boot.size_config, subsystem[SS_THIS].name);
        publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
        return false;    
    }
    // Load boot configuration table from EEPROM and check that it is valid (correct magic number and version)
    EEPROM.get(0, stored_cfg_boot);
    if (stored_cfg_boot.magic_number == cfg_this->cfg_boot.magic_number) {
        if (stored_cfg_boot.version == cfg_this->cfg_boot.version) {
            load_boot_config();
            // Check validity of the stored configurations
            return (check_config_bank(1) && check_config_bank(2) && check_config_bank(3));
        }
        else {
            sprintf(buffer, "Boot configuration table in %s EEPROM has wrong version (%u, expected %u); re-initializing", subsystem[SS_THIS].name, stored_cfg_boot.version, cfg_this->cfg_boot.version);
        }
    }
    else {
        sprintf(buffer, "No valid boot configuration table found in %s EEPROM (magic number %u, expected %u); re-initializing", subsystem[SS_THIS].name, (uint8_t)stored_cfg_boot.magic_number, (uint8_t)cfg_this->cfg_boot.magic_number);
    }
    publish_event(STS_THIS, SS_THIS, EVENT_ERROR, buffer);
    // Re-initialize boot configuration table to default values
    clear_config_bank(1);
    clear_config_bank(2);
    clear_config_bank(3);
    set_boot_bank(0);
    return false;
}

bool load_boot_config () {
    // Load boot configuration table from EEPROM
    EEPROM.get(0, cfg_this->cfg_boot);
    sprintf(buffer, "Configurations found in %s EEPROM banks: 1:%s 2:%s 3:%s, %u selected", subsystem[SS_THIS].name, cfg_this->cfg_boot.cfg_name[0], cfg_this->cfg_boot.cfg_name[1], cfg_this->cfg_boot.cfg_name[2], cfg_this->cfg_boot.boot_bank);              
    publish_event(STS_THIS, SS_THIS, EVENT_INIT, buffer);
    return true;        
}

bool save_boot_config () {
    // Save boot configuration table to EEPROM    
    EEPROM.put(0, cfg_this->cfg_boot);
    if(!EEPROM.commit()) {
        sprintf(buffer, "Failed to commit boot configuration table to %s EEPROM", subsystem[SS_THIS].name);
        publish_event(STS_THIS, SS_THIS, EVENT_ERROR, buffer);
        return false;
    }
    cfg_boot_t stored_cfg_boot;
    EEPROM.get(0, stored_cfg_boot);
    if(memcmp(&stored_cfg_boot, &cfg_this->cfg_boot, sizeof(cfg_boot_t)) != 0) {
        sprintf(buffer, "Verification of boot configuration table in %s EEPROM failed", subsystem[SS_THIS].name);
        publish_event(STS_THIS, SS_THIS, EVENT_ERROR, buffer);
        return false;
    }
    sprintf(buffer, "Saved boot configuration table to %s EEPROM", subsystem[SS_THIS].name);              
    publish_event(STS_THIS, SS_THIS, EVENT_INFO, buffer);
    return true;        
}

bool clear_config_bank (const uint8_t bank) {
    cfg_packet_t empty_config = {};
    if (bank>0 and bank<4) {
        sprintf(buffer, "Clearing configuration in EEPROM bank %u", bank);              
        publish_event(STS_THIS, SS_THIS, EVENT_INFO, buffer);
        save_config_bank(bank, "empty", &empty_config);  
        return true;
    }
    return false;
}

bool set_boot_bank (uint8_t bank) {
    cfg_this->cfg_boot.boot_bank = bank;
    save_boot_config();
    if(bank==0) {
        sprintf (buffer, "Set %s to boot with default configuration", subsystem[SS_THIS].name);
    }
    else {
        sprintf (buffer, "Set %s to boot with configuration from bank %u (%s)", subsystem[SS_THIS].name, bank, cfg_this->cfg_boot.cfg_name[bank-1]);
    }
    publish_event (STS_THIS, SS_THIS, EVENT_INFO, buffer);    
    return true;
}

bool check_config_bank (const uint8_t bank) { 
    cfg_packet_t stored_config = {};
    if (bank>0 and bank<4) {
        EEPROM.get(cfg_this->cfg_boot.size_cfg_boot + (bank-1) * cfg_this->cfg_boot.size_config, stored_config);
        if (stored_config.magic_number == cfg_this->magic_number or stored_config.magic_number == 0) {
            if (stored_config.version == cfg_this->version or stored_config.version == 0) {
                return true;
            }
            else {
                sprintf(buffer, "Configuration '%s' in %s EEPROM bank %u has wrong version (%u, expected %u)", cfg_this->cfg_boot.cfg_name[bank-1], subsystem[SS_THIS].name, bank, stored_config.version, cfg_this->version);              
            }
        }
        else {
            sprintf(buffer, "No configuration found in %s EEPROM bank %u (magic number %u, expected %u)", subsystem[SS_THIS].name, bank, (uint8_t)stored_config.magic_number, (uint8_t)cfg_this->magic_number);
        } 
        publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
        clear_config_bank(bank);
    }
    return false;
}

bool load_config_bank (const uint8_t bank) {
    cfg_packet_t stored_config = {};
    cfg_boot_t preserved_cfg_boot = {};
    if (bank>0 and bank<4) {
        EEPROM.get(cfg_this->cfg_boot.size_cfg_boot + (bank-1) * cfg_this->cfg_boot.size_config, stored_config);
        if (stored_config.magic_number == cfg_this->magic_number) {
            if (stored_config.version == cfg_this->version) {
                memcpy(&preserved_cfg_boot, &cfg_this->cfg_boot, sizeof(cfg_boot_t)); // preserve boot configuration table
                EEPROM.get(cfg_this->cfg_boot.size_cfg_boot + (bank-1) * cfg_this->cfg_boot.size_config, *cfg_this);
                memcpy(&cfg_this->cfg_boot, &preserved_cfg_boot, sizeof(cfg_boot_t)); // restore boot configuration table
                sprintf(buffer, "Loaded configuration '%s' from %s EEPROM bank %u", cfg_this->cfg_boot.cfg_name[bank-1], subsystem[SS_THIS].name, bank);              
                publish_event(STS_THIS, SS_THIS, EVENT_INIT, buffer);
                publish_packet((ccsds_t*)cfg_this);
                return true;
            }
        }
    }
    sprintf(buffer, "Keeping default configuration, not loading from EEPROM bank");              
    publish_event(STS_THIS, SS_THIS, EVENT_INIT, buffer);
    return false;
}

bool save_config_bank (const uint8_t bank, const char* tag, const cfg_packet_t* cfg_ptr) {
    if (bank>0 and bank<4) {
        strcpy(cfg_this->cfg_boot.cfg_name[bank-1], tag);
        save_boot_config();
        EEPROM.put(cfg_this->cfg_boot.size_cfg_boot + (bank-1) * cfg_this->cfg_boot.size_config, *cfg_ptr);
        if(!EEPROM.commit()) {
            sprintf(buffer, "Failed to commit configuration to %s EEPROM bank %u", subsystem[SS_THIS].name, bank);
            publish_event(STS_THIS, SS_THIS, EVENT_ERROR, buffer);
            return false;
        }
        cfg_packet_t stored_config;
        EEPROM.get(cfg_this->cfg_boot.size_cfg_boot + (bank-1) * cfg_this->cfg_boot.size_config, stored_config);
        if(memcmp(&stored_config, cfg_ptr, sizeof(cfg_packet_t)) != 0) {
            sprintf(buffer, "Verification of configuration in %s EEPROM bank %u failed", subsystem[SS_THIS].name, bank);
            publish_event(STS_THIS, SS_THIS, EVENT_ERROR, buffer);
            return false;
        }
        sprintf(buffer, "Saved  configuration  to %s EEPROM bank %u", subsystem[SS_THIS].name, bank);              
        publish_event(STS_THIS, SS_THIS, EVENT_INFO, buffer);
        return true; 
    }
    return false;
}

/*void print_boot_config () { // TODO: delete
    cfg_boot_t stored_cfg_boot;
    EEPROM.get(0, stored_cfg_boot);
    Serial.print("Memory: ");
    Serial.println(get_hex_str((byte*)&cfg_this->cfg_boot, sizeof(cfg_boot_t)));
    Serial.print("Stored: ");
    Serial.println(get_hex_str((byte*)&stored_cfg_boot, sizeof(cfg_boot_t)));
}*/

/*void print_config_bank (uint8_t bank) { // TODO: delete
    cfg_packet_t stored_cfg;
    EEPROM.get(cfg_this->cfg_boot.size_cfg_boot + (bank-1) * cfg_this->cfg_boot.size_config, stored_cfg);
    Serial.print("Memory: ");
    Serial.println(get_hex_str((byte*)cfg_this, sizeof(cfg_packet_t)));
    Serial.print("Stored: ");
    Serial.println(get_hex_str((byte*)&stored_cfg, sizeof(cfg_packet_t)));
}*/


bool set_opsmode (const uint8_t mode) {
    
    if (mode == MODE_CHECKOUT or mode == MODE_NOMINAL or mode == MODE_MAINTENANCE) {
        if (tm_this->opsmode == MODE_MAINTENANCE and mode != MODE_MAINTENANCE) {
           // leaving maintenance mode, deactivate wifi and use ESPNOW again
           disable_wifi_services();
           tm_this->espnow_rx_enabled = true;
           tm_this->espnow_tx_enabled = true;
           sprintf (buffer, "Disabling WiFi services and reenabling ESPNOW");
           publish_event (STS_THIS, SS_THIS, EVENT_INFO, buffer);
        }
        sprintf (buffer, "Setting %s opsmode to '%s'", subsystem[SS_THIS].name, opsmode[mode].name);
        publish_event (STS_THIS, SS_THIS, EVENT_INFO, buffer);
        if (mode == MODE_MAINTENANCE and tm_this->opsmode != MODE_MAINTENANCE) {
           // entering maintenance mode, activate wifi and do not use ESPNOW
           enable_wifi_services();
           tm_this->espnow_rx_enabled = false;
           tm_this->espnow_tx_enabled = false;
           sprintf (buffer, "Enabling WiFi services and disabling ESPNOW");
           publish_event (STS_THIS, SS_THIS, EVENT_INFO, buffer);
        }
        tm_this->opsmode = mode;
    }
    else {
        sprintf (buffer, "Opsmode '%s' not known or not allowed", opsmode[mode].name);
        publish_event (STS_THIS, SS_THIS, EVENT_ERROR, buffer);
    }
    return true;
}

bool set_parameter (const char* parameter, const char* value) {
    bool success = false;
    char value_str[10];
    if (!strcmp(parameter, "boot_bank")) {
        if(atoi(value)>=0 and atoi(value)<4) {
            set_boot_bank(atoi(value)); 
            sprintf(value_str, "%u", atoi(value));
            success = true;
        }
    }
    else if (!strcmp(parameter, "wifi_channel")) {
        if(atoi(value)>0 and atoi(value)<14) {
            cfg_this->wifi_channel = atoi(value);
            sprintf(value_str, "%u", atoi(value));
            success = true;
        }
    }
    else if (!strcmp(parameter, "target_opsmode")) {
        if(atoi(value)>=0 and atoi(value)<4) {
            cfg_this->target_opsmode = atoi(value);
            strcpy(value_str, opsmode[atoi(value)].name);
            success = true;
        }
    }
    else if (!strcmp(parameter, "serial_baud")) {
        if(atoi(value)>0) {
            cfg_this->serial_baud = atoi(value);
            sprintf(value_str, "%u", atoi(value));
            success = true;
        }
    }
    else if (!strcmp(parameter, "radio_baud")) {
        if(atoi(value)>0) {
            cfg_this->radio_baud = atoi(value);
            sprintf(value_str, "%u", atoi(value));
            success = true;
        }
    }
    else if (!strcmp(parameter, "ftp_fs")) {
        if(atoi(value)>=0 and atoi(value)<4) {
            cfg_this->ftp_fs = atoi(value);
            strcpy(value_str, filesystem[atoi(value)].name);
            success = true;
        }
    }
    else if (!strcmp(parameter, "archive_fs")) {
        if(atoi(value)>=0 and atoi(value)<4) {
            cfg_this->archive_fs = atoi(value);
            strcpy(value_str, filesystem[atoi(value)].name);
            success = true;
        }
    }
    else if (!strcmp(parameter, "wifi_ap_enable")) {
        if(atoi(value)==-1) { value=cfg_this->wifi_ap_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->wifi_ap_enable = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }
    else if (!strcmp(parameter, "wifi_sta_enable")) {
        if(atoi(value)==-1) { value=cfg_this->wifi_sta_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->wifi_sta_enable = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }
    else if (!strcmp(parameter, "ftp_enable")) {
        if(atoi(value)==-1) { value=cfg_this->ftp_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->ftp_enable = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }
    else if (!strcmp(parameter, "ota_enable")) {
        if(atoi(value)==-1) { value=cfg_this->ota_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->ota_enable = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }
    else if (!strcmp(parameter, "espnow_longrange")) {
        if(atoi(value)==-1) { value=cfg_this->espnow_longrange?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->espnow_longrange = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");

            success = true;
        }
    }
    else if (!strcmp(parameter, "espnow_broadcast")) {
        if(atoi(value)==-1) { value=cfg_this->espnow_broadcast?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->espnow_broadcast = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }
    else if (!strcmp(parameter, "espnow_rx_enable")) {
        if(atoi(value)==-1) { value=cfg_this->espnow_rx_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->espnow_rx_enable = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }    
    else if (!strcmp(parameter, "espnow_tx_enable")) {
        if(atoi(value)==-1) { value=cfg_this->espnow_tx_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->espnow_tx_enable = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }
    else if (!strcmp(parameter, "espnow_buffer_enable")) {
        if(atoi(value)==-1) { value=cfg_this->espnow_buffer_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->espnow_buffer_enable = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }
    else if (!strcmp(parameter, "serial_rx_enable")) {
        if(atoi(value)==-1) { value=cfg_this->serial_rx_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->serial_rx_enable = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }
    else if (!strcmp(parameter, "serial_tx_enable")) {
        if(atoi(value)==-1) { value=cfg_this->serial_tx_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->serial_tx_enable = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }
    else if (!strcmp(parameter, "serial_buffer_enable")) {
        if(atoi(value)==-1) { value=cfg_this->serial_buffer_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->serial_buffer_enable = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }
    else if (!strcmp(parameter, "radio_rx_enable")) {
        if(atoi(value)==-1) { value=cfg_this->radio_rx_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->radio_rx_enable = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }    
    else if (!strcmp(parameter, "radio_tx_enable")) {
        if(atoi(value)==-1) { value=cfg_this->radio_tx_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->radio_tx_enable = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }
    else if (!strcmp(parameter, "radio_buffer_enable")) {
        if(atoi(value)==-1) { value=cfg_this->radio_buffer_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->radio_buffer_enable = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }
    else if (!strcmp(parameter, "archive_enable")) {
        if(atoi(value)==-1) { value=cfg_this->archive_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->archive_enable = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }
    else if (!strcmp(parameter, "archive_buffer_enable")) {
        if(atoi(value)==-1) { value=cfg_this->archive_buffer_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->archive_buffer_enable = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }
    else if (!strcmp(parameter, "fs_enable")) {
        if(atoi(value)==-1) { value=cfg_this->fs_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->fs_enable = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }
    else if (!strcmp(parameter, "flush_fs_enable")) {
        if(atoi(value)==-1) { value=cfg_this->flush_fs_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->flush_fs_enable = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }
    else if (!strcmp(parameter, "sd_enable")) {
        if(atoi(value)==-1) { value=cfg_this->sd_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->sd_enable = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }
    else if (!strcmp(parameter, "buzzer_enable")) {
        if(atoi(value)==-1) { value=cfg_this->buzzer_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_this->buzzer_enable = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }
    else if (!strcmp(parameter, "timestamp")) { 
        setTime(atoi(value));
        tm_this->time_set = true;
        var.delta_millis = millis()%1000;
        sprintf(value_str, "%04u-%02u-%02u %02u:%02u:%02u.%03u", year(atoi(value)), month(atoi(value)), day(atoi(value)), hour(atoi(value)), minute(atoi(value)), second(atoi(value)), var.delta_millis);
        success = true;
    }
    else if (!strcmp(parameter, "pressure_enable")) {
        if(atoi(value)==-1) { value=cfg_this->pressure_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_esp32.pressure_enable = atoi(value);
            tm_esp32.pressure_enabled = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }  
    else if (!strcmp(parameter, "motion_enable")) {
        if(atoi(value)==-1) { value=cfg_this->motion_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_esp32.motion_enable = atoi(value);
            tm_esp32.motion_enabled = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }  
    else if (!strcmp(parameter, "gps_enable")) {
        if(atoi(value)==-1) { value=cfg_this->gps_enable?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            cfg_esp32.gps_enable = atoi(value);
            tm_esp32.gps_enabled = atoi(value);
            sprintf(value_str, "%s", (atoi(value)==1)?"true":"false");
            success = true;
        }
    }      
    else if (!strcmp(parameter, "pressure_tm_rate")) {
        if(atoi(value)>=0 and atoi(value)<=255) {
            cfg_this->pressure_tm_rate = atoi(value);
            sprintf(value_str, "%u", atoi(value));
            var.pressure_interval = (1000 / cfg_esp32.pressure_tm_rate);
            success = true;
        }
    }  
    else if (!strcmp(parameter, "motion_tm_rate")) {
        if(atoi(value)>=0 and atoi(value)<=255) {
            cfg_this->motion_tm_rate = atoi(value);
            sprintf(value_str, "%u", atoi(value));
            var.motion_interval = (1000 / cfg_esp32.motion_tm_rate);
            success = true;
        }
    }  
    else if (!strcmp(parameter, "gps_tm_rate")) {
        //if(atoi(value)==1 or atoi(value)==5 or atoi(value)==10 or atoi(value)==16) {
        if(atoi(value)>=0 and atoi(value)<=255) {
            cfg_this->gps_tm_rate = atoi(value);
            sprintf(value_str, "%u", atoi(value));
            var.gps_interval = (1000 / cfg_esp32.gps_tm_rate);
            success = true;
        }
    }  
    else if (!strncmp(parameter, "routing_espnow", 14)) {
        const char* packet_name = &parameter[15];
        uint8_t PID=0;
        while(strcmp(packet[PID].name, packet_name) and PID<NUMBER_OF_PID) {
            PID++;
        }
        Serial.println(get_hex_str((byte*)&cfg_this->routing_espnow,4));
        Serial.print(PID);
        Serial.print(":");
        Serial.print(get_routing(&cfg_this->routing_espnow, PID));
        Serial.print("->");
        if(atoi(value)==-1) { value=get_routing(&cfg_this->routing_espnow, PID)?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            set_routing(&cfg_this->routing_espnow, PID, atoi(value));
            sprintf(value_str, "%u", atoi(value));
            success = true;
        }
        Serial.println(value);
        Serial.println(get_hex_str((byte*)&cfg_this->routing_espnow,4));
    }  
    else if (!strncmp(parameter, "routing_serial", 14)) {
        const char* packet_name = &parameter[15];
        uint8_t PID=0;
        while(strcmp(packet[PID].name, packet_name) and PID<NUMBER_OF_PID) {
            PID++;
        }
        Serial.println(get_hex_str((byte*)&cfg_this->routing_serial,4));
        Serial.print(PID);
        Serial.print(":");
        Serial.print(get_routing(&cfg_this->routing_serial, PID));
        Serial.print("->");
        if(atoi(value)==-1) { value=get_routing(&cfg_this->routing_serial, PID)?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            set_routing(&cfg_this->routing_serial, PID, atoi(value));
            sprintf(value_str, "%u", atoi(value));
            success = true;
        }
        Serial.println(value);
        Serial.println(get_hex_str((byte*)&cfg_this->routing_serial,4));
    }  
    else if (!strncmp(parameter, "routing_radio", 13)) {
        const char* packet_name = &parameter[14];
        uint8_t PID=0;
        while(strcmp(packet[PID].name, packet_name) and PID<NUMBER_OF_PID) {
            PID++;
        }
        Serial.println(get_hex_str((byte*)&cfg_this->routing_radio,4));
        Serial.print(PID);
        Serial.print(":");
        Serial.print(get_routing(&cfg_this->routing_radio, PID));
        Serial.print("->");
        if(atoi(value)==-1) { value=get_routing(&cfg_this->routing_radio, PID)?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            set_routing(&cfg_this->routing_radio, PID, atoi(value));
            sprintf(value_str, "%u", atoi(value));
            success = true;
        }
        Serial.println(value);
        Serial.println(get_hex_str((byte*)&cfg_this->routing_radio,4));
    }  
    else if (!strncmp(parameter, "routing_archive", 15)) {
        const char* packet_name = &parameter[16];
        uint8_t PID=0;
        while(strcmp(packet[PID].name, packet_name) and PID<NUMBER_OF_PID) {
            PID++;
        }
        Serial.println(get_hex_str((byte*)&cfg_this->routing_archive,4));
        Serial.print(PID);
        Serial.print(":");
        Serial.print(get_routing(&cfg_this->routing_archive, PID));
        Serial.print("->");
        if(atoi(value)==-1) { value=get_routing(&cfg_this->routing_archive, PID)?"0":"1"; }
        if(atoi(value)==0 or atoi(value)==1) {
            set_routing(&cfg_this->routing_archive, PID, atoi(value));
            sprintf(value_str, "%u", atoi(value));
            success = true;
        }
        Serial.println(value);
        Serial.println(get_hex_str((byte*)&cfg_this->routing_archive,4));
    }  
    if(success) {
        publish_packet((ccsds_t*)cfg_this);
        sprintf (buffer, "Set %s to %s", parameter, value_str);
    }
    else {
        sprintf (buffer, "Failed to set %s to %s", parameter, value);
    }
    return success; 
}

// WiFi Functionality

void setup_wifi () {
    // Setup wifi interface for use by ESPNOW or WiFi services (AP, STA, FTP, OTA)
    char hostname[64];
    sprintf(hostname, "%s-%s", cfg_this->rocket_name, subsystem[SS_THIS].name);
    //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
    if (cfg_this->wifi_ap_enable) {
        WiFi.mode(WIFI_AP_STA); 
    }
    else {
        WiFi.mode(WIFI_STA); 
    }
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
    WiFi.setHostname(hostname);
    WiFi.setSleep(false);
    //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1); 
}

bool setup_wifi_ap () {
    if (cfg_this->wifi_ap_enable) {
        byte my_mac[6];
        WiFi.softAP(cfg_this->rocket_name, cfg_this->password, cfg_this->wifi_channel);
        esp_wifi_set_channel(cfg_this->wifi_channel, WIFI_SECOND_CHAN_NONE);
        esp_wifi_get_mac(WIFI_IF_AP, my_mac);
        sprintf (buffer, "Started WiFi access point %s with IP %s (%02x:%02x:%02x:%02x:%02x:%02x) on channel %d", 
  	                      cfg_this->rocket_name, WiFi.softAPIP().toString().c_str(), my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5], 0);
  		publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer); 
        tm_this->wifi_ap_enabled = true;
  		return true; 
  	}
  	return false;
}

bool setup_wifi_sta () {
    // Connect to WiFi network, as defined in fli3d_secrets.h
    const unsigned long timeout_ms = 10000;
    const unsigned long start_ms = millis();
    if (cfg_this->wifi_sta_enable) {
        WiFi.begin(wifi_ssid, wifi_password);
        while (WiFi.status() != WL_CONNECTED && (millis() - start_ms) < timeout_ms) {
            delay(500);
        }

        if (WiFi.status() == WL_CONNECTED) {
            sprintf (buffer, "Connected to WiFi network %s with IP %s", wifi_ssid, WiFi.localIP().toString().c_str());
            publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
            tm_this->wifi_sta_enabled = true;
            cfg_this->wifi_my_ip[0] = WiFi.localIP()[0];
            cfg_this->wifi_my_ip[1] = WiFi.localIP()[1];
            cfg_this->wifi_my_ip[2] = WiFi.localIP()[2];
            cfg_this->wifi_my_ip[3] = WiFi.localIP()[3];
            publish_packet((ccsds_t*)cfg_this);
            return true;
        }

        sprintf (buffer, "Failed to connect to WiFi network %s within %lu ms", wifi_ssid, timeout_ms);
        publish_event (STS_THIS, SS_THIS, EVENT_WARNING, buffer);
        tm_this->wifi_sta_enabled = false;
        return false;
    }
    return false;
}

void enable_wifi_services () {
    // Set up wifi -> move to maintenance mode only
    setup_wifi_ap();
    setup_wifi_sta(); 

    // Initialize FTP server
    if (cfg_this->ftp_enable) {
        //tm_this->ftp_enabled = ftp_setup();
    }

    // Initialize OTA
    if (cfg_this->ota_enable) {
        setup_ota();
    }
}

void disable_wifi_services () {
    WiFi.disconnect(true, false);
    tm_this->wifi_ap_enabled = false;
    tm_this->wifi_sta_enabled = false;
    tm_this->ota_enabled = false;
    tm_this->ftp_enabled = false;
    cfg_this->wifi_my_ip[0] = 0;
    cfg_this->wifi_my_ip[1] = 0;
    cfg_this->wifi_my_ip[2] = 0;
    cfg_this->wifi_my_ip[3] = 0;
    publish_packet((ccsds_t*)cfg_this);

    setup_wifi(); // Reset WiFi for ESPNOW mode
    sprintf (buffer, "Disconnected from WiFi network and disabled WiFi services");
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
}

// ESP-NOW Functionality

void OnDataSent_espnow(const esp_now_peer_info_t *info, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
  	    sprintf(buffer, "ESP-NOW packet sent from %s to %s but not delivered", 
  	  	                 subsystem[SS_THIS].name,
                         subsystem[SS_ESPNOW_PEER].name);
  	  	publish_event(STS_THIS, SS_ESPNOW, EVENT_WARNING, buffer);
    }
}

void OnDataRecv_espnow(const esp_now_peer_info_t *info, const uint8_t *espnow_rx_buffer, int len) {
    if (cfg_this->espnow_rx_enable) {
		tm_this->espnow_rx_active = true;
        if(valid_ccsds_hdr((ccsds_t*)espnow_rx_buffer, PKT_TM) or valid_ccsds_hdr((ccsds_t*)espnow_rx_buffer, PKT_TC)) {
            tm_this->espnow_rx_pktrate++;
		    add_packet_to_memory_buffer(ccsds_rx_fifo, (ccsds_t*)espnow_rx_buffer, SS_ESPNOW_PEER, COMMS_ESPNOW);
        }
        else {
            sprintf(buffer, "ESP-NOW packet received from %02x:%02x:%02x:%02x:%02x:%02x with invalid CCSDS header (%02x %02x %02x %02x %02x %02x)", 
                            info->peer_addr[0],
                            info->peer_addr[1],
                            info->peer_addr[2],
                            info->peer_addr[3],
                            info->peer_addr[4],
                            info->peer_addr[5],
                            espnow_rx_buffer[0], 
                            espnow_rx_buffer[1], 
                            espnow_rx_buffer[2],
                            espnow_rx_buffer[3],
                            espnow_rx_buffer[4],
                            espnow_rx_buffer[5]);
            publish_event(STS_THIS, SS_ESPNOW, EVENT_WARNING, buffer);
        }
    }
}

bool setup_espnow () {
    byte my_mac[6];
    byte mac_broadcast[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    uint32_t espVersion;
    esp_now_peer_info_t peerInfo = {};
	
    if (cfg_this->espnow_rx_enable or cfg_this->espnow_tx_enable) {
        esp_wifi_set_ps(WIFI_PS_NONE);
        esp_wifi_set_channel(cfg_this->wifi_channel, WIFI_SECOND_CHAN_NONE);
        // Check my MAC address
        esp_wifi_get_mac(WIFI_IF_STA, my_mac);
        if (memcmp(&cfg_this->my_mac, &my_mac, 6)) {
            sprintf(buffer, "%s mac (%02x:%02x:%02x:%02x:%02x:%02x) does not match configured default (%02x:%02x:%02x:%02x:%02x:%02x); overriding default", 
                          subsystem[SS_THIS].name,
                          my_mac[0],
                          my_mac[1],
                          my_mac[2],
                          my_mac[3],
                          my_mac[4],
                          my_mac[5],
                          cfg_this->my_mac[0],
                          cfg_this->my_mac[1],
                          cfg_this->my_mac[2],
                          cfg_this->my_mac[3],
                          cfg_this->my_mac[4],
                          cfg_this->my_mac[5]);
            publish_event(STS_THIS, SS_ESPNOW, EVENT_WARNING, buffer);
            memcpy(&cfg_this->my_mac, &my_mac, 6);
        }
        // Check peer MAC address
        if (SS_OTHER==SS_GNDCTRL && memcmp(&cfg_this->peer_mac, &default_mac_gndctrl, 6)) {
            sprintf(buffer, "%s mac (%02x:%02x:%02x:%02x:%02x:%02x) from memory bank does not match default (%02x:%02x:%02x:%02x:%02x:%02x); using memory bank mac", 
                          subsystem[SS_OTHER].name,
                          cfg_this->peer_mac[0],
                          cfg_this->peer_mac[1],
                          cfg_this->peer_mac[2],
                          cfg_this->peer_mac[3],
                          cfg_this->peer_mac[4],
                          cfg_this->peer_mac[5],
                          default_mac_gndctrl[0],
                          default_mac_gndctrl[1],
                          default_mac_gndctrl[2],
                          default_mac_gndctrl[3],
                          default_mac_gndctrl[4],
                          default_mac_gndctrl[5]);
            publish_event(STS_THIS, SS_ESPNOW, EVENT_WARNING, buffer);
        }
        if (SS_OTHER==SS_ESP32 && memcmp(&cfg_this->peer_mac, &default_mac_esp32, 6)) {
            sprintf(buffer, "%s mac (%02x:%02x:%02x:%02x:%02x:%02x) from memory bank does not match default (%02x:%02x:%02x:%02x:%02x:%02x); using memory bank mac", 
                          subsystem[SS_OTHER].name,
                          cfg_this->peer_mac[0],
                          cfg_this->peer_mac[1],
                          cfg_this->peer_mac[2],
                          cfg_this->peer_mac[3],
                          cfg_this->peer_mac[4],
                          cfg_this->peer_mac[5],
                          default_mac_esp32[0],
                          default_mac_esp32[1],
                          default_mac_esp32[2],
                          default_mac_esp32[3],
                          default_mac_esp32[4],
                          default_mac_esp32[5]);
            publish_event(STS_THIS, SS_ESPNOW, EVENT_WARNING, buffer);
        }
    
        // Initialize ESP-NOW
        if (esp_now_init() == ESP_OK) {
            if (cfg_this->espnow_longrange) {
                esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR); 
            }
            esp_now_get_version(&espVersion);
            sprintf (buffer, "Initialized ESP-NOW v%lu on %s (max %d bytes) on mac %02x:%02x:%02x:%02x:%02x:%02x%s", 
                            espVersion, 
                            subsystem[SS_THIS].name, 
                            ESP_NOW_MAX_DATA_LEN,
                            cfg_this->my_mac[0],
                            cfg_this->my_mac[1],
                            cfg_this->my_mac[2],
                            cfg_this->my_mac[3],
                            cfg_this->my_mac[4],
                            cfg_this->my_mac[5],
                            cfg_this->espnow_longrange?" (long range mode)":""); 
            publish_event (STS_THIS, SS_ESPNOW, EVENT_INIT, buffer);
            tm_this->espnow_rx_enabled = true;
            tm_this->espnow_tx_enabled = true;
        }
        else {
            sprintf (buffer, "%s failed to initialize ESP-NOW", subsystem[SS_THIS].name); 
            publish_event (STS_THIS, SS_ESPNOW, EVENT_ERROR, buffer);
            return false;
        }
        
        // Register peer
        if (cfg_this->espnow_broadcast) {
            memcpy(peerInfo.peer_addr, mac_broadcast, 6);
        }
        else {
            memcpy(peerInfo.peer_addr, cfg_this->peer_mac, 6);
        }
        //if (cfg_this->wifi_ap_enable) {
            peerInfo.channel = cfg_this->wifi_channel;
        //}
        //else {
        //    peerInfo.channel = 0;  
        //}
        peerInfo.encrypt = false;
        register_espnow_peer(peerInfo);
        //if (!cfg_this->espnow_broadcast) {
        //    peerInfo.peer_addr[5] = cfg_this->peer_mac[5]+1;
        //    register_espnow_peer(peerInfo);
       // }

        // Register functions on ESP-NOW receive/send
        esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent_espnow));
        esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv_espnow));
        return true;
    }
    else {
        return false;
    }
}

void register_espnow_peer (esp_now_peer_info_t peerInfo) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        if(cfg_this->espnow_broadcast) {
            sprintf(buffer, "Failed to configure ESP-NOW on %s for broadcast", subsystem[SS_THIS].name);
        }
        else {
            sprintf(buffer, "Failed to add %s as ESP-NOW peer %02x:%02x:%02x:%02x:%02x:%02x on WiFi channel %u", 
                            subsystem[SS_ESPNOW_PEER].name,
                            peerInfo.peer_addr[0], 
                            peerInfo.peer_addr[1], 
                            peerInfo.peer_addr[2], 
                            peerInfo.peer_addr[3], 
                            peerInfo.peer_addr[4], 
                            peerInfo.peer_addr[5],
                            peerInfo.channel); 
        }
        publish_event (STS_THIS, SS_ESPNOW, EVENT_WARNING, buffer);
    }
    else {
        if(cfg_this->espnow_broadcast) {
            sprintf(buffer, "Configured ESP-NOW on %s for broadcast", subsystem[SS_THIS].name);
        }
        else {
            sprintf(buffer, "Added %s as ESP-NOW peer %02x:%02x:%02x:%02x:%02x:%02x of %s on WiFi channel %u", 
                            subsystem[SS_ESPNOW_PEER].name,
                            peerInfo.peer_addr[0], 
                            peerInfo.peer_addr[1], 
                            peerInfo.peer_addr[2], 
                            peerInfo.peer_addr[3], 
                            peerInfo.peer_addr[4], 
                            peerInfo.peer_addr[5],
                            subsystem[SS_THIS].name, 
                            peerInfo.channel); 
        }
        publish_event (STS_THIS, SS_ESPNOW, EVENT_INIT, buffer);
    }       
}

bool send_packet_through_espnow (ccsds_t* ccsds_ptr) {
    if (tm_this->espnow_tx_enabled) {
	    tm_this->espnow_tx_pktrate++;
  	    tm_this->espnow_tx_active = true;
		esp_err_t result = esp_now_send(cfg_this->peer_mac, (uint8_t*)ccsds_ptr, get_ccsds_packet_len (ccsds_ptr));
        Serial.print("e");
		if (result == ESP_OK) {
			return true;
		}
		else {
			sprintf(buffer, "Failed to send ESP-NOW packet from %s to %s", subsystem[SS_THIS].name, subsystem[SS_ESPNOW_PEER].name);
			publish_event(STS_THIS, SS_ESPNOW, EVENT_WARNING, buffer);
		}
	}
	return false;
}

// SerialTransfer Functionality

bool setup_serialtransfer (Stream &serialport) {
    if(cfg_this->serial_rx_enable or cfg_this->serial_tx_enable) { 
        serialtransfer.begin(serialport);
        tm_this->serial_rx_enabled = cfg_this->serial_rx_enable;
        tm_this->serial_tx_enabled = cfg_this->serial_tx_enable;
        return true;
    }
    return false;
}

bool check_serialtransfer_rx () {
    static byte serial_rx_buffer[BUFFER_MAX_SIZE];
    if (cfg_this->serial_rx_enable) {
        if(serialtransfer.available()) {        
            tm_this->serial_rx_active = true;    
            serialtransfer.rxObj(serial_rx_buffer);
            if(valid_ccsds_hdr((ccsds_t*)&(serial_rx_buffer), PKT_TM) or valid_ccsds_hdr((ccsds_t*)&(serial_rx_buffer), PKT_TC)) {
                tm_this->serial_rx_pktrate++;
       		    add_packet_to_memory_buffer(ccsds_rx_fifo, (ccsds_t*)serial_rx_buffer, SS_SERIAL_PEER, COMMS_SERIAL);
       		    return true;
       		}
       		else {
                sprintf(buffer, "Serial packet received by %s from %s with invalid CCSDS header (%02x %02x %02x %02x %02x %02x)", 
                                subsystem[SS_THIS].name,
                                subsystem[SS_SERIAL_PEER].name,
                                serial_rx_buffer[0], 
                                serial_rx_buffer[1], 
                                serial_rx_buffer[2],
                                serial_rx_buffer[3],
                                serial_rx_buffer[4],
                                serial_rx_buffer[5]);
                publish_event(STS_THIS, SS_SERIAL, EVENT_WARNING, buffer);
            }
        }
    }
    return false;
}

bool send_packet_through_serial (ccsds_t* ccsds_ptr) {
    if (tm_this->serial_tx_enabled) {
	    tm_this->serial_tx_pktrate++;
  	    tm_this->serial_tx_active = true;
  	    serialtransfer.sendDatum(*ccsds_ptr, get_ccsds_packet_len(ccsds_ptr));
        Serial.print("s");
  	    return true;
	}
	return false;
}

#if defined(PLATFORM_ESP32) || defined(PLATFORM_GNDCTRL)
// Radio Functionality
bool setup_radio () {
    if(cfg_this->radio_rx_enable or cfg_this->radio_tx_enable) {
        radio.setSpiPin(RADIO_SCK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
        radio.Init();                // must be set to initialize the cc1101!
        radio.setCCMode(1);          // set config for internal transmission mode.
        radio.setModulation(0);      // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
        radio.setMHZ(433.92);        // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
        radio.setDeviation(47.60);   // Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.
        radio.setChannel(0);         // Set the Channelnumber from 0 to 255. Default is cahnnel 0.
        radio.setChsp(199.95);       // The channel spacing is multiplied by the channel number CHAN and added to the base frequency in kHz. Value from 25.39 to 405.45. Default is 199.95 kHz.
        radio.setRxBW(812.50);       // Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.
        radio.setDRate(99.97);       // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!
        radio.setPA(10);             // Set TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!
        radio.setSyncMode(3);        // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
        radio.setSyncWord(211, 145); // Set sync word. Must be the same for the transmitter and receiver. (Syncword high, Syncword low)
        radio.setAdrChk(0);          // Controls address check configuration of received packages. 0 = No address check. 1 = Address check, no broadcast. 2 = Address check and 0 (0x00) broadcast. 3 = Address check and 0 (0x00) and 255 (0xFF) broadcast.
        radio.setAddr(0);            // Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
        radio.setWhiteData(0);       // Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on.
        radio.setPktFormat(0);       // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.
        radio.setLengthConfig(1);    // 0 = Fixed packet length mode. 1 = Variable packet length mode. 2 = Infinite packet length mode. 3 = Reserved
        radio.setPacketLength(0);    // Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed.
        radio.setCrc(1);             // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.
        radio.setCRC_AF(0);          // Enable automatic flush of RX FIFO when CRC is not OK. This requires that only one packet is in the RXIFIFO and that packet length is limited to the RX FIFO size.
        radio.setDcFilterOff(0);     // Disable digital DC blocking filter before demodulator. Only for data rates ≤ 250 kBaud The recommended IF frequency changes when the DC blocking is disabled. 1 = Disable (current optimized). 0 = Enable (better sensitivity).
        radio.setManchester(0);      // Enables Manchester encoding/decoding. 0 = Disable. 1 = Enable.
        radio.setFEC(0);             // Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. 0 = Disable. 1 = Enable.
        radio.setPRE(0);             // Sets the minimum number of preamble bytes to be transmitted. Values: 0 : 2, 1 : 3, 2 : 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24
        radio.setPQT(0);             // Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted.
        radio.setAppendStatus(0);    // When enabled, two status bytes will be appended to the payload of the packet. The status bytes contain RSSI and LQI values, as well as CRC OK.
        
        if(radio.getCC1101()) {
            sprintf(buffer, "Radio module %s initialised on %s", "CC1101", subsystem[SS_THIS].name);
            publish_event(STS_THIS, SS_RADIO, EVENT_INIT, buffer);
        }
        else {
            publish_event(STS_THIS, SS_RADIO, EVENT_ERROR, "Radio module not detected");
            tm_this->radio_rx_enabled = false;
            tm_this->radio_tx_enabled = false;
            return false;
        }
        //radio.SetRx();
        //radio.SetTx();
       
        if(cfg_this->radio_rx_enable) {
            tm_this->radio_rx_enabled = true;
        }
        if(cfg_this->radio_tx_enable) {
            tm_this->radio_tx_enabled = true;
        }
        return true;
    }
    return false;
}

bool check_radio_rx () {
    static byte radio_rx_buffer[BUFFER_MAX_SIZE];
    if (cfg_this->radio_rx_enable) {
        if(radio.CheckRxFifo(20)) {        
            tm_this->radio_rx_active = true;    
            radio.ReceiveData(radio_rx_buffer);
            if(valid_ccsds_hdr((ccsds_t*)&(radio_rx_buffer), PKT_TM) or valid_ccsds_hdr((ccsds_t*)&(radio_rx_buffer), PKT_TC)) {
                tm_this->radio_rx_pktrate++;
       		    add_packet_to_memory_buffer(ccsds_rx_fifo, (ccsds_t*)radio_rx_buffer, SS_RADIO, COMMS_RADIO);
       		    return true;
       		}
       		else {
                sprintf(buffer, "Radio packet with invalid CCSDS header received by %s (%02x %02x %02x %02x %02x %02x)", 
                                subsystem[SS_THIS].name,
                                radio_rx_buffer[0], 
                                radio_rx_buffer[1], 
                                radio_rx_buffer[2],
                                radio_rx_buffer[3],
                                radio_rx_buffer[4],
                                radio_rx_buffer[5]);
                publish_event(STS_THIS, SS_RADIO_PEER, EVENT_WARNING, buffer);
            }
        }
    }
    return false;
}

bool send_packet_through_radio (ccsds_t* ccsds_ptr) {
    if (tm_this->radio_tx_enabled) {
	    tm_this->radio_tx_pktrate++;
  	    tm_this->radio_tx_active = true;
		radio.SendData((uint8_t*)ccsds_ptr, get_ccsds_packet_len (ccsds_ptr), 100);
        Serial.print("r");
        return true;
	}
	return false;
}
#endif

// File System Functionality

bool setup_fs () {
    if (cfg_this->fs_enable) {
        if (LittleFS.begin(false)) {
            sprintf (buffer, "Initialized FS (size: %u kB; free: %u kB)", LittleFS.totalBytes()/1024, (LittleFS.totalBytes()-LittleFS.usedBytes())/1024);
            publish_event (STS_THIS, SS_FS, EVENT_INIT, buffer);
            if (cfg_this->flush_fs_enable) {
                flush_fs();
            }
            tm_this->fs_enabled = true;
            tm_this->fs_active = true;
            return true;
        }
        else {
            if (LittleFS.begin(true)) {
                sprintf(buffer, "Formatted and initialized FS (size: %u kB; free: %u kB)", LittleFS.totalBytes()/1024, fs_free());
                publish_event(STS_THIS, SS_FS, EVENT_WARNING, buffer);
                tm_this->fs_enabled = true;
                tm_this->fs_active = true;
                return true;
            }
            else {
                publish_event(STS_THIS, SS_FS, EVENT_ERROR, "Failed to initialize or format FS");
                tm_this->fs_enabled = false;
                return false;
            }
        }
    }
    return false;
}

bool setup_sd () {
    if (cfg_this->sd_enable) {
        if (!SD_MMC.begin()) {
            publish_event (STS_THIS, SS_SD, EVENT_ERROR, "Card reader initialisation failed");
            tm_this->sd_enabled = false;
            return false;
        }
        if (SD_MMC.cardType() == CARD_NONE) {
            publish_event (STS_THIS, SS_SD, EVENT_ERROR, "Card reader initialisation failed: no SD card inserted");
            tm_this->sd_enabled = false;
            return false;
        }
        sprintf (buffer, "Card reader initialised and SD card mounted: size: %llu MB; space: %llu MB; used: %llu MB", SD_MMC.cardSize() / (1024 * 1024), SD_MMC.totalBytes() / (1024 * 1024), SD_MMC.usedBytes() / (1024 * 1024));
        publish_event (STS_THIS, SS_SD, EVENT_INIT, buffer);       
        tm_this->sd_enabled = true;
        tm_this->sd_active = true;
    }
    return true;
}

uint16_t fs_free () {
    if (tm_this->fs_enabled) {
        sync_archive_file();
        if (LittleFS.totalBytes()-LittleFS.usedBytes() <= 8192) {
            tm_this->fs_enabled = false;
            sprintf (buffer, "Disabling further write access to FS because it is full");
            publish_event (STS_THIS, SS_FS, EVENT_ERROR, buffer);
        }
        return ((LittleFS.totalBytes()-LittleFS.usedBytes())/1024);
    }
    else {
        return 0;
    }
}

uint32_t sd_free () {
    if (tm_this->sd_enabled) {
        return ((SD_MMC.totalBytes() - SD_MMC.usedBytes())/1024);
    }
    return 0;
}

bool flush_fs () {
    File root = LittleFS.open("/");
    File file;
    char file_path[20];
    while ((file = root.openNextFile())) {
        sprintf(file_path, "/%s", file.name());
        if (strcmp(file_path, tm_this->archive_path)) {
            Serial.printf("Deleting file: %s (%u bytes)\n", file.name(), file.size());
            file.close();
            LittleFS.remove(file_path);
        }
        else {
            Serial.printf("Keeping file: %s (%u bytes)\n", file.name(), file.size());
        }
    }
    tm_this->fs_active = true;
    sprintf(buffer, "Flushed %s FS (free: %u kB)", filesystem[FS_LITTLEFS].name, (LittleFS.totalBytes()-LittleFS.usedBytes())/1024);
    publish_event (STS_THIS, SS_FS, EVENT_INIT, buffer);
    return true;
}

/*void create_today_dir () {
    if(tm_this->time_set) {
        sprintf (cfg_this->today_dir, "/%02u%02u%02u", year(), month(), day());
        switch(tm_this->archive_fs) {
        case FS_LITTLEFS:   
            if(!LittleFS.exists(cfg_this->today_dir)) {
                LittleFS.mkdir(cfg_this->today_dir);
                tm_this->fs_active = true;
            }
            break;
        case FS_SD_MMC:
            if(!SD_MMC.exists(cfg_this->today_dir)) {
                SD_MMC.mkdir(cfg_this->today_dir);
                tm_this->sd_active = true;
            }
            break;
        }
    }
}*/

// Packet Archive Functionality

bool setup_archive () {
    if(cfg_this->archive_enable) {
        switch (cfg_this->archive_fs) {
        case FS_LITTLEFS: 
            do {
                set_next_archive_path(tm_this->archive_path);
            }
            while(LittleFS.exists(tm_this->archive_path));
            var.archive_file = LittleFS.open(tm_this->archive_path, "a+");
            tm_this->fs_active = true;
            break;
        case FS_SD_MMC:
            do {
                set_next_archive_path(tm_this->archive_path);
            }
            while(SD_MMC.exists(tm_this->archive_path));
            var.archive_file = SD_MMC.open(tm_this->archive_path, "a+");
            tm_this->sd_active = true;
            break;
        }
        if (!var.archive_file) {
            sprintf (buffer, "Failed to open '%s' on %s in append/read mode", tm_this->archive_path, filesystem[cfg_this->archive_fs].name);
            publish_event (STS_THIS, SS_ARCHIVE, EVENT_ERROR, buffer);
            tm_this->archive_fs = FS_NONE;
            tm_this->archive_enabled = false;
            tm_this->archive_active = true;
            return false;
        }
        tm_this->archive_fs = cfg_this->archive_fs;
        tm_this->archive_enabled = true;
        tm_this->archive_active = true;
        return true;
    }
    return false;
}

void sync_archive_file () {
    if (var.archive_file) {
        var.archive_file.close();
    }
}

/*
void move_archive_file () {
    char new_archive_path[20];
    sync_archive_file();
    switch (tm_this->archive_fs) {
    case FS_LITTLEFS: 
        do {
            set_next_archive_path(new_archive_path);
        }
        while(LittleFS.exists(new_archive_path));
        LittleFS.rename(tm_this->archive_path, new_archive_path);
        tm_this->fs_active = true;
        break;
    case FS_SD_MMC:
        do {
            set_next_archive_path(new_archive_path);
        }
        while(SD_MMC.exists(new_archive_path));
        SD_MMC.rename(tm_this->archive_path, new_archive_path);
        tm_this->sd_active = true;
        break;
    }
    sprintf (buffer, "Moved %s archive from '%s' to '%s'", subsystem[SS_THIS].name, tm_this->archive_path, new_archive_path);
    publish_event (STS_THIS, SS_ARCHIVE, EVENT_INFO, buffer);
    strcpy(tm_this->archive_path, new_archive_path);
} */
   
void set_next_archive_path (char* archive_path) {
    static char sequencer1 = 'A';
    static char sequencer2 = 'A';
    if(tm_this->time_set) {
        sprintf (archive_path, "/%02u%02u%02u/%c%c.ccsds", year(), month(), day(), sequencer1, sequencer2++);
    }
    else {
        sprintf (archive_path, "/%c%c.ccsds", sequencer1, sequencer2++);
    }
    if (sequencer2 == '[') {
      sequencer1++;
      sequencer2 = 'A';
    }
}

bool save_packet_to_archive (ccsds_t* ccsds_ptr) {
    if (tm_this->archive_enabled) {
        tm_this->archive_pktrate++;
        tm_this->archive_active = true;
        uint8_t packet_len = get_ccsds_packet_len(ccsds_ptr);
        switch(tm_this->archive_fs) {
        case FS_LITTLEFS:
            if (tm_this->fs_enabled and tm_this->archive_enabled) {
                if(!var.archive_file) {
                    var.archive_file = LittleFS.open(tm_this->archive_path, "a+");
                }
                var.archive_file.write((const uint8_t*)ccsds_ptr, packet_len);
                archive.packet_len = packet_len;
                archive.packet_offset = var.archive_file.position() - packet_len;
                tm_this->fs_active = true;
                tm_this->archive_active = true;
                Serial.print("a");
                return true;
            }
            break;
        case FS_SD_MMC:
            if (tm_this->sd_enabled and tm_this->archive_enabled) {
                if(!var.archive_file) {
                    var.archive_file = SD_MMC.open(tm_this->archive_path, "a+");
                }
                var.archive_file.write((const uint8_t*)ccsds_ptr, packet_len);
                archive.packet_len = packet_len;
                archive.packet_offset = var.archive_file.position() - packet_len;
                tm_this->sd_active = true;
                tm_this->archive_active = true;
                Serial.print("a");
                return true;
            }
            break;
        }
    }
    return false;
}

// Packet Buffering Functionality

bool add_packet_to_memory_buffer (LinkedList<buffer_t*> *ccsds_fifo_ptr, ccsds_t* ccsds_ptr, uint8_t source, uint8_t comms) {
    // copy ccsds packet to memory
    uint16_t ccsds_len = get_ccsds_packet_len(ccsds_ptr);
    ccsds_t* ccsds_copy = (ccsds_t*)malloc(ccsds_len);
    memcpy(ccsds_copy, ccsds_ptr, ccsds_len);
    // create a linked list entry for the ccsds packet
    buffer_t* ccsds_buffer = (buffer_t*)malloc(sizeof(buffer_t));
    ccsds_buffer->ccsds_ptr = ccsds_copy;
    ccsds_buffer->packet_source = source;
    ccsds_buffer->packet_comms = comms;
    ccsds_fifo_ptr->add(ccsds_buffer);
    return true;
}

buffer_t* get_packet_from_memory_buffer (LinkedList<buffer_t*> *ccsds_fifo_ptr, uint16_t index) {
    if (ccsds_fifo_ptr->size() > index) {
        return ccsds_fifo_ptr->get(index);
    }
    else {
        return NULL;
    }   
}

bool delete_packet0_from_memory_buffer (LinkedList<buffer_t*> *ccsds_fifo_ptr) {
    if (ccsds_fifo_ptr->size()) {
        buffer_t* ccsds_buffer = ccsds_fifo_ptr->shift();
        free(ccsds_buffer->ccsds_ptr);
        return true;
    }
    else {
        return false;
    }
}

// TM/TC Functionality

void process_rx_queue () {
    // Process queue of packets that have been received over ESP-NOW, Serial, radio, FS buffer or others
    while (buffer_t* ccsds_rx_buffer = get_packet_from_memory_buffer(ccsds_rx_fifo, 0)) {  
        if (valid_ccsds_hdr(ccsds_rx_buffer->ccsds_ptr, PKT_TM)) {
            // The received packet is TM
            uint8_t PID = get_ccsds_pid(ccsds_rx_buffer->ccsds_ptr); 
            //Serial.print(millis());
            //Serial.print(" In RX queue: ");
            //print_ccsds_data (ccsds_rx_buffer->ccsds_ptr);
            //Serial.println("");
            if (packet[PID].source != SS_THIS) {
                // The TM originated from another subsystem and I'm likely in the transmission chain (good): store in local structure and redistribute
                if (get_ccsds_packet_ctr(packet[PID].ccsds_ptr) != get_ccsds_packet_ctr(ccsds_rx_buffer->ccsds_ptr)) {  // filter out duplicates
                    memcpy(packet[PID].ccsds_ptr, ccsds_rx_buffer->ccsds_ptr, get_ccsds_packet_len(ccsds_rx_buffer->ccsds_ptr));
                    add_packet_to_memory_buffer(ccsds_tx_fifo, packet[PID].ccsds_ptr, ccsds_rx_buffer->packet_source, ccsds_rx_buffer->packet_comms);
                }
            }
            else {
                // Somebody sent us a TM that originated from this subsystem (bad): block to avoid infinite loop
                sprintf(buffer, "Blocked %s TM packet received by %s over %s", packet[PID].name, subsystem[SS_THIS].name, comms[ccsds_rx_buffer->packet_comms].name);
				publish_event(STS_THIS, SS_TCTM, EVENT_WARNING, buffer);
            }
        }
        else if (valid_ccsds_hdr (ccsds_rx_buffer->ccsds_ptr, PKT_TC)) {
            // The received packet is TC
            uint8_t PID = get_ccsds_pid(ccsds_rx_buffer->ccsds_ptr); 
            if (packet[PID].dest == SS_THIS) {
                // The TC is to be executed by this subsystem (good): store in local structure and execute
                memcpy(tc_this, ccsds_rx_buffer->ccsds_ptr, get_ccsds_packet_len(ccsds_rx_buffer->ccsds_ptr));
                if(tc_this->cmd_id==CMD_SET_PARAMETER or tc_this->cmd_id==CMD_SAVE_CONFIG) {
                    sprintf(buffer, "Received TC for %s with cmd_id %u (%s), parameter [%s]", subsystem[SS_THIS].name, tc_this->cmd_id, command[tc_this->cmd_id].name, (const char*)tc_this->parameter);
                }
                else {
                    sprintf(buffer, "Received TC for %s with cmd_id %u (%s), parameter [0x%02x]", subsystem[SS_THIS].name, tc_this->cmd_id, command[tc_this->cmd_id].name, tc_this->parameter[0]);
                }
                publish_event(STS_THIS, SS_TCTM, EVENT_CMD_ACK, buffer);
                execute_tc();
            }
            else if (packet[PID].source != SS_THIS) { 
                // The TC is for some other subsystem and I'm likely in the transmission chain (good): store in local structure and redistribute
                memcpy(packet[PID].ccsds_ptr, ccsds_rx_buffer->ccsds_ptr, get_ccsds_packet_len(ccsds_rx_buffer->ccsds_ptr));
                add_packet_to_memory_buffer(ccsds_tx_fifo, packet[PID].ccsds_ptr, ccsds_rx_buffer->packet_source, ccsds_rx_buffer->packet_comms);
            }
            else {
                // Somebody sent us a TC that originated from this subsystem (bad): block to avoid infinite loop 
                sprintf(buffer, "Blocked %s TC packet received by %s over %s", packet[PID].name, subsystem[SS_THIS].name, comms[ccsds_rx_buffer->packet_comms].name);
                publish_event(STS_THIS, SS_TCTM, EVENT_WARNING, buffer);
            }
        }
        else {
            // Invalid CCSDS packet
            sprintf (buffer, "Received packet with invalid CCSDS header (%s...) over %s", get_hex_str((byte*)ccsds_rx_buffer, 6).c_str(), comms[ccsds_rx_buffer->packet_comms].name);
            publish_event (STS_THIS, SS_TCTM, EVENT_WARNING, buffer);
        }
        delete_packet0_from_memory_buffer(ccsds_rx_fifo);
    }
}

void process_tx_queue () {
    static buffer_t* ccsds_tx_buffer;
    
    tm_this->buffer_size = ccsds_tx_fifo->size();
      
    if(tm_this->buffer_size > var.espnow_buffer_index) {   // there's packets to be considered
        if(cfg_this->espnow_tx_enable) {                        // this interface is relevant as it may come up if not already active
            if(tm_this->espnow_tx_enabled) {                    // this interface is active, so we'll want to send
                ccsds_tx_buffer = get_packet_from_memory_buffer(ccsds_tx_fifo, var.espnow_buffer_index);
                uint8_t PID = get_ccsds_pid(ccsds_tx_buffer->ccsds_ptr);
                var.espnow_buffer_index++;
                if (get_routing(&cfg_this->routing_espnow, PID)) {
                    Serial.print("E");
                    if (!send_packet_through_espnow(ccsds_tx_buffer->ccsds_ptr) and cfg_this->espnow_buffer_enable) {
                        // sending failed so if we are interested in buffering we want to attempt resend
                        var.espnow_buffer_index--;
                    }
                }
            }
            else if(!cfg_this->espnow_buffer_enable) { // if we're not interested in buffering, move on to next packet
                var.espnow_buffer_index++; 
            }
        }
        else { // move on, we'll not be sending through this interface anyway
            var.espnow_buffer_index++; 
        }
    }
    
    if(tm_this->buffer_size > var.serial_buffer_index) {
        if(cfg_this->serial_tx_enable) {
            if(tm_this->serial_tx_enabled) {
                ccsds_tx_buffer = get_packet_from_memory_buffer(ccsds_tx_fifo, var.serial_buffer_index);
                uint8_t PID = get_ccsds_pid(ccsds_tx_buffer->ccsds_ptr);
                var.serial_buffer_index++;
                if (get_routing(&cfg_this->routing_serial, PID)) {
                    Serial.print("S");
                    if (!send_packet_through_serial(ccsds_tx_buffer->ccsds_ptr) and cfg_this->serial_buffer_enable) {
                        var.serial_buffer_index--;
                    }
                }
            }
            else if(!cfg_this->serial_buffer_enable) {
                var.serial_buffer_index++; 
            }
        }
        else {
            var.serial_buffer_index++; 
        }
    }

 #if defined(PLATFORM_ESP32) || defined(PLATFORM_GNDCTRL)
    if(tm_this->buffer_size > var.radio_buffer_index) {
        if(cfg_this->radio_tx_enable) {
            if(tm_this->radio_tx_enabled) {
                ccsds_tx_buffer = get_packet_from_memory_buffer(ccsds_tx_fifo, var.radio_buffer_index);
                uint8_t PID = get_ccsds_pid(ccsds_tx_buffer->ccsds_ptr);
                var.radio_buffer_index++;
                if (get_routing(&cfg_this->routing_radio, PID)) {
                    Serial.print("R");
                    if (!send_packet_through_radio(ccsds_tx_buffer->ccsds_ptr) and cfg_this->radio_buffer_enable) {
                        var.radio_buffer_index--;
                    }
                }
            }
            else if(!cfg_this->radio_buffer_enable) {
                var.radio_buffer_index++; 
            }
        }
        else {
            var.radio_buffer_index++; 
        }
    }
 #endif

    if(tm_this->buffer_size > var.archive_buffer_index) {
        if(cfg_this->archive_enable) {
            if(tm_this->archive_enabled) {
                ccsds_tx_buffer = get_packet_from_memory_buffer(ccsds_tx_fifo, var.archive_buffer_index);
                uint8_t PID = get_ccsds_pid(ccsds_tx_buffer->ccsds_ptr);
                var.archive_buffer_index++;
                if (get_routing(&cfg_this->routing_archive, PID)) {
                    Serial.print("A");
                    if (!save_packet_to_archive(ccsds_tx_buffer->ccsds_ptr) and cfg_this->archive_buffer_enable) {
                        var.archive_buffer_index--;
                    }
                }
            }
            else if(!cfg_this->archive_buffer_enable) {
                var.archive_buffer_index++; 
            }
        }
        else {
            var.archive_buffer_index++; 
        }
    }
    
    while((var.espnow_buffer_index and var.serial_buffer_index and var.radio_buffer_index and var.archive_buffer_index) or tm_this->buffer_size>250) {
        delete_packet0_from_memory_buffer(ccsds_tx_fifo);
        if(var.espnow_buffer_index)  { var.espnow_buffer_index--; }
        if(var.serial_buffer_index)  { var.serial_buffer_index--; }
        if(var.radio_buffer_index)   { var.radio_buffer_index--; }
        if(var.archive_buffer_index) { var.archive_buffer_index--; }
        tm_this->buffer_size = ccsds_tx_fifo->size();
    }
}

void publish_packet (ccsds_t* ccsds_ptr) {
    update_packet(ccsds_ptr);
    add_packet_to_memory_buffer(ccsds_tx_fifo, ccsds_ptr, SS_THIS, COMMS_ANY);
    reset_packet(ccsds_ptr);
    process_tx_queue();
}

void publish_event (uint8_t PID, uint8_t subsystem, uint8_t event_type, const char* event_message) {
	if (SS_THIS == SS_ESP32) {
		Serial.println(event_message);
	}
    switch (event_type) {
    case EVENT_ERROR:    tm_this->error_ctr++; break;  
    case EVENT_WARNING:  tm_this->warning_ctr++; break;  
    case EVENT_CMD_RESP: tm_this->tc_exec_ctr++; break; 
    case EVENT_CMD_FAIL: tm_this->tc_fail_ctr++; break; 
    }  
    sts_this->subsystem = subsystem;
    sts_this->type = event_type;
    strcpy (sts_this->message, event_message);
    set_ccsds_payload_len (packet[PID].ccsds_ptr, strlen (event_message) + 4);
    publish_packet (packet[PID].ccsds_ptr);
}

void publish_cmd (uint8_t PID, uint8_t command_id, const byte* cmd_payload, uint8_t payload_len) {
    switch(PID) {
        case TC_ESP32: 
            tc_esp32.cmd_id = command_id;
            memcpy (tc_esp32.parameter, cmd_payload, payload_len);
            break;
        case TC_ESP32CAM: 
            tc_esp32cam.cmd_id = command_id;
            memcpy (tc_esp32cam.parameter, cmd_payload, payload_len);
            break;
        case TC_GNDCTRL: 
            tc_gndctrl.cmd_id = command_id;
            memcpy (tc_gndctrl.parameter, cmd_payload, payload_len);
            break;
    }
    set_ccsds_payload_len (packet[PID].ccsds_ptr, payload_len);
    publish_packet (packet[PID].ccsds_ptr);
}

void update_packet (ccsds_t* ccsds_ptr) {
    // Increment sequence counter in CCSDS header
    ((ccsds_hdr_t*)ccsds_ptr)->seq_ctr_L++;
    if (((ccsds_hdr_t*)ccsds_ptr)->seq_ctr_L == 0) {
        ((ccsds_hdr_t*)ccsds_ptr)->seq_ctr_H++;
    }
    if (((ccsds_hdr_t*)ccsds_ptr)->sec_hdr and tm_this->time_set) {
        ((ccsds_t*)ccsds_ptr)->ccsds_sec_hdr.seconds = now();
        ((ccsds_t*)ccsds_ptr)->ccsds_sec_hdr.subseconds = (millis()+var.delta_millis)%1000;
    }
       
    // Do updates on packets
    switch (get_ccsds_pid (ccsds_ptr)) {
    case STS_ESP32:     sts_esp32.packet_ctr++;
                        break;
    case STS_ESP32CAM:  sts_esp32cam.packet_ctr++;
                        break;
    case STS_GNDCTRL:   sts_gndctrl.packet_ctr++;
                        break;
    case TM_ESP32:      tm_esp32.millis = millis();
                        tm_esp32.packet_ctr++;
                        tm_esp32.mem_free = ESP.getFreeHeap()/1024;
                        tm_esp32.fs_free = fs_free();
                        tm_esp32.buffer_size = ccsds_tx_fifo->size();
                        tm_esp32.espnow_buffer_queue = tm_esp32.buffer_size - var.espnow_buffer_index;
                        tm_esp32.serial_buffer_queue = tm_esp32.buffer_size - var.serial_buffer_index;
                        tm_esp32.radio_buffer_queue = tm_esp32.buffer_size - var.radio_buffer_index;
                        tm_esp32.archive_buffer_queue = tm_esp32.buffer_size - var.archive_buffer_index;
                        tm_esp32.battery_voltage = 0.9*tm_esp32.battery_voltage + 0.1*2*analogReadMilliVolts(BAT_V_PIN);
                        tm_esp32.battery_percentage = min(100, max(0, (tm_esp32.battery_voltage - cfg_esp32.battery_voltage_min) * 100 / (cfg_esp32.battery_voltage_max - cfg_esp32.battery_voltage_min)));
                        /*if (get_routing(&cfg_esp32->routing_espnow, TM_ESP32) and tm_esp32->espnow_tx_enabled) {
                            tm_esp32->espnow_tx_pktrate++;
                        }
                        if (get_routing(&cfg_esp32->routing_serial, TM_ESP32) and tm_esp32->serial_tx_enabled) {
                            tm_esp32->serial_tx_pktrate++;
                        }
                        if (get_routing(&cfg_esp32->routing_radio, TM_ESP32) and tm_esp32->radio_tx_enabled) {
                            tm_esp32->radio_tx_pktrate++;
                        }                        
                        if (get_routing(&cfg_esp32->routing_archive, TM_ESP32) and ((cfg_esp32->archive_fs==FS_LITTLEFS and tm_esp32->fs_enabled) or 
                                                                                     (cfg_esp32->archive_fs==FS_SD_MMC and tm_esp32->sd_enabled)) {
                            tm_esp32->archive_pktrate++;
                        } */
                        break; 
    case TM_GPS:        tm_gps.millis=millis();
                        tm_gps.packet_ctr++;
                        break;                    
    case TM_MOTION:     tm_motion.millis=millis();
                        tm_motion.packet_ctr++;
                        break; 
    case TM_PRESSURE:   tm_pressure.millis=millis();
                        tm_pressure.packet_ctr++;
                        break;                         
    case TM_RADIO:      tm_radio.millis=millis();
                        tm_radio.packet_ctr++; /*
                        tm_radio.opsmode = tm_esp32.opsmode;
                        tm_radio.error_ctr = min(255, esp32.error_ctr + esp32cam.error_ctr);
                        tm_radio.warning_ctr = min(255, esp32.warning_ctr + esp32cam.warning_ctr);
                        tm_radio.pressure_height = max(0, min(255, (bmp280.height+50)/100));
                        tm_radio.pressure_velocity_v = int8_t((bmp280.velocity_v+((bmp280.velocity_v > 0) - (bmp280.velocity_v < 0))*50)/100);
                        tm_radio.temperature = int8_t((bmp280.temperature+((bmp280.temperature > 0) - (bmp280.temperature < 0))*50)/100);
                        tm_radio.motion_tilt = uint8_t((motion.tilt+50)/100);
                        tm_radio.motion_g = uint8_t((motion.g+50)/100);  
                        tm_radio.motion_a = int8_t((motion.a+((motion.a > 0) - (motion.a < 0))*50)/100);
                        tm_radio.motion_rpm = int8_t((motion.rpm+((motion.rpm > 0) - (motion.rpm < 0))*50)/100); 
                        tm_radio.gps_satellites = tm_gps.satellites;
                        tm_radio.gps_velocity_v = int8_t(-(neo6mv2.v_down+((neo6mv2.v_down > 0) - (neo6mv2.v_down < 0))*50)/100);
                        tm_radio.gps_velocity = uint8_t(sqrt (neo6mv2.v_north*neo6mv2.v_north + neo6mv2.v_east*neo6mv2.v_east + neo6mv2.v_down*neo6mv2.v_down) / 1000000);   
                        tm_radio.gps_height = max(0, min(255, (neo6mv2.z+50)/100));
                        tm_radio.camera_image_ctr = tm_camera.packet_ctr;
                        tm_radio.esp32_buffer_active = esp32.buffer_active;
                        tm_radio.separation_sts = esp32.separation_sts;
                        tm_radio.esp32cam_buffer_active = esp32cam.buffer_active; */
                        break;                          
    case TM_ESP32CAM:   tm_esp32cam.millis = millis();
                        tm_esp32cam.packet_ctr++;
                        tm_esp32cam.mem_free = ESP.getFreeHeap()/1024;
                        tm_esp32cam.fs_free = fs_free();
                        tm_esp32cam.sd_free = sd_free();
                        tm_esp32cam.buffer_size = ccsds_tx_fifo->size();
                        tm_esp32cam.espnow_buffer_queue = tm_esp32cam.buffer_size - var.espnow_buffer_index;
                        tm_esp32cam.serial_buffer_queue = tm_esp32cam.buffer_size - var.serial_buffer_index;
                        tm_esp32cam.archive_buffer_queue = tm_esp32cam.buffer_size - var.archive_buffer_index;
                    /*    if (cfg_this->routing_espnow[TM_THIS] and tm_this->espnow_tx_enabled) {
                            tm_this->espnow_tx_pktrate++;
                        }
                        if (cfg_this->routing_serial[TM_THIS] and tm_this->serial_tx_enabled) {
                            tm_this->serial_tx_pktrate++;
                        }
                        if (cfg_this->routing_radio[TM_THIS] and tm_this->radio_tx_enabled) {
                            tm_this->radio_tx_pktrate++;
                        }                        
                        if (get_routing(&cfg_this->routing_archive, TM_THIS) and ((cfg_this->archive_fs==FS_LITTLEFS and tm_this->fs_enabled) or 
                                                                                     (cfg_this->archive_fs==FS_SD_MMC and tm_this->sd_enabled)) {
                            tm_this->archive_pktrate++;
                        } */
                        break;   
    case TM_CAMERA:     tm_camera.millis = millis();
                        tm_camera.packet_ctr++;
                        break;
    case TM_GNDCTRL:    tm_gndctrl.millis = millis();
                        tm_gndctrl.packet_ctr++;
                        tm_gndctrl.mem_free = ESP.getFreeHeap()/1024;
                        tm_gndctrl.fs_free = fs_free();
                        tm_gndctrl.battery_voltage = 0.9*tm_gndctrl.battery_voltage + 0.1*2*analogReadMilliVolts(BAT_V_PIN);
                        tm_gndctrl.battery_percentage = min(100, max(0, (tm_gndctrl.battery_voltage - cfg_gndctrl.battery_voltage_min) * 100 / (cfg_gndctrl.battery_voltage_max - cfg_gndctrl.battery_voltage_min)));
                        tm_gndctrl.buffer_size = ccsds_tx_fifo->size();
                        tm_gndctrl.espnow_buffer_queue = tm_gndctrl.buffer_size - var.espnow_buffer_index;
                        tm_gndctrl.serial_buffer_queue = tm_gndctrl.buffer_size - var.serial_buffer_index;
                        tm_gndctrl.archive_buffer_queue = tm_gndctrl.buffer_size - var.archive_buffer_index;
             /*           if (cfg_this->routing_espnow[TM_THIS] and tm_this->espnow_tx_enabled) {
                            tm_this->espnow_tx_pktrate++;
                        }
                        if (cfg_this->routing_serial[TM_THIS] and tm_this->serial_tx_enabled) {
                            tm_this->serial_tx_pktrate++;
                        }
                        if (cfg_this->routing_radio[TM_THIS] and tm_this->radio_tx_enabled) {
                            tm_this->radio_tx_pktrate++;
                        }                        
                        if (get_routing(&cfg_this->routing_archive, TM_THIS) and ((cfg_this->archive_fs==FS_LITTLEFS and tm_this->fs_enabled) or 
                                                                                     (cfg_this->archive_fs==FS_SD_MMC and tm_this->sd_enabled)) {
                            tm_this->archive_pktrate++;
                        }     */                  
                        break;                        
    case TIMER_ESP32:   tmr_esp32.idle_duration = max(0, 1000 - tmr_esp32.radio_duration - tmr_esp32.pressure_duration - tmr_esp32.motion_duration - tmr_esp32.gps_duration - tmr_esp32.esp32cam_duration - tmr_esp32.ota_duration - tmr_esp32.ftp_duration - tmr_esp32.wifi_duration - tmr_esp32.tc_duration);
                        break;
    case TIMER_ESP32CAM:tmr_esp32cam.idle_duration = max(0, 1000 - tmr_esp32cam.sd_duration - tmr_esp32cam.camera_duration - tmr_esp32cam.ftp_duration - tmr_esp32cam.wifi_duration - tmr_esp32cam.tc_duration);
                        break;
    }
}

void reset_packet (ccsds_t* ccsds_ptr) {
    switch (get_ccsds_pid (ccsds_ptr)) {
    case TC_ESP32:      tc_esp32.parameter[0] = 0;
                        break;
    case TC_ESP32CAM:   tc_esp32cam.parameter[0] = 0;
                        break;
	case TC_GNDCTRL:    tc_gndctrl.parameter[0] = 0;
                        break;
    case STS_ESP32:     sts_esp32.message[0] = 0;
                        break;
    case STS_ESP32CAM:  sts_esp32cam.message[0] = 0;
                        break;
    case STS_GNDCTRL:   sts_gndctrl.message[0] = 0;
                        break;
    case TM_ESP32:      tm_esp32.espnow_rx_pktrate = 0;
                        tm_esp32.espnow_tx_pktrate = 0;
                        tm_esp32.serial_rx_pktrate = 0;
                        tm_esp32.serial_tx_pktrate = 0;
                        tm_esp32.radio_rx_pktrate = 0;
    	                tm_esp32.radio_tx_pktrate = 0;
                        tm_esp32.archive_pktrate = 0;
                        tm_esp32.pressure_pktrate = 0;
                        tm_esp32.motion_pktrate = 0;
                        tm_esp32.gps_pktrate = 0;
						tm_esp32.radio_rssi = 0;
                        tm_esp32.wifi_active = false;
                        tm_esp32.ftp_active = false;
                        tm_esp32.archive_active = false;
                        tm_esp32.fs_active = false;
                        tm_esp32.pressure_active = false;
                        tm_esp32.motion_active = false;
                        tm_esp32.gps_active = false; 
                        tm_esp32.espnow_rx_active = false;
                        tm_esp32.espnow_tx_active = false;
                        tm_esp32.serial_rx_active = false;
                        tm_esp32.serial_tx_active = false;
                        tm_esp32.radio_rx_active = false;
                        tm_esp32.radio_tx_active = false;
                        tm_esp32.buzzer_active=false;
                        break;
    case TM_GPS:        tm_esp32.gps_pktrate++;
                        tm_gps.status = 8;  // set default to "none"
                        break;
    case TM_MOTION:     tm_esp32.motion_pktrate++;
                        break;
    case TM_PRESSURE:   tm_esp32.pressure_pktrate++;
                        break;
    case TM_RADIO:      tm_radio.pressure_active = false;
                        tm_radio.motion_active = false;
                        tm_radio.gps_active = false; 
                        tm_radio.camera_active = false; 
                        break;
    case TM_ESP32CAM:   tm_esp32cam.espnow_rx_pktrate = 0;
                        tm_esp32cam.espnow_tx_pktrate = 0;
                        tm_esp32cam.serial_rx_pktrate = 0;
                        tm_esp32cam.serial_tx_pktrate = 0;
                        tm_esp32cam.archive_pktrate = 0;
                        tm_esp32cam.camera_pktrate = 0;
                        tm_esp32cam.wifi_active = false;
                        tm_esp32cam.ftp_active = false;
                        tm_esp32cam.archive_active = false;
                        tm_esp32cam.fs_active = false;
                        tm_esp32cam.sd_active = false;
                        tm_esp32cam.camera_active = false;
                        tm_esp32cam.espnow_rx_active = false;
                        tm_esp32cam.espnow_tx_active = false;
                        tm_esp32cam.serial_rx_active = false;
                        tm_esp32cam.serial_tx_active = false;
                        //tm_esp32cam.rtsp_active = false;
                        break;
    case TM_CAMERA:     strcpy (tm_camera.filename, "");
                        tm_camera.filesize = 0;
                        tm_camera.wifi_ms = 0;
                        tm_camera.sd_ms = 0;
                        tm_camera.exposure_ms = 0;
                        break;
	case TM_GNDCTRL:    tm_gndctrl.espnow_rx_pktrate = 0;
						tm_gndctrl.espnow_tx_pktrate = 0;
						tm_gndctrl.serial_rx_pktrate = 0;
						tm_gndctrl.serial_tx_pktrate = 0;
						tm_gndctrl.radio_tx_pktrate = 0;
						tm_gndctrl.radio_rx_pktrate = 0;
						tm_gndctrl.archive_pktrate = 0;
						tm_gndctrl.radio_rssi = 0;
						tm_gndctrl.wifi_active = false;
                        tm_gndctrl.ftp_active = false;
                        tm_gndctrl.archive_active = false;
                        tm_gndctrl.fs_active = false;
                        tm_gndctrl.espnow_rx_active = false;
						tm_gndctrl.espnow_tx_active = false;
						tm_gndctrl.serial_rx_active = false;
						tm_gndctrl.serial_tx_active = false;
						tm_gndctrl.radio_rx_active = false;
						tm_gndctrl.radio_tx_active = false;
                        tm_gndctrl.buzzer_active=false;
						break;                        
    case TIMER_ESP32:   tmr_esp32.radio_duration = 0;
                        tmr_esp32.pressure_duration = 0;
                        tmr_esp32.motion_duration = 0;
                        tmr_esp32.gps_duration = 0;
                        tmr_esp32.esp32cam_duration = 0;
                        tmr_esp32.ota_duration = 0;
                        tmr_esp32.ftp_duration = 0;
                        tmr_esp32.wifi_duration = 0;
                        tmr_esp32.tc_duration = 0;
                        tmr_esp32.idle_duration = 0;
                        tmr_esp32.publish_fs_duration = 0;
                        tmr_esp32.publish_espnow_duration = 0;
                        break;                            
    case TIMER_ESP32CAM:tmr_esp32cam.camera_duration = 0;
                        tmr_esp32cam.tc_duration = 0;
                        tmr_esp32cam.sd_duration = 0;
                        tmr_esp32cam.ftp_duration = 0;
                        tmr_esp32cam.wifi_duration = 0;
                        tmr_esp32cam.idle_duration = 0;
                        tmr_esp32cam.publish_sd_duration = 0;
                        tmr_esp32cam.publish_fs_duration = 0;
                        tmr_esp32cam.publish_espnow_duration = 0;
                        break;
    }
}

bool get_routing(uint32_t *routing, uint8_t PID) {
    return (*routing & (1U << PID)) ? 1 : 0;
}

void set_routing(uint32_t *routing, uint8_t PID, bool status) {
    if (status) {
        *routing |= (1U << PID);
        Serial.println("activating");
    }
    else {
        *routing &= ~(1U << PID);
        Serial.println("deactivating");
    }
}

// CCSDS Functionality

void init_ccsds () {
    for (uint8_t PID=0; PID<NUMBER_OF_PID; PID++) {
    	init_ccsds_hdr (packet[PID].ccsds_ptr, packet[PID].APID, packet[PID].type, packet[PID].size);
    }
}

void init_ccsds_hdr (ccsds_t* ccsds_ptr, uint16_t APID, uint8_t pkt_type, uint16_t pkt_len) {
	uint16_t len = pkt_len - sizeof(ccsds_hdr_t) - 1;
	(ccsds_ptr->ccsds_hdr).version = 0;
	(ccsds_ptr->ccsds_hdr).type = pkt_type;
	if (pkt_type) {
	    (ccsds_ptr->ccsds_hdr).sec_hdr = false; // TC
	}
	else {
	    (ccsds_ptr->ccsds_hdr).sec_hdr = true; // TM
	}
	(ccsds_ptr->ccsds_hdr).apid_H = (uint8_t)(APID >> 8);
	(ccsds_ptr->ccsds_hdr).apid_L = (uint8_t)APID;
	(ccsds_ptr->ccsds_hdr).seq_flag = 3;
	(ccsds_ptr->ccsds_hdr).pkt_len_H = (uint8_t)(len >> 8);  
	(ccsds_ptr->ccsds_hdr).pkt_len_L = (uint8_t)len;
    (ccsds_ptr->ccsds_sec_hdr).seconds = 0;
    (ccsds_ptr->ccsds_sec_hdr).subseconds = 0;
}

bool valid_ccsds_hdr (ccsds_t* ccsds_ptr, bool pkt_type) {
	return (((ccsds_hdr_t*)ccsds_ptr)->version == 0 and
			((ccsds_hdr_t*)ccsds_ptr)->type == pkt_type and
            ((ccsds_hdr_t*)ccsds_ptr)->seq_flag == 3);
            // and
            //((ccsds_hdr_t*)ccsds_ptr)->pkt_len_L + 256*((ccsds_hdr_t*)ccsds_ptr)->pkt_len_H + 1 <= PARAMETER_MAX_SIZE);
}

uint16_t get_ccsds_apid (ccsds_t* ccsds_ptr) {
  return (256*((ccsds_hdr_t*)ccsds_ptr)->apid_H + ((ccsds_hdr_t*)ccsds_ptr)->apid_L);
}

uint16_t get_ccsds_packet_ctr (ccsds_t* ccsds_ptr) {
  return (256*((ccsds_hdr_t*)ccsds_ptr)->seq_ctr_H + ((ccsds_hdr_t*)ccsds_ptr)->seq_ctr_L);
}

uint16_t get_ccsds_packet_len (ccsds_t* ccsds_ptr) {
    if(((ccsds_hdr_t*)ccsds_ptr)->sec_hdr) {
        return (sizeof(ccsds_hdr_t) + sizeof(ccsds_sec_hdr_t) + 256*((ccsds_hdr_t*)ccsds_ptr)->pkt_len_H + ((ccsds_hdr_t*)ccsds_ptr)->pkt_len_L + 1);
    }
    else {
        return (sizeof(ccsds_hdr_t) + 256*((ccsds_hdr_t*)ccsds_ptr)->pkt_len_H + ((ccsds_hdr_t*)ccsds_ptr)->pkt_len_L + 1);
    }
}

bool get_ccsds_packet_type (ccsds_t* ccsds_ptr) {
  return (((ccsds_hdr_t*)ccsds_ptr)->type);
}

uint32_t get_ccsds_seconds (ccsds_t* ccsds_ptr) {
  return (ccsds_ptr->ccsds_sec_hdr.seconds);
}

uint16_t get_ccsds_subseconds (ccsds_t* ccsds_ptr) {
  return (ccsds_ptr->ccsds_sec_hdr.subseconds);
}

void set_ccsds_payload_len (ccsds_t* ccsds_ptr, uint16_t len) {
  (ccsds_ptr->ccsds_hdr).pkt_len_H = (uint8_t)((len - 1) >> 8);  
  (ccsds_ptr->ccsds_hdr).pkt_len_L = (uint8_t)(len - 1);
}

uint8_t get_ccsds_pid (ccsds_t* ccsds_ptr) {
    uint16_t APID = get_ccsds_apid(ccsds_ptr);
    for (uint8_t PID=0; PID<NUMBER_OF_PID; PID++) {
        if(packet[PID].APID == APID) {
            return PID;
        }
    }
    return false;
}

// Command Functionality

bool execute_tc () { 
    switch (tc_this->cmd_id) {
    case CMD_REBOOT:        return cmd_reboot((uint8_t)tc_this->parameter[0]);
                            break;
    case CMD_SET_OPSMODE:   return cmd_set_opsmode((uint8_t)tc_this->parameter[0]);
                            break;
    }
    if (tm_this->opsmode == MODE_CHECKOUT) {
        switch (tc_this->cmd_id) { 
        case CMD_SET_PARAMETER: return cmd_set_parameter(tc_this->parameter, (char*)(tc_this->parameter + strlen(tc_this->parameter) + 1)); 
                                break;
        case CMD_LOAD_CONFIG:   return cmd_load_config((uint8_t)tc_this->parameter[0]);
                                break;                                                       
        case CMD_SAVE_CONFIG:   return cmd_save_config((uint8_t)tc_this->parameter[0], (const char*)&tc_this->parameter[1]); 
                                break;
        case CMD_FLUSH_FS:      return cmd_flush_fs();
                                break;
        case CMD_LIST_FS:       return cmd_list_fs();
                                break;    
        default:                sprintf(buffer,  "CCSDS command to %s not understood", subsystem[SS_THIS].name);
                                publish_event(STS_THIS, SS_TCTM, EVENT_CMD_FAIL, buffer);
                                return false;
                                break;
        }
    }
    else {
        publish_event (STS_THIS, SS_TCTM, EVENT_CMD_FAIL, "Not understood or only allowed in CHECKOUT mode");
        return false;   
    }
}

bool cmd_reboot (const uint8_t system) {
    switch(system) {
    case SS_THIS: // this
        sprintf(buffer, "Rebooting %s subsystem", subsystem[SS_THIS].name);
        publish_event(STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
        sync_archive_file();
        delay(1000);
        ESP.restart();
    break;
    case SS_OTHER: // other
        sprintf(buffer, "Sending reboot command to %s subsystem", subsystem[SS_OTHER].name);
        publish_event(STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
        tc_other->cmd_id = CMD_REBOOT;
        tc_other->parameter[0] = SS_OTHER;
        set_ccsds_payload_len((ccsds_t*)tc_other, 7); 
        publish_packet((ccsds_t*)tc_other);
    break;
    case SS_ANY: // both
        tc_other->cmd_id = CMD_REBOOT;
        tc_other->parameter[0] = SS_OTHER;
        set_ccsds_payload_len((ccsds_t*)tc_other, 7); 
        publish_packet((ccsds_t*)tc_other);
        sprintf(buffer, "Sending reboot command to %s and rebooting %s subsystem", subsystem[SS_OTHER].name, subsystem[SS_THIS].name);
        publish_event(STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
        sync_archive_file();
        delay(1000);
        ESP.restart();
    }     
    return true;
}

bool cmd_set_opsmode (const uint8_t opsmode) {
    return set_opsmode(opsmode); 
}

bool cmd_set_parameter (const char* parameter, const char* value) {
    if (set_parameter(parameter, value)) {
        publish_event(STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
        return true;
    }
    else {
        publish_event(STS_THIS, SS_THIS, EVENT_CMD_FAIL, "Command needs valid parameter/value");
        return false;
    }
}

bool cmd_load_config (const uint8_t bank) {
    if (load_config_bank(bank)) {
        return true;
    }
    else {
        return false;
    }
}

bool cmd_save_config (const uint8_t bank, const char* tag) {
    if (save_config_bank(bank, tag, cfg_this)) {
        sprintf (buffer, "Stored current %s configuration to bank %u as '%s'", subsystem[SS_THIS].name, bank, tag);
        publish_event (STS_THIS, SS_THIS, EVENT_CMD_RESP, buffer);
        publish_packet ((ccsds_t*)cfg_this);
        return true;
    }
    else {
        sprintf (buffer, "Failed to store current %s configuration to bank %u as '%s'", subsystem[SS_THIS].name, bank, tag);
        publish_event (STS_THIS, SS_THIS, EVENT_CMD_FAIL, buffer);
        return false;
    }
}

bool cmd_flush_fs () {
    if (flush_fs()) {
        return true;
    }
    else {
        return false;
    }
}

bool cmd_list_fs () {
    File file;
    File root;
    uint8_t ctr = 0;
    switch(tm_this->archive_fs) {
    case FS_LITTLEFS: root = LittleFS.open("/");
                      break;
    case FS_SD_MMC:   root = SD_MMC.open("/");
                      break;
    }
    while ((file = root.openNextFile())) {
        sprintf(buffer, "File: %s / %u kB / %04u-%02u-%02u %02u:%02u:%02u", file.path(), file.size(), year(file.getLastWrite()), month(file.getLastWrite()), day(file.getLastWrite()), hour(file.getLastWrite()), minute(file.getLastWrite()), second(file.getLastWrite()));
        publish_event (STS_THIS, SS_FS, EVENT_CMD_RESP, buffer); 
        ctr++;
    }
    switch(tm_this->archive_fs) {
    case FS_LITTLEFS:   sprintf(buffer, "FS: %s / size: %u kB / free: %u kB / entries: %u", filesystem[tm_this->archive_fs].name, LittleFS.totalBytes()/1024, fs_free(), ctr);
                        break;
    case FS_SD_MMC:     sprintf(buffer, "FS: %s / size: %llu kB / free: %u kB / entries: %u", filesystem[tm_this->archive_fs].name, SD_MMC.totalBytes()/1024, fs_free(), ctr);
                        break;
    }
    publish_event (STS_THIS, SS_FS, EVENT_CMD_RESP, buffer);        
    tm_this->fs_active = true;
    return true;
}

// OTA Functionality

void setup_ota() {
    ArduinoOTA.setPort(3232);
    char hostname[64];
    sprintf(hostname, "%s-%s", cfg_this->rocket_name, subsystem[SS_THIS].name);
    ArduinoOTA.setHostname(hostname);
    ArduinoOTA.setPassword(cfg_this->password);

    ArduinoOTA
    .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
        else // U_SPIFFS
        type = "filesystem";
        
        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
    })
    .onEnd([]() {
        Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    
    ArduinoOTA.begin();
    
    if(tm_this->wifi_ap_enabled or tm_this->wifi_sta_enabled) {
        publish_event (STS_THIS, SS_THIS, EVENT_INIT, "OTA capability initialized");
    }
    else {
        publish_event (STS_THIS, SS_THIS, EVENT_INIT, "OTA capability initialized but no wifi enabled");
    }
}

bool get_ntp_time() {
    tm timeinfo;
    configTime(0, 0, "ntp.telenet.be", "pool.ntp.org");
    if (getLocalTime(&timeinfo)) {
        setTime(timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
//        sprintf(buffer, "Time set through NTP: %04u-%02u-%02u %02u:%02u:%02u", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        sprintf(buffer, "Time set through NTP: %04u-%02u-%02u %02u:%02u:%02u", year(), month(), day(), hour(), minute(), second());
        publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
        tm_this->time_set = true;
        return true;
    }
    else {
        publish_event (STS_THIS, SS_THIS, EVENT_ERROR, "Failed to obtain NTP time");
        return false;
    }
}

// Support Functions

void print_ccsds_data (ccsds_t* ccsds_ptr) {
    Serial.print("APID: ");
    Serial.print(get_ccsds_apid(ccsds_ptr));
    Serial.print(" ctr: ");
    Serial.print(get_ccsds_packet_ctr(ccsds_ptr));
    Serial.print(" - ");
}

String get_hex_str (byte* blob, uint16_t length) {
    byte* pblob = blob;
    const char * hex = "0123456789ABCDEF";
    String hex_str;
    for(uint8_t i=0; i < length; i++, pblob++){
        hex_str += hex[(*pblob>>4) & 0xF];
        hex_str += hex[ *pblob     & 0xF];
    }
    return (hex_str);
}