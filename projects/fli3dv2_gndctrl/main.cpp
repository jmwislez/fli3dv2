/* 
 * Fli3dv2 - Ground Control 
 *
 * Receives tm packets from Fli3dv2 payload over ESP-NOW, and send them to Yamcs over USB
 * Receives tc packets from Yamcs over USB, and send them to Fli3dv2 payload over ESP-NOW
 *
 * To compile in Visual Studio with PlatformIO, for ESP32 Dev Module.
 *
 */

// Set versioning
#define SW_VERSION "Fli3d gndctrl v0.1.0 (20260805)"

// Libraries
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <fli3dv2.h>

// Global variables used in this file
//extern tc_gndctrl_t     tc_gndctrl;
//extern sts_gndctrl_t    sts_gndctrl;
extern tm_esp32_t   tm_esp32;
extern tm_gndctrl_t tm_gndctrl;
extern cfg_packet_t cfg_gndctrl;
extern var_t        var;

tm_gndctrl_t        *tm_this = &tm_gndctrl;
tc_packet_t         *tc_this = &tc_gndctrl;
tc_packet_t         *tc_other = &tc_esp32;
sts_packet_t        *sts_this = &sts_gndctrl;
tmr_gndctrl_t       *tmr_this = &tmr_gndctrl;
cfg_packet_t        *cfg_this = &cfg_gndctrl;

// ROUTING (PID)
//                                              0: TC_ESP32 
//                                              |  1: TC_ESP32CAM 
//                                              |  |  2: TC_GNDCTRL 
//                                              |  |  |  3: STS_ESP32 
//                                              |  |  |  |  4: STS_ESP32CAM 
//                                              |  |  |  |  |  5: STS_GNDCTRL 
//                                              |  |  |  |  |  |  6: TM_ESP32 
//                                              |  |  |  |  |  |  |  7: TM_GPS 
//                                              |  |  |  |  |  |  |  |  8: TM_MOTION 
//                                              |  |  |  |  |  |  |  |  |  9: TM_PRESSURE
//                                              |  |  |  |  |  |  |  |  |  |  A: TM_RADIO
//                                              |  |  |  |  |  |  |  |  |  |  |  B: TM_ESP32CAM
//                                              |  |  |  |  |  |  |  |  |  |  |  |  C: TM_CAMERA
//                                              |  |  |  |  |  |  |  |  |  |  |  |  |  D: TM_GNDCTRL
//                                              |  |  |  |  |  |  |  |  |  |  |  |  |  |  E: tmr_ESP32
//                                              |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  F: tmr_ESP32CAM
//                                              |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  G: tmr_GNDCTRL
//                                              |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  H: cfg_ESP32
//                                              |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  I: cfg_ESP32CAM
//                                              |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  J: cfg_GNDCTRL
//                                              0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F  G  H  I  J
bool default_routing_espnow[NUMBER_OF_PID] =  { 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
bool default_routing_radio[NUMBER_OF_PID] =   { 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
bool default_routing_serial[NUMBER_OF_PID] =  { 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
bool default_routing_archive[NUMBER_OF_PID] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

void setup_timer() {
    var.next_second = 1000*(millis()/1000) + 1000;
    var.next_tx_time = millis();
}

void set_esp32_time() {
    sprintf(buffer, "Sending time sync TC to ESP32 (%lu)", now());
    publish_event(STS_THIS, SS_TCTM, EVENT_INFO, buffer);
    uint8_t len = snprintf(NULL, 0, "timestamp%c%lu%c", '\0', now(), '\0');
    snprintf(buffer, len + 1, "timestamp%c%lu%c", '\0', now(), '\0');
    publish_cmd(TC_ESP32, CMD_SET_PARAMETER, (byte*)buffer, len + 1);
}

void setup() {
    // Initial settings configuration
    init_config();
    
    // Serial port to Yamcs
    Serial.begin(cfg_this->serial_baud);
    setup_serialtransfer(Serial);

    // Startup telemetry
    init_ccsds();
    sprintf (buffer, "%s started on %s", SW_VERSION, subsystem[SS_THIS].name); 
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer); 
    publish_packet((ccsds_t*)tm_this);
    publish_packet((ccsds_t*)cfg_this);

    // Load stored configuration
    if(init_boot_config()) {
        load_config_bank(cfg_this->cfg_boot.boot_bank);
    }
    publish_packet((ccsds_t*)tm_this);
    publish_packet((ccsds_t*)cfg_this);

    // Connect to wifi to get time from NTP server
    setup_wifi();
    enable_wifi_services();
    get_ntp_time();
    disable_wifi_services();

    // Set up ESP-NOW for communication with Fli3dv2 rocket
    setup_espnow();
    publish_packet((ccsds_t*)tm_this);

    // Set up 433 MHz radio for communication with Fli3dv2 rocket
    setup_radio();

    // Set up file system for local storage of telemetry data
    setup_fs();
    setup_sd();
    setup_archive();

    // Initialisation complete
    setup_timer();
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, "Initialisation complete");  
    set_opsmode(cfg_this->target_opsmode);
}
 
void loop() {
    check_serialtransfer_rx();
    check_radio_rx();
    process_rx_queue();
    if (millis()>=var.next_tx_time) {
        process_tx_queue();
        var.next_tx_time += 20;
    }
    if (millis()>=var.next_second) {
        if (tm_this->time_set and tm_esp32.packet_ctr>0 and !tm_esp32.time_set) {
            set_esp32_time();
        }
        publish_packet((ccsds_t*)tm_this);
        var.next_second+=1000;
    }
    if (tm_this->opsmode == MODE_MAINTENANCE) {
        // In maintenance mode, we can check for OTA and FTP
        if (cfg_this->ota_enable) {
            ArduinoOTA.handle();
            tm_this->ota_enabled = true;
        }
        if (tm_this->ftp_enabled) {
            //ftp_check(cfg_this->buffer_fs);
        }
    }
}