/* 
 * Fli3dv2 - ESP32CAM camera module
 *
 * Sends tm packets to Fli3dv2 ESP32 over serial
 * Receives tc packets from Fli3dv2 ESP32 over serial
 * Acquires images from camera and saves them to SD card and sends them over Wifi
 *
 * To compile in Visual Studio with PlatformIO, for ESP32CAM
 *
 */

// Set versioning
#define SW_VERSION "Fli3d ESP32CAM v0.1.0 (20260809)"

// Libraries
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <fli3dv2.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// Global variables used in this file
extern tm_esp32cam_t    tm_esp32cam;
extern cfg_packet_t     cfg_esp32cam;
extern var_t            var;

tm_esp32cam_t        *tm_this = &tm_esp32cam;
tc_packet_t         *tc_this = &tc_esp32cam;
tc_packet_t         *tc_other = &tc_esp32;
sts_packet_t        *sts_this = &sts_esp32cam;
tmr_esp32cam_t       *tmr_this = &tmr_esp32cam;
cfg_packet_t        *cfg_this = &cfg_esp32cam;

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
//                                              |  |  |  |  |  |  |  |  |  |  |  |  |  |  E: TMR_ESP32
//                                              |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  F: TMR_ESP32CAM
//                                              |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  G: TMR_GNDCTRL
//                                              |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  H: CFG_ESP32
//                                              |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  I: CFG_ESP32CAM
//                                              |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  J: CFG_GNDCTRL
//                                              0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F  G  H  I  J
bool default_routing_espnow[NUMBER_OF_PID] =  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
bool default_routing_radio[NUMBER_OF_PID] =   { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
bool default_routing_serial[NUMBER_OF_PID] =  { 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0 };
bool default_routing_archive[NUMBER_OF_PID] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

extern bool setup_ov2640();

void setup_timer() {
    var.next_second = 1000*(millis()/1000) + 1000;
    var.next_tx_time = millis();
}

void setup() {
    // Initial settings configuration
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    init_config();
    
    // Serial port to Fli3dv2 ESP32
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

    // Connect to wifi
    setup_wifi();
    enable_wifi_services();
    get_ntp_time();
    publish_packet((ccsds_t*)tm_this);

    // Set up file system for local storage of telemetry data
    setup_fs();
    setup_sd();
    setup_archive();

    // Start camera
    setup_ov2640();

    // Initialisation complete
    setup_timer();
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, "Initialisation complete");  
    set_opsmode(cfg_this->target_opsmode);
}
 
void loop() {
    check_serialtransfer_rx();
    process_rx_queue();
    if (millis()>=var.next_tx_time) {
        process_tx_queue();
        var.next_tx_time += 20;
    }
    if (millis()>=var.next_second) {
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