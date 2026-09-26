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
#define SW_VERSION "1.0.0"
#define SW_DATE "20260926"

// Libraries
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <fli3dv2.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "CameraController.h"
#include "WebInterface.h"
#include "esp_timer.h"

// Global variables used in this file
extern tm_esp32cam_t    tm_esp32cam;
extern cfg_packet_t     cfg_esp32cam;
extern tmr_packet_t     tmr_esp32cam;
extern var_t            var;

tm_esp32cam_t       *tm_this = &tm_esp32cam;
tc_packet_t         *tc_this = &tc_esp32cam;
tc_packet_t         *tc_other = &tc_esp32;
sts_packet_t        *sts_this = &sts_esp32cam;
tmr_packet_t        *tmr_this = &tmr_esp32cam;
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
//                                              |  |  |  |  |  |  |  |  |  |  A: TM_SUMMARY
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
//bool default_routing_espnow[NUMBER_OF_PID] ={ 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0 };
bool default_routing_radio[NUMBER_OF_PID] =   { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
bool default_routing_serial[NUMBER_OF_PID] =  { 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0 };
//bool default_routing_serial[NUMBER_OF_PID] ={ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
bool default_routing_archive[NUMBER_OF_PID] = { 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0 };

CameraController cameraController;
WebInterface webInterface;

void sendTM(void *arg) {
    publish_packet((ccsds_t*)tm_this);
    if (tm_esp32cam.camera_enabled) {
        publish_packet((ccsds_t*)&tm_camera);
    }
}

void checkTX(void *arg) {
    process_tx_queue();
}

void checkRX(void *arg) {
    // Process incoming packets
    check_serialtransfer_rx();
    process_rx_queue();
}

void SyncArchive(void *arg) {
    // Ensure storage of archive data
    sync_archive_file();
    camera_sync_video();
}
    

void setup() {
    // Initial settings configuration
    //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    init_config();
    
    // Serial port to Fli3dv2 ESP32
    if (cfg_esp32cam.serial_rx_enable or cfg_esp32cam.serial_tx_enable) {
        Serial.begin(57600);
        setup_serialtransfer(Serial);
        tm_esp32cam.serial_rx_enabled = cfg_esp32cam.serial_rx_enable;
        tm_esp32cam.serial_tx_enabled = cfg_esp32cam.serial_tx_enable;
    }

    // Startup telemetry
    init_ccsds();
    sprintf (buffer, "Fli3d ESP32CAM v%s [%s] started for %s [%s]", SW_VERSION, SW_DATE, cfg_this->rocket_name, reset_reason[esp_reset_reason()].name); 
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer); 
    publish_packet((ccsds_t*)tm_this);
    publish_packet((ccsds_t*)cfg_this);

    // Start sending tm packets every second
    esp_timer_create_args_t timer_argsTM = {
        .callback = &sendTM,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "BackgroundTaskTimer0"
    };

    esp_timer_handle_t timer_handleTM;
    esp_timer_create(&timer_argsTM, &timer_handleTM);
    esp_timer_start_periodic(timer_handleTM, 1000000);
    sleep(10);

    // Start checking the send queue every 100 ms
    esp_timer_create_args_t timer_argsTX = {
        .callback = &checkTX,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "BackgroundTaskTimer1"
    };

    esp_timer_handle_t timer_handleTX;
    esp_timer_create(&timer_argsTX, &timer_handleTX);
    esp_timer_start_periodic(timer_handleTX, 100000);
    sleep(10);

    // Start checking the receive queue every 500 ms
    esp_timer_create_args_t timer_argsRX = {
        .callback = &checkRX,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "BackgroundTaskTimer2"
    };

    esp_timer_handle_t timer_handleRX;
    esp_timer_create(&timer_argsRX, &timer_handleRX);
    esp_timer_start_periodic(timer_handleRX, 500000);
    sleep(10);

    // Sync the filesystem every 5 minutes
    esp_timer_create_args_t timer_argsArchive = {
        .callback = &SyncArchive,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "BackgroundTaskTimer3"
    };

    esp_timer_handle_t timer_handleArchive;
    esp_timer_create(&timer_argsArchive, &timer_handleArchive);
    esp_timer_start_periodic(timer_handleArchive, 300000000);

    // Load stored configuration
    if(init_boot_config()) {
        load_config_bank(cfg_this->cfg_boot.boot_bank);
    }
    publish_packet((ccsds_t*)cfg_this);

    // Connect to wifi
    setup_wifi();
    enable_wifi_services();
    get_ntp_time();

    // Set up file system for local storage of telemetry data
    setup_fs();
    setup_sd();
    create_today_directory();
    setup_archive();

    // Initialize FTP server
    if (cfg_this->ftp_enable) {
        setup_ftp ();
    }

    // Initialize camera
    if (cfg_esp32cam.camera_enable) {
        tm_esp32cam.camera_enabled = cameraController.begin();    
    }

    // Initialize web interface
    if (tm_esp32cam.wifi_sta_enabled or tm_esp32cam.wifi_ap_enabled) {
        tm_esp32cam.webserver_enabled = webInterface.begin(&cameraController);
    }

    // Initialisation complete
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, "ESP32CAM initialisation complete");  
    set_opsmode(cfg_this->target_opsmode);
}


void loop() {
    switch (tm_this->opsmode) {
    case MODE_CHECKOUT:
    case MODE_NOMINAL:
        // Camera operations
        if(tm_esp32cam.camera_enabled) {
            cameraController.process();
        }
        if(tm_esp32cam.webserver_enabled) {
            webInterface.process();   
        } 
        break;
    case MODE_MAINTENANCE:
        // OTA and FTP
        if (cfg_this->ota_enable) {
            ArduinoOTA.handle();
            tm_this->ota_enabled = true; // TODO: put somewhere else and add active
        }
        if (tm_this->ftp_enabled) {
            handle_ftp();
            tm_this->ftp_enabled = true; // TODO: put somewhere else and add active
        }
    }
}