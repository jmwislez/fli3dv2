/* 
 *  Fli3dv2 - core system functionality
 *  
 *  To compile in Visual Studio with PlatformIO, with ESP32 core v3.3.x, for ESP32 MH-ET LIVE MiniKit.
 *  Use partition scheme "Default with spiffs" or custom partition scheme "Fli3d ESP32 (OTA/maximized SPIFFS)".
 */

// Set versioning
#define SW_VERSION "Fli3d ESP32 v1.99.0 (20260807)"

// Set functionality to compile
#define RADIO
#define PRESSURE
#define MOTION
#define GPS
//#define CAMERA

// Libraries
#include <Arduino.h>
#include <fli3dv2.h>
#include "fli3dv2_esp32.h"
#include <ArduinoOTA.h>
#include "esp_timer.h"

// Global variables used in this file
bool reset_gps_timer, separation_sts_changed; // TODO: keep?
extern char buffer[BUFFER_MAX_SIZE];
extern tm_esp32_t   tm_esp32;
extern cfg_packet_t cfg_esp32;
extern var_t        var;

tm_esp32_t          *tm_this = &tm_esp32;
tc_packet_t         *tc_this = &tc_esp32;
tc_packet_t         *tc_other = &tc_esp32cam;
sts_packet_t        *sts_this = &sts_esp32;
tmr_esp32_t         *tmr_this = &tmr_esp32;
cfg_packet_t        *cfg_this = &cfg_esp32;

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
bool default_routing_espnow[NUMBER_OF_PID] =  { 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0 };
bool default_routing_serial[NUMBER_OF_PID] =  { 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0 };
bool default_routing_radio[NUMBER_OF_PID] =   { 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0 };
bool default_routing_archive[NUMBER_OF_PID] = { 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0 };

void sendTM(void *arg) {
    publish_packet((ccsds_t*)tm_this);
}

void checkTX(void *arg) {
    process_tx_queue();
}

void setup() {
    // Initial configuration
    init_config();

    Serial.begin(cfg_this->serial_baud); // debug output
    if(cfg_esp32.gps_enable) {
        Serial1.begin(cfg_this->serial_baud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); // communication with GPS
    }
    if (cfg_esp32.serial_rx_enable or cfg_esp32.serial_tx_enable) {
        Serial2.begin(cfg_this->serial_baud, SERIAL_8N1, ESP32CAM_RX_PIN, ESP32CAM_TX_PIN); // communication with ESP32CAM
        setup_serialtransfer(Serial2);
    }
    init_ccsds();
    sprintf (buffer, "%s started on %s", SW_VERSION, subsystem[SS_THIS].name); 
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);  
    publish_packet((ccsds_t*)tm_this);
    publish_packet((ccsds_t*)cfg_this);
    setup_power();

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

    // Load configuration
    setup_gpio();
    if(cfg_this->cfg_boot.boot_bank != 255) {
        if(init_boot_config()) {
            load_config_bank(cfg_this->cfg_boot.boot_bank);
        }
        publish_packet((ccsds_t*)cfg_this);
    }

    // Set up ESP-NOW
    setup_wifi();
    setup_espnow();

    // Set up radio
    setup_radio();

    // Set up file system
    setup_fs();
    setup_sd();
    setup_archive();

    //
    setup_separation();
    setup_buzzer();

    #ifdef CAMERA
    if (cfg_esp32.camera_enable) { 
        esp32.camera_enabled = true;
    }
    #endif // CAMERA 

    #ifdef GPS
    if (cfg_esp32.gps_enable) {
        if((tm_esp32.gps_enabled = setup_neo6mv2())) {
        }
    }
    #endif // GPS

    #ifdef MOTION
    if (cfg_esp32.motion_enable) {
        if ((tm_esp32.motion_enabled = setup_icm20948())) {
            //mpu6050_calibrate();    // TODO: to be done offline on loose sensor, then put calibration values in configuration file
            //mpu6050_checkConfig(); 
            //mpu6050_printConfig(); 
        }
    }
    #endif // MOTION

    #ifdef PRESSURE
    if (cfg_esp32.pressure_enable) {
        if ((tm_esp32.pressure_enabled = setup_bmp388())) {
            //bmp388_calibrate();
        }
    }
    #endif // PRESSURE

    // Initialisation complete
    setup_timer();
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, "ESP32 initialisation complete");  
    set_opsmode(cfg_this->target_opsmode);
}

void loop() {
    static uint32_t start_millis;
  
    check_serialtransfer_rx();
    check_radio_rx();
    process_rx_queue();
    tmr_esp32.millis = millis();

    if (tm_this->opsmode == MODE_MAINTENANCE) {
        // In maintenance mode, we can check for OTA and FTP
        if (cfg_this->ota_enable) {
            ArduinoOTA.handle();
            tm_this->ota_enabled = true;
        }
        if (tm_this->ftp_enabled) {
            //ftp_check (cfg_this->buffer_fs);
        }
    }
    else { 
        // separation status (monitored via interrupt)
        if (separation_sts_changed) {
            separation_publish();
            separation_sts_changed = false;
        }
    
        // BMP388 pressure sensor
        #ifdef PRESSURE
        else if (tmr_esp32.millis >= var.next_pressure_time and tm_esp32.pressure_enabled) {
            start_millis = millis();
            if (tm_esp32.pressure_active = acquire_BMP()) {
                publish_packet ((ccsds_t*)&tm_pressure);
            }
            //else {
                // will try to reset pressure sensor once, and then give up
                //esp32.pressure_enabled = setup_icm20948();
            //}
            var.next_pressure_time = tmr_esp32.millis + var.pressure_interval;
            tmr_esp32.pressure_duration += millis() - start_millis;
        } 
        #endif // PRESSURE

        // ICM-20948 accelerometer/gyroscope/magnetometer
        #ifdef MOTION
        else if (tmr_esp32.millis >= var.next_motion_time and tm_esp32.motion_enabled) {
            start_millis = millis();
            if (tm_esp32.motion_active = acquire_IMU()) {
                publish_packet ((ccsds_t*)&tm_motion);
            }
            //else {
                // will try to reset accelerometer once, and then give up
                //esp32.motion_enabled = setup_icm20948();
            //}
            var.next_motion_time = tmr_esp32.millis + var.motion_interval;
            tmr_esp32.motion_duration += millis() - start_millis;
        } 
        #endif // MOTION
        
        // NEO6MV2 GPS
        #ifdef GPS
        else if (tmr_esp32.millis >= var.next_gps_time and tm_esp32.gps_enabled) {
            start_millis = millis();
            /*if (check_gps()) {
                publish_packet ((ccsds_t*)&tm_gps);
                reset_gps_timer = true;
                var.next_gps_time = tmr_esp32.millis + 1000;  // 1Hz as long as no data 
            }
            if (var.do_gps) {
                publish_packet ((ccsds_t*)&tm_gps);
                var.do_gps = false;
            } */
            if (tm_esp32.gps_active = acquire_gps()) {
                publish_packet ((ccsds_t*)&tm_gps);
                var.next_gps_time = tmr_esp32.millis + var.gps_interval;
            }
            else {
                publish_packet ((ccsds_t*)&tm_gps);
                var.next_gps_time = tmr_esp32.millis + 1000;  // 1Hz as long as no data
            }
            
            tmr_esp32.gps_duration += millis() - start_millis;
        }
        #endif // GPS
    }  
}