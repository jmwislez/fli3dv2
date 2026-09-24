/* 
 *  Fli3dv2 - core system functionality
 *  
 *  To compile in Visual Studio with PlatformIO, with ESP32 core v3.3.x, for ESP32 MH-ET LIVE MiniKit.
 *  Use partition scheme "Default with spiffs" or custom partition scheme "fli3d_esp32.csv".
 */

// Set versioning
#define SW_VERSION "1.99.0"
#define SW_DATE "20260924"

// Set functionality to compile
//#define RADIO
#define RS41
//#define COREMESH
//#define GPRS
//#define SEPARATION
//#define BATTERY
#define PRESSURE
#define PRESSURE2
#define MOTION
#define GPS
#define CAMERA
#define TIMER

// Libraries
#include <Arduino.h>
#include <fli3dv2.h>
#include "fli3dv2_esp32.h"
#include <ArduinoOTA.h>
#include "esp_timer.h"
#include "Wire.h" // for debug_i2c_scanner only

// Global variables used in this file
#ifdef GPS
bool reset_gps_timer; // TODO: keep?
#endif
#ifdef SEPARATION
bool separation_sts_changed; // TODO: keep?
#endif
extern char buffer[BUFFER_MAX_SIZE];
extern tm_esp32_t   tm_esp32;
extern cfg_packet_t cfg_esp32;
extern tmr_packet_t tmr_esp32;
extern var_t        var;

tm_esp32_t          *tm_this = &tm_esp32;
tc_packet_t         *tc_this = &tc_esp32;
tc_packet_t         *tc_other = &tc_esp32cam;
sts_packet_t        *sts_this = &sts_esp32;
tmr_packet_t        *tmr_this = &tmr_esp32;
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
bool default_routing_serial[NUMBER_OF_PID] =  { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
bool default_routing_radio[NUMBER_OF_PID] =   { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
bool default_routing_archive[NUMBER_OF_PID] = { 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0 };

void set_esp32cam_time() {
    sprintf(buffer, "Sending time sync TC to ESP32CAM (%lu)", now());
    publish_event(STS_THIS, SS_TCTM, EVENT_INFO, buffer);
    uint8_t len = snprintf(NULL, 0, "timestamp%c%lu%c", '\0', now(), '\0');
    snprintf(buffer, len + 1, "timestamp%c%lu%c", '\0', now(), '\0');
    publish_cmd(TC_ESP32CAM, CMD_SET_PARAMETER, (byte*)buffer, len + 1);
}

void sendTM(void *arg) {
    publish_packet((ccsds_t*)tm_this);
    if (tm_this->time_set and tm_esp32cam.opsmode!=MODE_INIT and !tm_esp32cam.time_set) {
        set_esp32cam_time();
    }
}

void checkTX(void *arg) {
    process_tx_queue();
}

void setup() {
    // Initial configuration
    init_config();

    Serial.begin(115200); // debug output
    #ifdef GPS
    if(cfg_esp32.gps_enable) {
        Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); // communication with GPS
    }
    #endif
    #ifdef CAMERA
    if (cfg_esp32.serial_rx_enable or cfg_esp32.serial_tx_enable) {
        Serial2.begin(115200, SERIAL_8N1, ESP32CAM_RX_PIN, ESP32CAM_TX_PIN); // communication with ESP32CAM
        setup_serialtransfer(Serial2);
        tm_esp32.serial_rx_enabled = cfg_esp32.serial_rx_enable;
        tm_esp32.serial_tx_enabled = cfg_esp32.serial_tx_enable;
    }
    #endif
    #ifdef RS41
    if(cfg_esp32.radio_tx_enable) {
        tm_esp32.radio_tx_enabled = setup_rs41();
    }
    #endif

    init_ccsds();
    sprintf (buffer, "Fli3d ESP32 v%s [%s] started for %s [%s]", SW_VERSION, SW_DATE, cfg_this->rocket_name, reset_reason[esp_reset_reason()].name); 
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);  
    publish_packet((ccsds_t*)tm_this);
    publish_packet((ccsds_t*)cfg_this);
    #ifdef BATTERY
    setup_power();
    #endif

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
    #ifdef RADIO
    setup_radio();
    #endif

    // Set up file system
    setup_fs();
    setup_sd();
    setup_archive();

    //
    #ifdef SEPARATION
    setup_separation();
    #endif
    setup_buzzer();

    #ifdef CAMERA
    if (cfg_esp32.camera_enable) { 
        tm_esp32.camera_enabled = true;
    }
    #endif // CAMERA 

    #ifdef GPS
    if (cfg_esp32.gps_enable) {
        tm_esp32.gps_enabled = setup_neo6mv2();
    }
    #endif // GPS

    #ifdef PRESSURE
    if (cfg_esp32.pressure_enable) {
        if ((tm_esp32.pressure_enabled = setup_bmp388())) {
            zero_bmp388();
        }
    }
    #endif // PRESSURE
    
    #ifdef PRESSURE2
    if (cfg_esp32.pressure2_enable) {
        if ((tm_esp32.pressure2_enabled = setup_bmp280())) {
            zero_bmp280();
        }
    }
    #endif // PRESSURE2

    #ifdef MOTION
    if (cfg_esp32.motion_enable) {
        if ((tm_esp32.motion_enabled = setup_icm20948())) {
            //mpu6050_calibrate();    // TODO: to be done offline on loose sensor, then put calibration values in configuration file
            //mpu6050_checkConfig(); 
            //mpu6050_printConfig(); 
        }
    }
    #endif // MOTION

    // Initialisation complete
    setup_timer();
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, "ESP32 initialisation complete");  
    set_opsmode(cfg_this->target_opsmode);
}

void loop() {

    check_serialtransfer_rx();
    #ifdef RADIO
    check_radio_rx();
    #endif
    process_rx_queue();

    var.now = millis();

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
        if(false) {
            // dummy to allow all else if below
        }
        // separation status (monitored via interrupt)
        #ifdef SEPARATION
        else if (separation_sts_changed) {
            separation_publish();
            separation_sts_changed = false;
        }
        #endif
    
        // BMP388 pressure sensor
        #ifdef PRESSURE
        else if (var.now >= var.next_pressure_time and tm_esp32.pressure_enabled) {
            if (tm_esp32.pressure_active = acquire_bmp388()) {
                publish_packet ((ccsds_t*)&tm_pressure);
            }
            //else {
                // will try to reset pressure sensor once, and then give up
                //esp32.pressure_enabled = setup_icm20948();
            //}
            var.next_pressure_time = var.now + var.pressure_interval;
        } 
        #endif // PRESSURE

        // BMP280 pressure sensor
        #ifdef PRESSURE2
        else if (var.now >= var.next_pressure_time and tm_esp32.pressure_enabled) {
            if (tm_esp32.pressure2_active = acquire_bmp280()) {
                publish_packet ((ccsds_t*)&tm_pressure);
            }
            //else {
                // will try to reset pressure sensor once, and then give up
                //esp32.pressure_enabled = setup_icm20948();
            //}
            var.next_pressure_time = var.now + var.pressure_interval;
        } 
        #endif // PRESSURE2

        // ICM-20948 accelerometer/gyroscope/magnetometer
        #ifdef MOTION
        else if (var.now >= var.next_motion_time and tm_esp32.motion_enabled) {
            if (tm_esp32.motion_active = acquire_icm20948()) {
                publish_packet ((ccsds_t*)&tm_motion);
                publish_packet ((ccsds_t*)&tm_radio); //TODO: remove (is test)
            }
            //else {
                // will try to reset accelerometer once, and then give up
                //esp32.motion_enabled = setup_icm20948();
            //}
            var.next_motion_time = var.now + var.motion_interval;
        } 
        #endif // MOTION
        
        // NEO6MV2 GPS
        #ifdef GPS
        else if (tm_esp32.gps_enabled and var.now >= var.next_gps_time) {
            if (tm_esp32.gps_active = acquire_neo6mv2()) {
                var.next_gps_time = var.now + var.gps_interval;
            }
            else {
                var.next_gps_time = var.now + 1000;  // 1Hz as long as no data 
            }
            publish_packet ((ccsds_t*)&tm_gps);
        }
        #endif // GPS

        //RS41
        #ifdef RS41
        else if (tm_esp32.radio_tx_enabled and var.now >= var.next_rs41_time) {
            send_radio_packet_to_rs41();
            var.next_rs41_time = var.now + 1000;
        }
        #endif
    }  
}