/*
 * Fli3d - Library (ESP-NOW, WiFi, TM/TC functionality)
 */
 
#ifndef _FLI3DV2_H_
#define _FLI3DV2_H_
#define LIB_VERSION "Fli3dv2 lib 1.99.0/20260727"

#include <Arduino.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <SerialTransfer.h>
#include <LinkedList.h>
#include <FS.h>
#include <LittleFS.h>
#include <SPI.h>
#include <SD_MMC.h>
#include <TimeLib.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
//#include <ESPFtpServer.h>

#define BUFFER_MAX_SIZE           512
#define PARAMETER_MAX_SIZE        128

// Pin assignment for ESP32 MH-ET minikit board

#define I2C_SCL_PIN               22   // default SCL I2C pin on ESP32
#define I2C_SDA_PIN               21   // default SDA I2C pin on ESP32
#define SEP_STS_PIN               2 
#define GPS_RX_PIN                16   // UART2 TX interface on ESP32
#define GPS_TX_PIN                17   // UART2 RX interface on ESP32
#define ESP32CAM_RX_PIN           39   // UART1 TX interface on ESP32
#define ESP32CAM_TX_PIN           32   // UART1 RX interface on ESP32
#define BAT_V_PIN                 34
#define BZ_PIN                    14
#define SET1_PIN                  25
#define SET2_PIN                  4
#define SET3_PIN                  26
#define SET4_PIN                  13
#define CUT_CMD_PIN               33
#define RADIO_CS_PIN              5
#define RADIO_SCK_PIN             18
#define RADIO_MISO_PIN            19
#define RADIO_MOSI_PIN            23 
#define RADIO_GDO0_PIN            27 
#define RADIO_GDO2_PIN            35         

//  MDB DEFINITION

#ifdef PLATFORM_ESP32
#define SS_ESPNOW_PEER  SS_GNDCTRL
#define SS_SERIAL_PEER  SS_ESP32CAM
#define SS_THIS      SS_ESP32       // define default subsystem
#define SS_OTHER     SS_ESP32CAM    // define counterpart subsystem
#define TM_THIS      TM_ESP32       // define default system TM packet
#define TM_OTHER     TM_ESP32CAM    // define counterpart system TM packet
#define TC_THIS      TC_ESP32       // define default system TC packet
#define TC_OTHER     TC_ESP32CAM    // define default TC packet destination
#define STS_THIS     STS_ESP32      // define default system STS packet
#define STS_OTHER    STS_ESP32CAM   // define counterpart system STS packet
#endif
#ifdef PLATFORM_ESP32CAM
#define SS_ESPNOW_PEER  SS_GNDCTRL
#define SS_SERIAL_PEER  SS_ESP32
#define SS_THIS      SS_ESP32CAM    // define default subsystem
#define SS_OTHER     SS_ESP32       // define counterpart subsystem
#define TM_THIS      TM_ESP32CAM    // define default system TM packet
#define TM_OTHER     TM_ESP32       // define counterpart system TM packet
#define TC_THIS      TC_ESP32CAM    // define default system TC packet
#define TC_OTHER     TC_ESP32       // define default TC packet destination
#define STS_THIS     STS_ESP32CAM   // define default system STS packet
#define STS_OTHER    STS_ESP32      // define counterpart system STS packet
#endif
#ifdef PLATFORM_GNDCTRL
#define SS_ESPNOW_PEER  SS_ESP32
#define SS_SERIAL_PEER  SS_YAMCS
#define SS_THIS      SS_GNDCTRL     // define default subsystem
#define SS_OTHER     SS_ESP32       // define counterpart subsystem
#define SS_PEER      SS_ESP32       // define counterpart subsystem
#define TM_THIS      TM_GNDCTRL     // define default system TM packet
#define TM_OTHER     TM_ESP32       // define counterpart system TM packet
#define TC_THIS      TC_GNDCTRL     // define default system TC packet
#define TC_OTHER     TC_ESP32       // define default TC packet destination
#define STS_THIS     STS_GNDCTRL    // define default system STS packet
#define STS_OTHER    STS_ESP32      // define counterpart system STS packet
#endif

// PIDs
#define TC_ESP32               0  // APID 42 (2a)
#define TC_ESP32CAM            1  // APID 43 (2b)
#define TC_GNDCTRL             2  // APID 44 (2c)
#define STS_ESP32              3  // APID 45 (2d)
#define STS_ESP32CAM           4  // APID 46 (2e)
#define STS_GNDCTRL            5  // APID 47 (2f)
#define TM_ESP32               6  // APID 48 (30)
#define TM_GPS                 7  // APID 49 (31)
#define TM_MOTION              8  // APID 50 (32)
#define TM_PRESSURE            9  // APID 51 (33)
#define TM_RADIO               10 // APID 52 (34) -> make TM_SUMMARY instead?
#define TM_ESP32CAM            11 // APID 53 (35)
#define TM_CAMERA              12 // APID 54 (36)
#define TM_GNDCTRL             13 // APID 55 (37)
#define TIMER_ESP32            14 // APID 56 (38)
#define TIMER_ESP32CAM         15 // APID 57 (39)
#define TIMER_GNDCTRL          16 // APID 58 (3a)
#define CONFIG_ESP32           17 // APID 59 (3b)
#define CONFIG_ESP32CAM        18 // APID 60 (3c)
#define CONFIG_GNDCTRL         19 // APID 61 (3d)
#define NUMBER_OF_PID          20

// subsystem
#define SS_POWER               0
#define SS_ESP32               1
#define SS_GPS                 2
#define SS_MOTION              3
#define SS_PRESSURE            4
#define SS_RADIO               5
#define SS_SEPARATION          6
#define SS_PARACHUTE           7
#define SS_BUZZER              8
#define SS_ESP32CAM            9
#define SS_CAMERA              10
#define SS_FS                  11
#define SS_SD                  12
#define SS_ESPNOW              13
#define SS_SERIAL              14
#define SS_TCTM                15
#define SS_ARCHIVE             16
#define SS_TIMER               17
#define SS_CONFIG              18
#define SS_GNDCTRL             19
#define SS_YAMCS               20
#define SS_ANY                 21
#define SS_NONE                22

// packet types and subtypes
#define PKT_TM                 0 // type and subtype
#define PKT_TC                 1 // type and subtype
#define PKT_STS                2 // subtype
#define PKT_TIMER              3 // subtype
#define PKT_CONFIG             4 // subtype

// Commands (also update xtce.fli3d.xml and name_t command[] in fli3dv2.cpp)
#define CMD_REBOOT             0
#define CMD_SET_OPSMODE        1
#define CMD_SET_PARAMETER      2
#define CMD_LOAD_CONFIG        3
#define CMD_SAVE_CONFIG        4
#define CMD_FLUSH_FS           5
#define CMD_LIST_FS            6

// event_types
#define EVENT_INIT             0
#define EVENT_INFO             1
#define EVENT_WARNING          2
#define EVENT_ERROR            3
#define EVENT_CMD              4
#define EVENT_CMD_ACK          5
#define EVENT_CMD_RESP         6
#define EVENT_CMD_FAIL         7

// communication channels
#define COMMS_WIFI             0
#define COMMS_ESPNOW           1
#define COMMS_SERIAL           2
#define COMMS_RADIO            3
#define COMMS_FS               4
#define COMMS_SD               5
#define COMMS_ANY              6

// opsmode
#define MODE_INIT              0
#define MODE_CHECKOUT          1
#define MODE_NOMINAL           2
#define MODE_MAINTENANCE       3

// file systems
#define FS_NONE                0
#define FS_LITTLEFS            1
#define FS_SD_MMC              2
#define FS_EEPROM              3

/*
// state
#define STATE_STATIC           0
#define STATE_THRUST           1
#define STATE_FREEFALL         2
#define STATE_PARACHUTE        3
extern const char stateName[4][10];

// cammode
#define CAM_INIT               0
#define CAM_IDLE               1
#define CAM_SINGLE             2
#define CAM_STREAM             3
extern const char cameraModeName[4][7];

// cam resolutions
#define RES_160x120            0
#define RES_240x176            1
#define RES_INVALID1           2
#define RES_INVALID2           3
#define RES_320x240            4 
#define RES_400x300            5
#define RES_640x480            6
#define RES_800x600            7
#define RES_1024x768           8
#define RES_1280x1024          9
#define RES_1600x1200          10           
extern const char cameraResolutionName[11][10];

extern const char gpsStatusName[9][11]; */


struct __attribute__ ((packed)) cfg_boot_t {
    char        magic_number = 'b';
    uint8_t     version = 2;
    uint8_t     boot_bank = 1;
    uint8_t     size_cfg_boot = 50;
    uint8_t     size_config = 250;
    char        cfg_name[3][10] = { "empty", "empty", "empty" };
};

struct __attribute__ ((packed)) ccsds_hdr_t {
    uint8_t     apid_H:3;                // 0:5
    bool        sec_hdr:1;               // 0: 4
    bool        type:1;                  // 0:  3
    uint8_t     version:3;               // 0:   0
    uint8_t     apid_L;                  // 1
    uint8_t     seq_ctr_H:6;             // 2:2
    uint8_t     seq_flag:2;              // 2: 0
    uint8_t     seq_ctr_L;               // 3
    uint8_t     pkt_len_H;               // 4
    uint8_t     pkt_len_L;               // 5
}; 

struct __attribute__ ((packed)) ccsds_sec_hdr_t {
    uint32_t    seconds;               
    uint16_t    subseconds;
}; 

struct __attribute__ ((packed)) ccsds_t {
    ccsds_hdr_t ccsds_hdr;
    ccsds_sec_hdr_t ccsds_sec_hdr;
    byte        blob[PARAMETER_MAX_SIZE+6];   // sized for longest possible sts_esp32/sts_esp32cam packet
};

struct __attribute__ ((packed)) tc_packet_t {     // APID: 42 (2a) / 43 (2b) / 44 (2c)
    ccsds_hdr_t ccsds_hdr;
    uint8_t     cmd_id;
    char        parameter[PARAMETER_MAX_SIZE];
}; 

struct __attribute__ ((packed)) sts_packet_t {    // APID: 45 (2d) / 46 (2e) / 47 (2f)
    ccsds_hdr_t ccsds_hdr;
    ccsds_sec_hdr_t ccsds_sec_hdr;
    uint16_t    packet_ctr;
    uint8_t     type:3;                  // 5-7
    uint8_t     subsystem:5;             //  0-4
    char        message[PARAMETER_MAX_SIZE];
};

struct __attribute__ ((packed)) tm_esp32_t {     // APID: 48 (30)
    ccsds_hdr_t ccsds_hdr;
    ccsds_sec_hdr_t ccsds_sec_hdr;
    uint32_t    millis:24;
    uint16_t    packet_ctr;
    uint8_t     opsmode:2;               // 6-7
    uint8_t     state:2;                 //  4-5
    uint8_t     archive_fs:2;            //   2-3 
    uint8_t     ftp_fs:2;                //    0-1 
    uint8_t     error_ctr;
    uint8_t     warning_ctr;
    uint8_t     tc_exec_ctr;
    uint8_t     tc_fail_ctr;
    uint8_t     espnow_rx_pktrate;
    uint8_t     espnow_tx_pktrate;
    uint8_t     serial_rx_pktrate;
    uint8_t     serial_tx_pktrate;
    uint8_t     radio_rx_pktrate;
    uint8_t     radio_tx_pktrate;
    uint8_t     archive_pktrate;
    uint8_t     pressure_pktrate;
    uint8_t     motion_pktrate;
    uint8_t     gps_pktrate;  
    int8_t      wifi_rssi;             // TODO: implement
    int8_t      radio_rssi;            // TODO: implement
    uint16_t    mem_free;
    uint16_t    fs_free;
    uint32_t    sd_free;
    uint8_t     espnow_buffer_queue;
    uint8_t     serial_buffer_queue;
    uint8_t     radio_buffer_queue;
    uint8_t     archive_buffer_queue;
    uint8_t     buffer_size;
    char        archive_path[20];
    
    bool        espnow_rx_enabled:1;   // 7
    bool        espnow_tx_enabled:1;   //  6
    bool        serial_rx_enabled:1;   //   5
    bool        serial_tx_enabled:1;   //    4
    bool        radio_rx_enabled:1;    //     3
    bool        radio_tx_enabled:1;    //      2
    bool        wifi_ap_enabled:1;     //       1
    bool        wifi_sta_enabled:1;    //        0
    
    bool        ftp_enabled:1;         // 7
    bool        ota_enabled:1;         //  6 
    bool        fs_enabled:1;          //   5
    bool        sd_enabled:1;          //    4
    bool        archive_enabled:1;     //     3
    bool        pressure_enabled:1;    //      2
    bool        motion_enabled:1;      //       1
    bool        gps_enabled:1;         //        0
    
    bool        buzzer_enabled:1;      // 7
    bool        wifi_connected:1;      //  6       TODO: keep or not?
    bool        ftp_connected:1;       //   5      TODO: keep or not?
    bool        espnow_connected:1;    //    4     TODO: keep or not?
    bool        serial_connected:1;    //     3    TODO: keep or not?
    bool        radio_connected:1;     //      2   TODO: keep or not?
    bool        separation_sts:1;      //       1
    bool        time_set:1;            //        0
    
    bool        ftp_active:1;          // 7 
    bool        fs_active:1;           //  6
    bool        sd_active:1;           //   5
    bool        archive_active:1;      //    4
    bool        buzzer_active:1;       //     3
    bool        pressure_active:1;     //      2
    bool        motion_active:1;       //       1
    bool        gps_active:1;          //        0
    
    bool        espnow_rx_active:1;    // 7
    bool        espnow_tx_active:1;    //  6
    bool        serial_rx_active:1;    //   5
    bool        serial_tx_active:1;    //    4
    bool        radio_rx_active:1;     //     3
    bool        radio_tx_active:1;     //      2
    bool        wifi_active:1;         //       1  TODO: keep or not?
    bool        free_30:1;             //        0
};

struct __attribute__ ((packed)) tm_gps_t {       // APID: 49 (31)
    ccsds_hdr_t ccsds_hdr;
    ccsds_sec_hdr_t ccsds_sec_hdr;
    uint16_t    packet_ctr;
    uint8_t     status:4;                // 4-7
    uint8_t     satellites:4;            //  0-3   *GGA
    uint8_t     hours;                   //        RMC,*GGA,ZDA
    uint8_t     minutes;                 //        RMC,*GGA,ZDA
    uint8_t     seconds;                 //        RMC,*GGA,ZDA
    uint8_t     centiseconds;            //        *GST
    int32_t     latitude;                //        RMC,*GGA,GLL
    int32_t     longitude;               //        RMC,*GGA,GLL
    int32_t     altitude;                // cm     *GGA
    int32_t     latitude_zero;
    int32_t     longitude_zero;
    int32_t     altitude_zero;           // cm  
    int16_t     x;                       // cm
    int16_t     y;                       // cm
    int16_t     z;                       // cm
    int16_t     x_err;                   // cm     *GST
    int16_t     y_err;                   // cm     *GST
    int16_t     z_err;                   // cm     *GST
    int32_t     v_north;                 // cm/s   VTG
    int32_t     v_east;                  // cm/s   VTG
    int32_t     v_down;                  // cm/s   PUBX_00
    uint16_t    milli_hdop;              //        *GSA
    uint16_t    milli_vdop;              //        *GSA
    uint16_t    milli_pdop;              //        *GSA
    bool        time_valid:1;            // 7
    bool        location_valid:1;        //  6
    bool        altitude_valid:1;        //   5
    bool        speed_valid:1;           //    4
    bool        hdop_valid:1;            //     3
    bool        vdop_valid:1;            //      2
    bool        pdop_valid:1;            //       1
    bool        error_valid:1;           //        0
    bool        offset_valid:1;          // 7
    bool        free_16:1;               //  6 - free to assign
    bool        free_15:1;               //   5 - free to assign
    bool        free_14:1;               //    4 - free to assign
    bool        free_13:1;               //     3 - free to assign
    bool        free_12:1;               //      2 - free to assign
    bool        free_11:1;               //       1 - free to assign
    bool        free_10:1;               //        0 - free to assign
};

struct __attribute__ ((packed)) tm_motion_t {    // APID: 50 (32)
    ccsds_hdr_t ccsds_hdr;
    ccsds_sec_hdr_t ccsds_sec_hdr;
    uint16_t    packet_ctr;
    int16_t     accel_x;                 // cm/s2
    int16_t     accel_y;                 // cm/s2
    int16_t     accel_z;                 // cm/s2
    int16_t     gyro_x;                  // cdeg/s
    int16_t     gyro_y;                  // cdeg/s
    int16_t     gyro_z;                  // cdeg/s
    int16_t     magn_x;                  // uT
    int16_t     magn_y;                  // uT
    int16_t     magn_z;                  // uT
    int16_t     tilt;                    // cdeg
    uint16_t    g;                       // mG
    int16_t     a;                       // cm/s2
    int16_t     rpm;                     // crpm
    uint8_t     accel_range:2;           //  6-7
    uint8_t     gyro_range:2;            //   4-5
    bool        accel_valid:1;           //    3
    bool        gyro_valid:1;            //     2
    bool        free_01:1;               //      1 - free to assign
    bool        free_00:1;               //       0 - free to assign
}; 

struct __attribute__ ((packed)) tm_pressure_t {  // APID: 51 (33)
    ccsds_hdr_t ccsds_hdr;
    ccsds_sec_hdr_t ccsds_sec_hdr;
    uint16_t    packet_ctr;
    uint32_t    pressure;                // Pa
    uint32_t    zero_level_pressure;     // Pa
    int16_t     height;                  // cm
    int16_t     velocity_v;              // cm/s
    int16_t     temperature;             // cdegC
    bool        height_valid:1;          // 7
    bool        free_06:1;               //  6 - free to assign
    bool        free_05:1;               //   5 - free to assign
    bool        free_04:1;               //    4 - free to assign
    bool        free_03:1;               //     3 - free to assign
    bool        free_02:1;               //      2 - free to assign
    bool        free_01:1;               //       1 - free to assign
    bool        free_00:1;               //        0 - free to assign
}; 

struct __attribute__ ((packed)) tm_radio_t {     // APID: 52 (34)
    ccsds_hdr_t ccsds_hdr;
    ccsds_sec_hdr_t ccsds_sec_hdr;
    uint16_t    packet_ctr;
    uint8_t     opsmode:2;               // 6
    uint8_t     state:2;                 //  4
    bool        pressure_active:1;       //   3
    bool        motion_active:1;         //    2
    bool        gps_active:1;            //     1
    bool        camera_active:1;         //      0
    uint8_t     error_ctr;
    uint8_t     warning_ctr;
    uint8_t     pressure_height;         // m               (0 - 255 m)
    int8_t      pressure_velocity_v;     // m/s             (-125 - 126 m/s)
    int8_t      temperature;             // degC            (-125 - 126 degC)
    uint8_t     motion_tilt;             // deg             (0 - 180 deg)
    uint8_t     motion_g;                // G / 10          (0 - 25.5 G)  
    int8_t      motion_a;                // m/s2            (-125 - 126 m/s2)
    int8_t      motion_rpm;              //                 (-125 - 126 rpm)  
    uint8_t     gps_satellites:4;        //   4-7
    bool        esp32_buffer_active:1;   //    3
    bool        esp32cam_buffer_active:1; //    2
    bool        esp32cam_sd_image_enabled:1; //  1
    bool        esp32cam_wifi_image_enabled:1; // 0
    int8_t      gps_velocity_v;          // m/s             (-125 - 126 m/s)
    uint8_t     gps_velocity;            // m/s             (0 - 255 m/s)
    uint8_t     gps_height;              // m               (0 - 255 m)
    uint16_t    camera_image_ctr;  
    bool        delete_030:1;        // 7
    bool        esp32_espnow_connected:1;          //  6 
    bool        delete_031:1;    //   5        
    bool        esp32_warn_espnow_connloss:1;      //    4       
    bool        delete_032:1;     //     3
    bool        delete_005:1;      //      2
    bool        esp32_err_fs_dataloss:1;         //       1  
    bool        separation_sts:1;                //        0
    bool        delete_033:1;     // 7
    bool        esp32cam_espnow_connected:1;       //  6
    bool        delete_034:1; //   5        
    bool        esp32cam_warn_espnow_connloss:1;   //    4       
    bool        delete_035:1;  //     3
    bool        delete_006:1;   //      2
    bool        esp32cam_err_fs_dataloss:1;      //       1
    bool        esp32cam_err_sd_dataloss:1;      //        0
}; 

struct __attribute__ ((packed)) tm_esp32cam_t {  // APID: 53 (35)
    ccsds_hdr_t ccsds_hdr;
    ccsds_sec_hdr_t ccsds_sec_hdr;
    uint32_t    millis:24;
    uint16_t    packet_ctr;
    uint8_t     opsmode:2;               // 6-7
    uint8_t     free:2;                  //  4-5
    uint8_t     archive_fs:2;            //   2-3 
    uint8_t     ftp_fs:2;                //    0-1 
    uint8_t     error_ctr;
    uint8_t     warning_ctr;
    uint8_t     tc_exec_ctr;
    uint8_t     tc_fail_ctr;
    uint8_t     espnow_rx_pktrate;
    uint8_t     espnow_tx_pktrate;
    uint8_t     serial_rx_pktrate;
    uint8_t     serial_tx_pktrate;
    uint8_t     archive_pktrate;
    uint8_t     camera_pktrate;
    uint8_t     sd_image_rate;
    int8_t      wifi_rssi;
    uint16_t    mem_free;
    uint16_t    fs_free;
    uint16_t    sd_free;
    char        archive_path[20];
    
    bool        wifi_ap_enabled:1;     // 7
    bool        wifi_sta_enabled:1;    //  6
    bool        espnow_rx_enabled:1;   //   5
    bool        espnow_tx_enabled:1;   //    4
    bool        serial_rx_enabled:1;   //     3
    bool        serial_tx_enabled:1;   //      2
    bool        radio_rx_enabled:1;    //       1
    bool        radio_tx_enabled:1;    //        0
    
    bool        ftp_enabled:1;         // 7
    bool        ota_enabled:1;         //  6 
    bool        archive_enabled:1;     //   5
    bool        fs_enabled:1;          //    4
    bool        sd_enabled:1;          //     3
    bool        camera_enabled:1;      //      2
    bool        wifi_rtsp_enabled:1;   //       1
    bool        free_00:1;             //        0
    
    bool        wifi_connected:1;      // 7 
    bool        ftp_connected:1;       //  6 
    bool        espnow_connected:1;    //   5 
    bool        serial_connected:1;    //    4
    bool        radio_connected:1;     //     3
    bool        free_12:1;             //      2
    bool        time_set:1;            //       1
    bool        free_10:1;             //        0
    
    bool        wifi_active:1;         // 7 
    bool        ftp_active:1;          //  6 
    bool        archive_active:1;      //   5
    bool        fs_active:1;           //    4
    bool        sd_active:1;           //     3
    bool        camera_active:1;       //      2
    bool        wifi_rtsp_active:1;    //       1
    bool        free_20:1;             //        0
    
    bool        espnow_rx_active:1;    // 7
    bool        espnow_tx_active:1;    //  6
    bool        serial_rx_active:1;    //   5
    bool        serial_tx_active:1;    //    4
    bool        free_33:1;             //     3
    bool        free_32:1;             //      2
    bool        free_31:1;             //       1
    bool        free_30:1;             //        0
};

struct __attribute__ ((packed)) tm_camera_t {    // APID: 54 (36)
    ccsds_hdr_t ccsds_hdr;
    ccsds_sec_hdr_t ccsds_sec_hdr;
    uint16_t    packet_ctr;
    uint8_t     camera_mode:2;           // 6-7
    uint8_t     resolution:4;            //  2-5
    bool        auto_res:1;              //   1
    bool        free_00:1;               //    0 - free to assign
    uint32_t    filesize:24; 
    uint8_t     wifi_ms; 
    uint8_t     sd_ms;
    uint8_t     exposure_ms;
    char        filename[36]; 
};

struct __attribute__ ((packed)) tm_gndctrl_t {   // APID: 55 (37)
    ccsds_hdr_t ccsds_hdr;
    ccsds_sec_hdr_t ccsds_sec_hdr;
    uint32_t    millis:24;
    uint16_t    packet_ctr;
    uint8_t     opsmode:2;             // 6-7
    uint8_t     free:2;                //  4-5
    uint8_t     archive_fs:2;          //   2-3 
    uint8_t     ftp_fs:2;              //    0-1 
    uint8_t     error_ctr;
    uint8_t     warning_ctr;
    uint8_t     tc_exec_ctr;
    uint8_t     tc_fail_ctr;
    uint8_t     espnow_rx_pktrate;
    uint8_t     espnow_tx_pktrate;
    uint8_t     serial_rx_pktrate;
    uint8_t     serial_tx_pktrate;
    uint8_t     radio_rx_pktrate;
    uint8_t     radio_tx_pktrate;
    uint8_t     archive_pktrate;
    int8_t      wifi_rssi;             // TODO: implement (once IDF5 is used)
    int8_t      radio_rssi;            // TODO: implement
    uint16_t    mem_free;
    uint16_t    fs_free;
    uint32_t    sd_free;
    uint8_t     espnow_buffer_queue;
    uint8_t     serial_buffer_queue;
    uint8_t     radio_buffer_queue;
    uint8_t     archive_buffer_queue;
    uint8_t     buffer_size;
    char        archive_path[20];
    bool        espnow_rx_enabled:1;   // 7
    bool        espnow_tx_enabled:1;   //  6
    bool        serial_rx_enabled:1;   //   5
    bool        serial_tx_enabled:1;   //    4
    bool        radio_rx_enabled:1;    //     3
    bool        radio_tx_enabled:1;    //      2
    bool        wifi_ap_enabled:1;     //       1
    bool        wifi_sta_enabled:1;    //        0
    
    bool        ftp_enabled:1;         // 7
    bool        ota_enabled:1;         //  6 
    bool        fs_enabled:1;          //   5
    bool        sd_enabled:1;          //    4
    bool        archive_enabled:1;     //     3
    bool        free_02:1;             //      2
    bool        free_01:1;             //       1
    bool        free_00:1;             //        0
    
    bool        buzzer_enabled:1;      // 7
    bool        wifi_connected:1;      //  6       TODO: keep or not?
    bool        ftp_connected:1;       //   5      TODO: keep or not?
    bool        espnow_connected:1;    //    4     TODO: keep or not?
    bool        serial_connected:1;    //     3    TODO: keep or not?
    bool        radio_connected:1;     //      2   TODO: keep or not?
    bool        free_11:1;             //       1
    bool        time_set:1;            //        0	
    
    bool        ftp_active:1;          // 7 
    bool        fs_active:1;           //  6
    bool        sd_active:1;           //   5
    bool        archive_active:1;      //    4
    bool        buzzer_active:1;       //     3
    bool        free_22:1;             //      2
    bool        free_21:1;             //       1
    bool        free_20:1;             //        0
    
    bool        espnow_rx_active:1;    // 7
    bool        espnow_tx_active:1;    //  6
    bool        serial_rx_active:1;    //   5
    bool        serial_tx_active:1;    //    4
    bool        radio_rx_active:1;     //     3
    bool        radio_tx_active:1;     //      2
    bool        wifi_active:1;         //       1  TODO: keep or not?
    bool        free_30:1;             //        0
};

struct __attribute__ ((packed)) tmr_esp32_t {  // APID: 56 (38)
    ccsds_hdr_t ccsds_hdr;
    ccsds_sec_hdr_t ccsds_sec_hdr;
    uint32_t    millis:24;
    uint16_t    packet_ctr;
    uint16_t    radio_duration;
    uint16_t    pressure_duration;
    uint16_t    motion_duration;
    uint16_t    gps_duration;
    uint16_t    esp32cam_duration;
    uint16_t    ota_duration;
    uint16_t    ftp_duration;
    uint16_t    wifi_duration;
    uint16_t    tc_duration;
    uint16_t    idle_duration;
    uint16_t    publish_fs_duration;
    uint16_t    delete_011;
    uint16_t    publish_espnow_duration;
};

struct __attribute__ ((packed)) tmr_esp32cam_t { // APID: 57 (39)  // TODO: fine-tune packet
    ccsds_hdr_t ccsds_hdr;
    ccsds_sec_hdr_t ccsds_sec_hdr;
    uint32_t    millis:24;
    uint16_t    packet_ctr;
    uint16_t    camera_duration;
    uint16_t    tc_duration;
    uint16_t    sd_duration;
    uint16_t    ftp_duration;
    uint16_t    wifi_duration;
    uint16_t    idle_duration;
    uint16_t    publish_sd_duration;
    uint16_t    publish_fs_duration;
    uint16_t    delete_012;
    uint16_t    publish_espnow_duration;
    uint16_t    ota_duration;
};

struct __attribute__ ((packed)) tmr_gndctrl_t {  // TODO: delete
    ccsds_hdr_t ccsds_hdr;
    ccsds_sec_hdr_t ccsds_sec_hdr;
    uint32_t    millis:24;
    uint16_t    packet_ctr;
    uint16_t    radio_duration;
    uint16_t    pressure_duration;
    uint16_t    motion_duration;
    uint16_t    gps_duration;
    uint16_t    esp32cam_duration;
    uint16_t    ota_duration;
    uint16_t    ftp_duration;
    uint16_t    wifi_duration;
    uint16_t    tc_duration;
    uint16_t    idle_duration;
    uint16_t    publish_fs_duration;
    uint16_t    publish_yamcs_duration;
    uint16_t    publish_espnow_duration;
};

struct __attribute__ ((packed)) cfg_packet_t {  // APID: 59/60/61 (3b/3d/3e)
    ccsds_hdr_t ccsds_hdr;
    ccsds_sec_hdr_t ccsds_sec_hdr;
    char        magic_number;
    uint8_t     version = 3;	
    uint8_t     wifi_channel:4;       // 7-4
    bool        espnow_longrange:1;   //  3
    bool        espnow_broadcast:1;   //   2
    uint8_t     target_opsmode:2;     //    1-0
    uint8_t     archive_fs:2;         // 7-6
    uint8_t     ftp_fs:2;             //  5-4
    uint8_t     free_00:4;            //   0-3
    byte        wifi_my_ip[4];        // TODO: assign fixed IP to each party NOPE?
    byte        my_mac[6];
    byte        peer_mac[6];
    uint32_t    serial_baud;
    uint32_t    radio_baud;
    char        rocket_name[10];
    char        password[10];
    
    bool        espnow_rx_enable:1;   // 7   
    bool        espnow_tx_enable:1;   //  6  
    bool        serial_rx_enable:1;   //   5      
    bool        serial_tx_enable:1;   //    4  
    bool        radio_rx_enable:1;    //     3     
    bool        radio_tx_enable:1;    //      2 
    bool        wifi_ap_enable:1;     //       1
    bool        wifi_sta_enable:1;    //        0 
    
    bool        ftp_enable:1;         // 7
    bool        ota_enable:1;         //  6
    bool        archive_enable:1;     //   5
    bool        fs_enable:1;          //    4
    bool        sd_enable:1;          //     3
    bool        flush_fs_enable:1;    //      2
    bool        free_11:1;            //       1
    bool        free_10:1;            //        0
    
    bool        espnow_buffer_enable:1;  // 7
    bool        serial_buffer_enable:1;  //  6
    bool        radio_buffer_enable:1;   //   5
    bool        archive_buffer_enable:1; //    4
    bool        free_23:1;               //     3
    bool        free_22:1;               //      2
    bool        free_21:1;               //       1
    bool        free_20:1;               //        0

    bool        dip_set1:1;           // 7
    bool        dip_set2:1;           //  6
    bool        dip_set3:1;           //   5
    bool        dip_set4:1;           //    4
    bool        buzzer_enable:1;      //     3
    bool        pressure_enable:1;    //      2       
    bool        motion_enable:1;      //       1
    bool        gps_enable:1;         //        0
    
    uint32_t    routing_espnow __attribute__((aligned(4)));
    uint32_t    routing_serial __attribute__((aligned(4)));
    uint32_t    routing_radio __attribute__((aligned(4)));
    uint32_t    routing_archive __attribute__((aligned(4)));
    
    cfg_boot_t cfg_boot; __attribute__((aligned(sizeof(cfg_boot_t))));

    uint8_t     pressure_tm_rate;      // Hz (up to 157 Hz, highest resolution up to 23 Hz); reached 176 Hz on ESP8266
    uint8_t     motion_tm_rate;        // Hz (up to 400) - 255 is highest set value
    uint8_t     gps_tm_rate;           // Hz (valid: 1,5,10,16)
    int16_t     mpu_accel_sensitivity; // �m/s2 per LSB 
    int16_t     mpu_accel_offset_x;    // y_sensor values // TODO: realign XYZ
    int16_t     mpu_accel_offset_y;    // z_sensor values
    int16_t     mpu_accel_offset_z;    // x_sensor values
    int16_t     mpu_gyro_offset_x;     // y_sensor values
    int16_t     mpu_gyro_offset_y;     // z_sensor values
    int16_t     mpu_gyro_offset_z;     // x_sensor values 
};

struct var_t {
    uint32_t    next_second;
    uint32_t    next_pressure_time;
    uint32_t    next_motion_time;
    uint32_t    next_gps_time;
    uint32_t    next_camera_time;
    uint32_t    next_tx_time;
    uint16_t    pressure_interval;
    uint16_t    motion_interval;
    uint16_t    gps_interval;
    uint16_t    camera_interval;
    uint16_t    delta_millis;
    uint8_t     boot_bank;
    uint8_t     espnow_buffer_index;
    uint8_t     serial_buffer_index;
    uint8_t     radio_buffer_index;
    uint8_t     archive_buffer_index;
    bool        do_pressure:1;
    bool        do_motion:1;
    bool        do_gps:1; 
    bool        do_camera:1;
    bool        do_tx:1;
    File        archive_file;
}; 

struct __attribute__ ((packed)) packet_properties_t {
    uint8_t     PID;
    uint16_t    APID;
    ccsds_t*    ccsds_ptr;
    uint16_t    size;
    char        name[16];
    uint8_t     source;
    uint8_t     dest;
    bool        type:1;
    uint8_t     subtype:3;
    bool        routing_espnow:1;
    bool        routing_serial:1;
    bool        routing_radio:1;
    bool        routing_archive:1;
};

struct __attribute__ ((packed)) name_t {
    uint8_t     ID;
    char        name[32];
};

struct __attribute__ ((packed)) buffer_t { 
    ccsds_t*    ccsds_ptr;
    uint8_t     packet_source;
    uint8_t     packet_comms;
};

struct __attribute__ ((packed)) index_t { 
    uint32_t    packet_offset;
    uint16_t    packet_len;
};

extern tc_packet_t         tc_esp32;
extern tc_packet_t         tc_esp32cam;
extern tc_packet_t         tc_gndctrl;
extern sts_packet_t        sts_esp32;
extern sts_packet_t        sts_esp32cam;
extern sts_packet_t        sts_gndctrl;
extern tm_esp32_t          tm_esp32;
extern tm_gps_t            tm_gps;
extern tm_motion_t         tm_motion;
extern tm_pressure_t       tm_pressure;
extern tm_radio_t          tm_radio;
extern tm_esp32cam_t       tm_esp32cam;
extern tm_camera_t         tm_camera;
extern tm_gndctrl_t        tm_gndctrl;
extern tmr_esp32_t         tmr_esp32;
extern tmr_esp32cam_t      tmr_esp32cam;
extern tmr_gndctrl_t       tmr_gndctrl;
extern cfg_packet_t        cfg_esp32;
extern cfg_packet_t        cfg_esp32cam;
extern cfg_packet_t        cfg_gndctrl;
extern var_t               var;
extern bool                default_routing_espnow[];
extern bool                default_routing_serial[];
extern bool                default_routing_radio[];
extern bool                default_routing_archive[];

#ifdef PLATFORM_ESP32
extern tm_esp32_t*         tm_this;
extern tm_esp32cam_t*      tm_other;
extern sts_packet_t*       sts_this;
extern sts_packet_t*       sts_other;
extern tc_packet_t*        tc_this;
extern tc_packet_t*        tc_other;
extern cfg_packet_t*    cfg_this;
#endif
#ifdef PLATFORM_ESP32CAM
extern tm_esp32cam_t*      tm_this;
extern tm_esp32_t*         tm_other;
extern sts_packet_t*       sts_this;
extern sts_packet_t*       sts_other;
extern tc_packet_t*        tc_this;
extern tc_packet_t*        tc_other;
extern cfg_packet_t*    cfg_this;
#endif
#ifdef PLATFORM_GNDCTRL
extern tm_gndctrl_t*       tm_this;
extern tm_esp32_t*         tm_other;
extern sts_packet_t*       sts_this;
extern sts_packet_t*       sts_other;
extern tc_packet_t*        tc_this;
extern tc_packet_t*        tc_other;
extern cfg_packet_t*    cfg_this;
#endif

extern char buffer[BUFFER_MAX_SIZE];
//extern packet_properties_t packet[];
extern name_t subsystem[];
//extern name_t command[];
//extern name_t event[];
//extern name_t comms[];
//extern name_t opsmode[];
//extern name_t filesystem[];

// Configuration Functionality
extern void init_config ();
extern bool load_boot_config ();
extern bool load_config (const uint8_t bank);
extern bool set_opsmode (const uint8_t mode);

// WiFi Functionality
extern void setup_wifi ();
extern bool setup_wifi_ap ();
extern bool setup_wifi_sta ();
extern void enable_wifi_services ();
extern void disable_wifi_services ();

// ESP-NOW Functionality
extern bool setup_espnow ();

// SerialTransfer Functionality
extern bool setup_serialtransfer (Stream &serialport);
extern bool check_serialtransfer_rx ();

// File System Functionality
extern bool setup_fs ();
extern bool setup_sd ();
extern uint16_t fs_free ();
extern uint32_t sd_free ();

// Archive Functionality
extern bool setup_archive ();

// TM/TC Functionality
extern void process_rx_queue ();
extern void process_tx_queue ();
extern void publish_packet (ccsds_t* ccsds_ptr);
extern void publish_event (uint8_t PID, uint8_t subsystem, uint8_t event_type, const char* event_message);

// CCSDS Functionality
extern void init_ccsds ();

// OTA Functionality
extern void setup_ota ();

// Support Functionality
extern String get_hex_str (byte* blob, uint16_t length);


#endif // _FLI3D_H_
