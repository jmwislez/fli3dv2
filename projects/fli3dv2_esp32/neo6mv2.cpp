// Support for NEO6MV2 GPS module

#include <NMEAGPS.h> 
#include <ublox/ubxGPS.h>
#include "fli3dv2.h"

NMEAGPS gps;
gps_fix fix;

const int32_t latitude_deg_to_m = M_PI*12742000/360;
const int32_t longitude_deg_to_m = M_PI*12742000*cos(51*M_PI/180)/360;

void sendUBX( const char *progmemBytes, size_t len ) {
    Serial1.write( 0xB5 ); // SYNC1
    Serial1.write( 0x62 ); // SYNC2
    uint8_t a = 0, b = 0;
    while (len-- > 0) {
        uint8_t c = pgm_read_byte( progmemBytes++ );
        a += c;
        b += a;
        Serial1.write( c );
    }
    Serial1.write( a ); // CHECKSUM A
    Serial1.write( b ); // CHECKSUM B
    } 

bool check_gps_communication (uint32_t baudrate) {
    char gps_char;
    uint8_t gps_good = 0;
    uint8_t gps_bad = 0;
    uint16_t ctr = 0;
    uint32_t start_probe_millis, now_millis;

    start_probe_millis = millis();
    now_millis = millis();
    while (now_millis - start_probe_millis < 5000) {
        while (Serial1.available() and gps_good < 255 and gps_bad < 255) {
            gps_char = Serial1.read();
            if (ctr++ > 200) {
                if ((gps_char >= ' ' and gps_char <= 'Z') or (uint8_t)gps_char == 10 or (uint8_t)gps_char == 13) {
                    Serial.print(gps_char);
                    gps_good++;
                }
                else {
                    Serial.print("[");
                    Serial.print((uint8_t)gps_char);
                    Serial.print("]");
                    gps_bad++;
                }
            }
        }
        now_millis = millis();
    } // timeout
    Serial.println();
    Serial.print("GPS ratio ");
    Serial.print(gps_good);
    Serial.print(":");
    Serial.println(gps_bad);   
    if (gps_good == 255) {
        // proper data flow received
        return true;
    }
    else if (gps_good == 0 and gps_bad == 0) {
        // timeout
        sprintf(buffer, "Timeout in trying to connect to GPS at %u baud", baudrate);
        publish_event (STS_ESP32, SS_GPS, EVENT_WARNING, buffer);
        return false;
    }
    else {
        return false;
    }
}

void set_neo6mv2_samplerate (uint8_t rate) {
    const char ubxRate1Hz[] PROGMEM =  { 0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00 };
    const char ubxRate5Hz[] PROGMEM =  { 0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00 };
    const char ubxRate10Hz[] PROGMEM = { 0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00 };
    const char ubxRate16Hz[] PROGMEM = { 0x06,0x08,0x06,0x00,0x3E,0x00,0x01,0x00,0x01,0x00 };  
    switch (rate) {
        case 1:  sendUBX( ubxRate1Hz, sizeof(ubxRate1Hz) ); break;
        case 5:  sendUBX( ubxRate5Hz, sizeof(ubxRate5Hz) ); break;
        case 10: sendUBX( ubxRate10Hz, sizeof(ubxRate10Hz) ); break;
        case 16: sendUBX( ubxRate16Hz, sizeof(ubxRate16Hz) ); break;
        default: sprintf (buffer, "Invalid sample rate request for GPS (%d Hz), setting to 1 Hz", cfg_esp32.gps_tm_rate);
                publish_event (STS_ESP32, SS_GPS, EVENT_WARNING, buffer);
                cfg_esp32.gps_tm_rate = 1;
                sendUBX( ubxRate1Hz, sizeof(ubxRate1Hz) );
                break;
    }
}

bool set_neo6mv2_baudrate (uint32_t GPSBaud) {
    const char baud4800  [] PROGMEM = "PUBX,41,1,3,3,4800,0";
    const char baud9600  [] PROGMEM = "PUBX,41,1,3,3,9600,0";
    const char baud19200 [] PROGMEM = "PUBX,41,1,3,3,19200,0";
    const char baud38400 [] PROGMEM = "PUBX,41,1,3,3,38400,0";
    const char baud57600 [] PROGMEM = "PUBX,41,1,3,3,57600,0";
    const char baud115200[] PROGMEM = "PUBX,41,1,3,3,115200,0";
    
    switch (GPSBaud) {
        case 4800:    gps.send_P( &Serial1, (const __FlashStringHelper *) baud4800 ); break;
        case 9600:    gps.send_P( &Serial1, (const __FlashStringHelper *) baud9600 ); break;
        case 19200:   gps.send_P( &Serial1, (const __FlashStringHelper *) baud19200 ); break;
        case 38400:   gps.send_P( &Serial1, (const __FlashStringHelper *) baud38400 ); break;
        case 57600:   gps.send_P( &Serial1, (const __FlashStringHelper *) baud57600 ); break;
        case 115200:  gps.send_P( &Serial1, (const __FlashStringHelper *) baud115200 ); break;
        default:      sprintf (buffer, "Invalid baud rate request for GPS (%d baud)", GPSBaud);
                    publish_event (STS_ESP32, SS_GPS, EVENT_WARNING, buffer);
                    return false;
                    break;
    }
    // Set GPS rate to GPSBaud
    sprintf (buffer, "Setting GPS to %u baud", GPSBaud);
    publish_event (STS_ESP32, SS_GPS, EVENT_INIT, buffer);
    return true;
}

bool restart_neo6mv2() {
    const char ubxReset[] PROGMEM = { 0x06,0x04,0x00,0x00,0x01,0x00 };

    sendUBX (ubxReset, sizeof(ubxReset));
    delay (1000);
    Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial1.flush();
    return true;
}

bool setup_neo6mv2() {
    // TODO: should be able to set GPS baudrate higher
    if (check_gps_communication (9600)) {
        const char disableRMC[] PROGMEM = "PUBX,40,RMC,0,0,0,0,0,0";
        const char disableGLL[] PROGMEM = "PUBX,40,GLL,0,0,0,0,0,0";
        const char disableGSV[] PROGMEM = "PUBX,40,GSV,0,0,0,0,0,0";
        const char disableGST[] PROGMEM = "PUBX,40,GST,0,0,0,0,0,0";
        const char disableGSA[] PROGMEM = "PUBX,40,GSA,0,0,0,0,0,0";
        const char disableGGA[] PROGMEM = "PUBX,40,GGA,0,0,0,0,0,0";
        const char disableVTG[] PROGMEM = "PUBX,40,VTG,0,0,0,0,0,0";
        const char disableZDA[] PROGMEM = "PUBX,40,ZDA,0,0,0,0,0,0";
        const char enableRMC[] PROGMEM = "PUBX,40,RMC,1,1,1,1,1,0";
        const char enableGLL[] PROGMEM = "PUBX,40,GLL,1,1,1,1,1,0";
        const char enableGSV[] PROGMEM = "PUBX,40,GSV,1,1,1,1,1,0";
        const char enableGST[] PROGMEM = "PUBX,40,GST,1,1,1,1,1,0";
        const char enableGSA[] PROGMEM = "PUBX,40,GSA,1,1,1,1,1,0";
        const char enableGGA[] PROGMEM = "PUBX,40,GGA,1,1,1,1,1,0";
        const char enableVTG[] PROGMEM = "PUBX,40,VTG,1,1,1,1,1,0";
        const char enableZDA[] PROGMEM = "PUBX,40,ZDA,1,1,1,1,1,0";
        const char ubxAirborne[] PROGMEM = { 0x06,0x24,0x24,0x00,0x05,0x00,0x06,0x02,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
        const uint32_t COMMAND_DELAY = 250; 

        // Disable unnecessary GPS messages (needed: GSA, GST, GGA, VTG) // TODO: check what can go ...
        //gps.send_P( &Serial1, (const __FlashStringHelper *) disableRMC ); delay( COMMAND_DELAY ); // position, velocity and time
        //gps.send_P( &Serial1, (const __FlashStringHelper *) disableGLL ); delay( COMMAND_DELAY ); // position data: position fix, time of position fix, and status
        //gps.send_P( &Serial1, (const __FlashStringHelper *) disableGSV ); delay( COMMAND_DELAY ); // number of SVs in view, PRN, elevation, azimuth, and SNR
        //gps.send_P( &Serial1, (const __FlashStringHelper *) disableGST ); delay( COMMAND_DELAY ); // GPS Pseudorange Noise Statistics
        //gps.send_P( &Serial1, (const __FlashStringHelper *) disableGSA ); delay( COMMAND_DELAY ); // GPS DOP and active satellites
        //gps.send_P( &Serial1, (const __FlashStringHelper *) disableGGA ); delay( COMMAND_DELAY ); // time, position, and fix related data
        //gps.send_P( &Serial1, (const __FlashStringHelper *) disableVTG ); delay( COMMAND_DELAY ); // actual track made good and speed over ground
        //gps.send_P( &Serial1, (const __FlashStringHelper *) disableZDA ); delay( COMMAND_DELAY ); // UTC day, month, and year, and local time zone offset
        gps.send_P( &Serial1, (const __FlashStringHelper *) enableRMC ); delay( COMMAND_DELAY ); // position, velocity and time
        gps.send_P( &Serial1, (const __FlashStringHelper *) enableGLL ); delay( COMMAND_DELAY ); // position data: position fix, time of position fix, and status
        gps.send_P( &Serial1, (const __FlashStringHelper *) enableGSV ); delay( COMMAND_DELAY ); // number of SVs in view, PRN, elevation, azimuth, and SNR
        gps.send_P( &Serial1, (const __FlashStringHelper *) enableGSA ); delay( COMMAND_DELAY ); // GPS DOP and active satellites
        gps.send_P( &Serial1, (const __FlashStringHelper *) enableGST ); delay( COMMAND_DELAY ); // GPS Pseudorange Noise Statistics
        gps.send_P( &Serial1, (const __FlashStringHelper *) enableGGA ); delay( COMMAND_DELAY ); // time, position, and fix related data
        gps.send_P( &Serial1, (const __FlashStringHelper *) enableVTG ); delay( COMMAND_DELAY ); // actual track made good and speed over ground
        gps.send_P( &Serial1, (const __FlashStringHelper *) enableZDA ); delay( COMMAND_DELAY ); // UTC day, month, and year, and local time zone offset  
        
        // Set navigation engine settings: CFG-NAV5 dynModel:6 fixMode:2
        // !UBX CFG-NAV5 5 6 2 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
        sendUBX (ubxAirborne, sizeof(ubxAirborne));
        delay( COMMAND_DELAY );
        
        // Set GPS sample rate
        set_neo6mv2_samplerate (cfg_esp32.gps_tm_rate);

        sprintf(buffer, "GPS unit NEO6MV2 initialized on UART1 at 9600 baud");
        publish_event(STS_THIS, SS_THIS, EVENT_INIT, buffer);
        return true;
    }
    else {        
        sprintf(buffer, "GPS unit NEO6MV2 does not respond on UART1 at 9600 baud");
        publish_event(STS_THIS, SS_THIS, EVENT_ERROR, buffer);
        return false;
    }
}

bool acquire_neo6mv2() {

    while (gps.available(Serial1)) {
        tm_esp32.gps_active = true;
        fix = gps.read();
        tm_gps.status = fix.status;
        tm_gps.satellites = fix.satellites;
        if ((tm_gps.time_valid = fix.valid.time)) {
            tm_gps.hours = fix.dateTime.hours;
            tm_gps.minutes = fix.dateTime.minutes;
            tm_gps.seconds = fix.dateTime.seconds;
            tm_gps.centiseconds = fix.dateTime_cs;
            if (tm_this->time_set == false and fix.valid.date) {
                setTime(tm_gps.hours, tm_gps.minutes, tm_gps.seconds, fix.dateTime.day, fix.dateTime.month, fix.dateTime.year);
                sprintf(buffer, "Time set through GPS: %04u-%02u-%02u %02u:%02u:%02u", year(), month(), day(), hour(), minute(), second());
                publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
                tm_this->time_set = true;
                if (tm_this->fs_enabled) {
                    //create_today_dir (FS_LITTLEFS);
                }
            }
        }
        if ((tm_gps.location_valid = fix.valid.location)) {
            tm_gps.latitude = fix.latitudeL();
            tm_gps.longitude = fix.longitudeL();
        }
        if ((tm_gps.altitude_valid = fix.valid.altitude)) {
            tm_gps.altitude = fix.altitude_cm();
        }
        if (tm_gps.offset_valid) {
            tm_gps.x = (int16_t)((tm_gps.latitude - tm_gps.latitude_zero)*latitude_deg_to_m)/10000; //cm
            tm_gps.y = (int16_t)((tm_gps.longitude - tm_gps.longitude_zero)*longitude_deg_to_m)/10000; //cm
            tm_gps.z = (int16_t)(tm_gps.altitude - tm_gps.altitude_zero);
        }      
        if ((tm_gps.speed_valid = fix.valid.velned)) {
            tm_gps.v_north = fix.velocity_north; // cm/s
            tm_gps.v_east = fix.velocity_east;   // cm/s
            tm_gps.v_down = fix.velocity_down;   // cm/s
        }
        if ((tm_gps.hdop_valid = fix.valid.hdop)) { 
            tm_gps.milli_hdop = fix.hdop; 
        } 
        if ((tm_gps.vdop_valid = fix.valid.vdop)) { 
            tm_gps.milli_vdop = fix.vdop; 
        } 
        if ((tm_gps.pdop_valid = fix.valid.pdop)) { 
            tm_gps.milli_pdop = fix.pdop; 
        } 
        if ((tm_gps.error_valid = (fix.valid.lat_err and fix.valid.lon_err and fix.valid.alt_err))) {
            tm_gps.x_err = fix.lat_err_cm;
            tm_gps.y_err = fix.lon_err_cm;
            tm_gps.z_err = fix.alt_err_cm;
        } 
        return true;
    }
    return false;
}

void zero_gps() {
    tm_gps.latitude_zero = tm_gps.latitude;
    tm_gps.longitude_zero = tm_gps.longitude;
    tm_gps.altitude_zero = tm_gps.altitude;
    tm_gps.offset_valid = true;
    sprintf(buffer, "GPS zero level position set");
    publish_event(STS_THIS, SS_GPS, EVENT_INIT, buffer);
}