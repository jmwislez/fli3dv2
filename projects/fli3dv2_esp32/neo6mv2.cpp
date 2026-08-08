// Support for NEO6MV2 GPS module

#include <TinyGPS++.h>
#include "fli3dv2.h"

TinyGPSPlus gps;

bool setup_neo6mv2() {
    if (!Serial1) {
        sprintf(buffer, "GPS unit NEO6MV2 does not respond on UART");
        publish_event(STS_THIS, SS_THIS, EVENT_ERROR, buffer);
        return false;
    }
    else {
        sprintf(buffer, "GPS unit NEO6MV2 initialized on UART");
        publish_event(STS_THIS, SS_THIS, EVENT_INIT, buffer);
        return true;
    }
}

bool acquire_gps() {
    while (Serial1.available()) {
        gps.encode(Serial1.read());
    }
    if (gps.location.isUpdated()) {
        tm_gps.location_valid = gps.location.isValid();
        tm_gps.latitude = int32_t(gps.location.lat()*10000000); // convert deg to microdegrees
        tm_gps.longitude = int32_t(gps.location.lng()*10000000); // convert deg to microdegrees
        tm_gps.altitude = int32_t(gps.altitude.value()); // cm
        tm_gps.altitude_valid = gps.altitude.isValid();
        tm_gps.time_valid = gps.time.isValid();
        tm_gps.hours = gps.time.hour();
        tm_gps.minutes = gps.time.minute();
        tm_gps.seconds = gps.time.second();
        tm_gps.centiseconds = gps.time.centisecond();


        tm_gps.satellites = gps.satellites.value();
        tm_gps.milli_hdop = gps.hdop.value();
        return true;
    }
    else {
        return false;
    }
}