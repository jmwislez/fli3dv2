
/*
 * Fli3d - separation detection functionality
 */

 #include <Arduino.h>
 #include <fli3dv2.h>
 #include "fli3dv2_esp32.h"

void ICACHE_RAM_ATTR separation_detectChange();
extern bool separation_sts_changed;

void setup_separation() {
  pinMode(SEP_STS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SEP_STS_PIN), separation_detectChange, CHANGE);
  tm_esp32.separation_sts = digitalRead (SEP_STS_PIN);
  sprintf(buffer, "Separation detection initialized (%s)", tm_esp32.separation_sts?"separated":"mated");
  publish_event (STS_ESP32, SS_SEPARATION, EVENT_INIT, buffer);
}

void separation_publish() {
  if (tm_esp32.separation_sts) {
    publish_event (STS_ESP32, SS_SEPARATION, EVENT_INFO, "Fli3d separated from rocket");   
  }
  else {
    publish_event (STS_ESP32, SS_SEPARATION, EVENT_INFO, "Fli3d mated to rocket");
  }
}

void separation_detectChange() {
  tm_esp32.separation_sts = digitalRead (SEP_STS_PIN);
  separation_sts_changed = true;
}