/*
 * Fli3d - power monitoring functionality
 */

 #include <Arduino.h>
 #include <fli3dv2.h>
 #include "fli3dv2_esp32.h"
 
void setup_power()
{
  analogSetPinAttenuation(BAT_V_PIN, ADC_11db);
  sprintf (buffer, "Battery monitoring initialized (%lu mV)", 2*analogReadMilliVolts(BAT_V_PIN)); 
  publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
}

