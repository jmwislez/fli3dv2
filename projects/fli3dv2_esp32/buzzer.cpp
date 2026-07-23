/*
 * Fli3d - buzzer functionality
 */

 #include <Arduino.h>
 #include <fli3dv2.h>
 #include "fli3dv2_esp32.h"
 
void setup_buzzer() 
{
    if (cfg_esp32.buzzer_enable) {
        pinMode(BZ_PIN, OUTPUT);
        digitalWrite(BZ_PIN, LOW);
        digitalWrite(BZ_PIN, HIGH);
        delay(100);
        digitalWrite(BZ_PIN, LOW);
        delay(100);
        digitalWrite(BZ_PIN, HIGH);
        delay(100);
        digitalWrite(BZ_PIN, LOW);
        sprintf (buffer, "Buzzer initialized on %s (beeped twice)", subsystem[SS_THIS].name); 
        publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
        tm_this->buzzer_enabled=true;
        tm_this->buzzer_active=true;
    }
}
