/*
 * Fli3d - GPIO functionality
 */

 #include <Arduino.h>
 #include <fli3dv2.h>
 #include "fli3dv2_esp32.h"
 
void setup_gpio()
{
    pinMode(SET1_PIN, INPUT_PULLUP);
    pinMode(SET2_PIN, INPUT_PULLUP);
    pinMode(SET3_PIN, INPUT_PULLUP);
    pinMode(SET4_PIN, INPUT_PULLUP);
    cfg_this->dip_set1 = !digitalRead (SET1_PIN);
    cfg_this->dip_set2 = !digitalRead (SET2_PIN);
    cfg_this->dip_set3 = !digitalRead (SET3_PIN);
    cfg_this->dip_set4 = !digitalRead (SET4_PIN);
    sprintf (buffer, "DIP switch settings acquired (1:%d 2:%d 3:%d 4:%d)", cfg_this->dip_set1, cfg_this->dip_set2, cfg_this->dip_set3, cfg_this->dip_set4);
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
    cfg_this->buzzer_enable=cfg_this->dip_set1;
    if (cfg_this->dip_set1) {
        sprintf (buffer, "Enabled buzzer based on DIP switch 1 (on)");
    }
    else {
        sprintf (buffer, "Disabled buzzer based on DIP switch 1 (off)");
    }
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
    cfg_this->flush_fs_enable=cfg_this->dip_set2;
    if (cfg_this->dip_set2) {
        sprintf (buffer, "Enabled FS flushing at boot based on DIP switch 2 (on)");
    }
    else {
        sprintf (buffer, "Disabled FS flushing at boot based on DIP switch 2 (off)");
    }
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
    if (cfg_this->dip_set3) {
        sprintf (buffer, "Using boot configuration stored in EEPROM based on DIP switch 3 (on)");
    }
    else {
        sprintf (buffer, "Skipping stored boot configuration based on DIP switch 3 (off)");
        cfg_this->cfg_boot.boot_bank=255; // force to skip stored boot configuration
    }
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
}