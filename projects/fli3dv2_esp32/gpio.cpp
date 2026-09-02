/*
 * Fli3d - GPIO functionality
 *
 * SET1: buzzer on/off (ON for flight)
 * SET2: FS flushing on/off (OFF for flight)
 * SET3: FS writable (on) / read only (off) (ON for flight)
 * SET4: camera acquisition on/off (ON for flight)
 * 
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

    // SET1
    cfg_this->buzzer_enable=cfg_this->dip_set1;
    if (cfg_this->dip_set1) {
        sprintf (buffer, "Enabled buzzer based on DIP switch 1 (on)");
    }
    else {
        sprintf (buffer, "Disabled buzzer based on DIP switch 1 (off)");
    }
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);

    // SET2
    cfg_this->flush_fs_enable=cfg_this->dip_set2;
    if (cfg_this->dip_set2) {
        sprintf (buffer, "Enabled FS flushing at boot based on DIP switch 2 (on)");
    }
    else {
        sprintf (buffer, "Disabled FS flushing at boot based on DIP switch 2 (off)");
    }
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);

    // SET3
    cfg_this->write_fs_enable=cfg_this->dip_set3; // TODO: IMPLEMENT!
    if (cfg_this->dip_set3) {
        sprintf (buffer, "Made FS writable based on DIP switch 3 (on)");
    }
    else {
        sprintf (buffer, "Made FS read-only based on DIP switch 3 (off)");
    }
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);
    
    // SET4
    cfg_this->camera_force_acquire=cfg_this->dip_set4; // TODO: IMPLEMENT!
    if (cfg_this->dip_set4) {
        sprintf (buffer, "Force camera acquisition based on DIP switch 4 (on)");
    }
    else {
        sprintf (buffer, "No camera acquisition forced by DIP switch 4 (off)");
    }
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer);

    // Other SET uses

    /* if (cfg_this->dip_set3) {
        sprintf (buffer, "Using boot configuration stored in EEPROM based on DIP switch 3 (on)");
    }
    else {
        sprintf (buffer, "Skipping stored boot configuration based on DIP switch 3 (off)");
        cfg_this->cfg_boot.boot_bank=255; // force to skip stored boot configuration
    }
    publish_event (STS_THIS, SS_THIS, EVENT_INIT, buffer); */

}