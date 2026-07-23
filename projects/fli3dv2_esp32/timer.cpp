/*
 * Fli3d - TM acquisition timer functionality 
 */

 #include <Arduino.h>
 #include <fli3dv2.h>
 #include "fli3dv2_esp32.h"
 
void setup_timer() {
    var.next_second = 1000*(millis()/1000) + 1000;
    var.next_tx_time = millis();
    var.pressure_interval = (1000 / cfg_esp32.pressure_tm_rate);
    var.motion_interval = (1000 / cfg_esp32.motion_tm_rate);
    var.gps_interval = (1000 / cfg_esp32.gps_tm_rate);
}

