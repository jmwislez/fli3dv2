// Support for OV2620 camera on ESP32CAM module 

#include <fli3dv2.h>
#include <eloquent_esp32cam.h>
#include <eloquent_esp32cam/viz/mjpeg.h>

using namespace eloq;
using namespace eloq::viz;

bool setup_ov2640() {
    if (cfg_esp32cam.camera_enable) {// camera settings
        camera.pinout.aithinker();
        camera.brownout.disable();
        camera.resolution.vga();
        camera.quality.high();

        while (!camera.begin().isOk())
            Serial.println(camera.exception.toString());

        while (!mjpeg.begin().isOk())
            Serial.println(mjpeg.exception.toString());
    
        tm_esp32cam.camera_enabled = true;

        sprintf(buffer, "Camera OV2640 initialized: mjpeg stream at %s", mjpeg.address().c_str());
        publish_event(STS_THIS, SS_THIS, EVENT_INIT, buffer);

        return true;
    }
    return false;
}