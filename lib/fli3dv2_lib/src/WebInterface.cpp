/* Based on vibecoded coding by Copilot */

#ifdef PLATFORM_ESP32CAM

#include "WebInterface.h"
#include <eloquent_esp32cam.h>
#include "fli3dv2.h"

using eloq::camera;

WebInterface::WebInterface() :
    _cameraController(nullptr),
    _server(80) {
}

bool WebInterface::begin(CameraController* cameraController) {
    _cameraController = cameraController;

    _server.on(
        "/image",
        HTTP_GET,
        [this]() {
            handleSnapshot();
        }
    );

    _server.on(
        "/video",
        HTTP_GET,
        [this]() {
            handleStream();
        }
    );

    _server.begin();

    return true;
}

void WebInterface::process() {
    _server.handleClient();

}

void WebInterface::handleSnapshot() {

    // Triggers frame acquisition
    if (!camera.capture().isOk()) {
        _server.send(
            500,
            "text/plain",
            "Capture failed"
        );
        return;
    }
    tm_camera.frame_rate++;
    tm_camera.frame_ctr++;

    _server.send_P(
        200,
        "image/jpeg",
        (const char*)camera.frame->buf,
        camera.frame->len
    );
    tm_esp32cam.wifi_active = true;
    tm_camera.wifi_images_active = true;
}

void WebInterface::handleStream() {
    // TODO: is blocking for main loop!

    if (tm_camera.wifi_video_enabled == false) {
        _server.send(
            503,
            "text/plain",
            "Streaming disabled"
        );
        return;
    }

    WiFiClient client =
        _server.client();

    client.print(
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n"
    );

    while (client.connected() && tm_camera.wifi_video_enabled) {

        if (!camera.capture().isOk()) {
            delay(10);
            continue;
        }
        tm_camera.frame_rate++;
        tm_camera.frame_ctr++;

        client.printf(
            "--frame\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %u\r\n\r\n",
            (unsigned)camera.frame->len
        );

        client.write(
            camera.frame->buf,
            camera.frame->len
        );

        tm_esp32cam.wifi_active = true;
        tm_camera.wifi_video_active = true;

        delay(30);
    }
}

#endif
