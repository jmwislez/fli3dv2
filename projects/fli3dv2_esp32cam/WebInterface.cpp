#include "WebInterface.h"

#include <eloquent_esp32cam.h>

using eloq::camera;

WebInterface::WebInterface() :
    _cameraController(nullptr),
    _server(80) {
}

bool WebInterface::begin(
    CameraController* cameraController
) {
    _cameraController = cameraController;

    _server.on(
        "/snapshot",
        HTTP_GET,
        [this]() {
            handleSnapshot();
        }
    );

    _server.on(
        "/status",
        HTTP_GET,
        [this]() {
            handleStatus();
        }
    );

    _server.on(
        "/stream",
        HTTP_GET,
        [this]() {
            handleStream();
        }
    );

    _server.begin();

    Serial.println("Web server started");

    return true;
}

void WebInterface::process() {
    _server.handleClient();
}

void WebInterface::handleSnapshot() {

    if (!camera.capture().isOk()) {

        _server.send(
            500,
            "text/plain",
            "Capture failed"
        );

        return;
    }

    _server.send_P(
        200,
        "image/jpeg",
        (const char*)camera.frame->buf,
        camera.frame->len
    );
}

void WebInterface::handleStatus() {

    String json;

    json += "{";

    json += "\"captureRate\":";
    json += String(
        _cameraController->getCaptureRate()
    );

    json += ",";

    json += "\"streaming\":";
    json += _cameraController->isStreamingEnabled()
                ? "true"
                : "false";

    json += ",";

    json += "\"recording\":";
    json += _cameraController->isRecordingEnabled()
                ? "true"
                : "false";

    json += "}";

    _server.send(
        200,
        "application/json",
        json
    );
}

void WebInterface::handleStream() {

    if (!_cameraController->isStreamingEnabled()) {

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

    while (
        client.connected() &&
        _cameraController->isStreamingEnabled()
    ) {

        if (!camera.capture().isOk()) {
            delay(10);
            continue;
        }

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

        client.print("\r\n");

        delay(30);
    }
}
