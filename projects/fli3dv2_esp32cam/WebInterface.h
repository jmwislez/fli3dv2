#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include "CameraController.h"

class WebInterface {
public:
    WebInterface();

    bool begin(CameraController* cameraController);

    void process();

private:
    CameraController* _cameraController;

    WebServer _server;

    void handleSnapshot();
    void handleStatus();
    void handleStream();
};

#endif