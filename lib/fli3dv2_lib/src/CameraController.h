/* Based on vibe coded code using Copilot */

#ifndef CAMERA_CONTROLLER_H
#define CAMERA_CONTROLLER_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <FS.h>
#include <SD_MMC.h>
#include <time.h>

#include <eloquent_esp32cam.h>

#include "VideoRecorder.h"

using eloq::camera;

class CameraController {
public:

    CameraController();

    bool begin();

    void process();

    void startRecording();

    void stopRecording();

private:

    struct FrameData {
        uint8_t* data;
        size_t size;
    }; 

    WebServer _server;

    VideoRecorder _videoRecorder;

    uint16_t _frameWidth;
    uint16_t _frameHeight;

private:

    bool initializeCamera();

    bool initializeHttpServer();

    void handleFrameCapture();

    bool acquireFrame(FrameData& frame);

    void processFrame(const FrameData& frame);

    void saveSnapshotToSd(const uint8_t* jpegData, size_t jpegSize);

    void uploadImageWifi(const uint8_t* jpegData, size_t jpegSize);

    String generateAviFilename() const;

    String generateJpegFilename() const;

    String getTimestampString() const;

    void executeCommand(
        const String& command
    );

    void handleHttpStream();

    void handleHttpSnapshot();

    void handleHttpStatus();

    void streamFrameToClient(
        WiFiClient& client,
        const uint8_t* jpegData,
        size_t jpegSize
    );
};

#endif
