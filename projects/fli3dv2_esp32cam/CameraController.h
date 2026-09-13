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

    enum CaptureMode {
        MODE_SINGLE,
        MODE_CONTINUOUS
    };

    CameraController();

    bool begin();

    void process();

    void printStatus();

    void setMode(CaptureMode mode);

    void setRate(float hz);

    void enableWifiImages(bool enable);

    void enableSdImages(bool enable);

    void enableVideoStreaming(bool enable);

    void enableVideoRecording(bool enable);

    bool isStreamingEnabled() const;

    bool isRecordingEnabled() const;

    bool isWifiImagesEnabled() const;

    bool isSdImagesEnabled() const;

    float getCaptureRate() const;

private:

    struct FrameData {
        uint8_t* data;
        size_t size;
    };

    CaptureMode _captureMode;

    bool _wifiImagesEnabled;
    bool _sdImagesEnabled;
    bool _videoStreamingEnabled;
    bool _videoRecordingEnabled;

    bool _singleShotPending;

    float _captureRateHz;

    uint32_t _captureIntervalMs;

    uint32_t _lastCaptureMs;

    WebServer _server;

    VideoRecorder _videoRecorder;

    uint16_t _frameWidth;
    uint16_t _frameHeight;

    bool _cameraReady;

private:

    bool initializeCamera();

    bool initializeHttpServer();

    void handleFrameCapture();

    bool acquireFrame(FrameData& frame);

    void processFrame(const FrameData& frame);

    void saveSnapshotToSd(
        const uint8_t* jpegData,
        size_t jpegSize
    );

    void uploadImageWifi(
        const uint8_t* jpegData,
        size_t jpegSize
    );

    void startRecording();

    void stopRecording();

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