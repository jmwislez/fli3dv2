#include "CameraController.h"
#include "fli3dv2.h"

static CameraController* g_instance = nullptr;

CameraController::CameraController() :
    _captureMode(MODE_SINGLE),
    _wifiImagesEnabled(false),
    _sdImagesEnabled(false),
    _videoStreamingEnabled(false),
    _videoRecordingEnabled(false),
    _singleShotPending(true),
    _captureRateHz(1.0f),
    _captureIntervalMs(1000),
    _lastCaptureMs(0),
    _server(80),
    _frameWidth(640),
    _frameHeight(480),
    _cameraReady(false)
{
    g_instance = this;
}

bool CameraController::begin() {
    tm_camera.camera_mode = CAM_INIT;

    _cameraReady = initializeCamera();

    if (!_cameraReady) {
        sprintf(buffer, "Camera OV2640 initialization failed");
        publish_event(STS_THIS, SS_CAMERA, EVENT_ERROR, buffer);
        tm_camera.camera_mode = CAM_NONE;
        return false;
    }

    if (_captureMode == MODE_SINGLE) {
        tm_camera.camera_mode = CAM_SINGLE;
        tm_camera.framerate = _captureRateHz;
    }
    else {
        tm_camera.camera_mode = CAM_CONTINUOUS;
        tm_camera.framerate = _videoRecorder.getFrameCount();
    }
    tm_camera.resolution = RES_640x480; // TODO: make resolution settable
    tm_camera.wifi_images_enabled = _wifiImagesEnabled;
    tm_camera.sd_images_enabled = _sdImagesEnabled;
    tm_camera.wifi_video_enabled = _videoStreamingEnabled;
    tm_camera.sd_video_enabled = _videoRecordingEnabled;
    tm_camera.framecount = 0;
    tm_camera.wifi_images_active = false;
    tm_camera.sd_images_active = false;
    tm_camera.wifi_video_active = false;
    tm_camera.sd_video_active = _videoRecorder.isRecording();
    
    tm_camera.http_server_enabled = initializeHttpServer();
    tm_camera.http_server_enabled = true;
    tm_camera.http_server_active = false;

    sprintf(buffer, "Camera OV2640 initialized");
    publish_event(STS_THIS, SS_CAMERA, EVENT_INIT, buffer);

    return true;
}

void CameraController::process() {

    _server.handleClient();

    uint32_t now = millis();

    switch (_captureMode) {

        case MODE_SINGLE:

            if (_singleShotPending) {
                handleFrameCapture();
                _singleShotPending = false;
            }

            break;

        case MODE_CONTINUOUS:

            if (now - _lastCaptureMs >= _captureIntervalMs) {
                _lastCaptureMs = now;
                handleFrameCapture();
            }

            break;
    }
}

void CameraController::setMode(CaptureMode mode) {

    _captureMode = mode;

    if (mode == MODE_SINGLE)
        _singleShotPending = true;
}

void CameraController::setRate(float hz) {

    if (hz <= 0.0f)
        return;

    _captureRateHz = hz;

    _captureIntervalMs =
        (uint32_t)(1000.0f / hz);
}

void CameraController::enableWifiImages(bool enable) {
    _wifiImagesEnabled = enable;
}

void CameraController::enableSdImages(bool enable) {
    _sdImagesEnabled = enable;
}

void CameraController::enableVideoStreaming(bool enable) {
    _videoStreamingEnabled = enable;
}

void CameraController::enableVideoRecording(bool enable) {

    _videoRecordingEnabled = enable;

    if (enable)
        startRecording();
    else
        stopRecording();
}

bool CameraController::isStreamingEnabled() const {
    return _videoStreamingEnabled;
}

bool CameraController::isRecordingEnabled() const {
    return _videoRecordingEnabled;
}

bool CameraController::isWifiImagesEnabled() const {
    return _wifiImagesEnabled;
}

bool CameraController::isSdImagesEnabled() const {
    return _sdImagesEnabled;
}

float CameraController::getCaptureRate() const {
    return _captureRateHz;
}

bool CameraController::initializeCamera() {

    camera.pinout.aithinker();

    camera.brownout.disable();

    camera.resolution.vga(); // TODO: set resolution

    camera.quality.high();

    if (!camera.begin().isOk()) {

        sprintf(buffer, "Camera initialization error: %s", camera.exception.toString());
        publish_event(STS_THIS, SS_CAMERA, EVENT_ERROR, buffer);

        return false;
    }

    return true;
}

bool CameraController::initializeHttpServer() {
    if (tm_esp32cam.wifi_sta_enabled) {
        _server.on(
            "/status",
            HTTP_GET,
            [this]() {
                handleHttpStatus();
            }
        );

        _server.on(
            "/snapshot",
            HTTP_GET,
            [this]() {
                handleHttpSnapshot();
            }
        );

        _server.on(
            "/stream",
            HTTP_GET,
            [this]() {
                handleHttpStream();
            }
        );

        _server.begin();

        sprintf(buffer, "Image server initialized: http://%s/snapshot", WiFi.localIP());
        publish_event(STS_THIS, SS_CAMERA, EVENT_INIT, buffer);   

        sprintf(buffer, "Video server initialized: http://%s/stream", WiFi.localIP());
        publish_event(STS_THIS, SS_CAMERA, EVENT_INIT, buffer);   
    
        return true;
    }
    else {
        return false;
    }
}

void CameraController::handleFrameCapture() {

    FrameData frame;

    if (!acquireFrame(frame))
        return;

    processFrame(frame);
}

bool CameraController::acquireFrame(
    FrameData& frame
) {
    if (!camera.capture().isOk()) {
        sprintf(buffer, "Camera capture: %s", camera.exception.toString());
        publish_event(STS_THIS, SS_CAMERA, EVENT_ERROR, buffer);
        return false;
    }

    frame.data = camera.frame->buf;
    frame.size = camera.frame->len;

    return true;
}

void CameraController::processFrame(
    const FrameData& frame
) {
    if (_sdImagesEnabled)
        saveSnapshotToSd(
            frame.data,
            frame.size
        );

    if (_wifiImagesEnabled)
        uploadImageWifi(
            frame.data,
            frame.size
        );

    if (_videoRecorder.isRecording()) {
        _videoRecorder.addFrame(
            frame.data,
            frame.size
        );
    }
}

void CameraController::saveSnapshotToSd(
    const uint8_t* jpegData,
    size_t jpegSize
) {
    if (!tm_esp32cam.sd_enabled)
        return;

    String filename =
        generateJpegFilename();

    File file =
        SD_MMC.open(
            filename,
            FILE_WRITE
        );

    if (!file)
        return;

    file.write(
        jpegData,
        jpegSize
    );

    file.close();

    Serial.print("Saved ");
    Serial.println(filename);
}

void CameraController::uploadImageWifi(
    const uint8_t* jpegData,
    size_t jpegSize
) {
    (void)jpegData;
    (void)jpegSize;

    Serial.println(
        "WiFi upload hook not implemented"
    );
}

void CameraController::startRecording() {

    if (!tm_esp32cam.sd_enabled)
        return;

    if (_videoRecorder.isRecording())
        return;

    String filename =
        generateAviFilename();

    if (_videoRecorder.begin(
            filename,
            _frameWidth,
            _frameHeight,
            (uint16_t)_captureRateHz
        )) {

        sprintf(buffer, "Recording video to %s on SD", filename);
        publish_event(STS_THIS, SS_CAMERA, EVENT_INFO, buffer);
    }
}

void CameraController::stopRecording() {

    if (!_videoRecorder.isRecording())
        return;

    _videoRecorder.stop();

    sprintf(buffer, "Stopped recording video to SD");
    publish_event(STS_THIS, SS_CAMERA, EVENT_INFO, buffer);
    
}

String CameraController::generateAviFilename() const {

    return "/" +
           getTimestampString() +
           ".avi";
}

String CameraController::generateJpegFilename() const {

    return "/" +
           getTimestampString() +
           ".jpg";
}

String CameraController::getTimestampString() const {

    struct tm timeinfo;

    if (!getLocalTime(&timeinfo))
        return String(millis());

    char buffer[32];

    strftime(
        buffer,
        sizeof(buffer),
        "%Y%m%d_%H%M%S",
        &timeinfo
    );

    return String(buffer);
}

void CameraController::executeCommand(
    const String& command
) {

    if (command == "status") {

        printStatus();
        return;
    }

    if (command == "snapshot") {

        handleFrameCapture();
        return;
    }

    if (command == "set_mode single") {

        setMode(MODE_SINGLE);
        return;
    }

    if (command == "set_mode continuous") {

        setMode(MODE_CONTINUOUS);
        return;
    }

    if (command.startsWith("set_rate ")) {

        float hz =
            command.substring(9).toFloat();

        setRate(hz);

        return;
    }

    if (command == "send_images_wifi on") {

        enableWifiImages(true);
        return;
    }

    if (command == "send_images_wifi off") {

        enableWifiImages(false);
        return;
    }

    if (command == "send_images_sd on") {

        enableSdImages(true);
        return;
    }

    if (command == "send_images_sd off") {

        enableSdImages(false);
        return;
    }

    if (command == "video_stream on") {

        enableVideoStreaming(true);
        return;
    }

    if (command == "video_stream off") {

        enableVideoStreaming(false);
        return;
    }

    if (command == "record_video on") {

        enableVideoRecording(true);
        return;
    }

    if (command == "record_video off") {

        enableVideoRecording(false);
        return;
    }

    Serial.print("Unknown command: ");
    Serial.println(command);
}

void CameraController::handleHttpSnapshot() {

    tm_camera.http_server_active = true;
    
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

void CameraController::handleHttpStatus() {

    String json;

    json += "{";
    json += "\"mode\":\"";
    json += (_captureMode == MODE_SINGLE)
                ? "single"
                : "continuous";
    json += "\",";
    json += "\"rate\":";
    json += String(_captureRateHz);
    json += ",";
    json += "\"streaming\":";
    json += _videoStreamingEnabled ? "true" : "false";
    json += ",";
    json += "\"recording\":";
    json += _videoRecorder.isRecording()
                 ? "true"
                 : "false";
    json += "}";

    _server.send(
        200,
        "application/json",
        json
    );
}

void CameraController::streamFrameToClient(
    WiFiClient& client,
    const uint8_t* jpegData,
    size_t jpegSize
) {

    client.printf(
        "--frame\r\n"
        "Content-Type: image/jpeg\r\n"
        "Content-Length: %u\r\n\r\n",
        (unsigned)jpegSize
    );

    client.write(
        jpegData,
        jpegSize
    );

    client.print("\r\n");
}

void CameraController::handleHttpStream() {

    if (!_videoStreamingEnabled) {

        _server.send(
            503,
            "text/plain",
            "Video streaming disabled"
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
        _videoStreamingEnabled
    ) {

        if (!camera.capture().isOk()) {
            delay(10);
            continue;
        }

        streamFrameToClient(
            client,
            camera.frame->buf,
            camera.frame->len
        );

        delay(30);
    }
}