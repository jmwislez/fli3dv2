/* Based on vibe coded code using Copilot */

#include "CameraController.h"
#include "fli3dv2.h"
#include <sys/time.h>

static CameraController* g_instance = nullptr;

CameraController::CameraController() :
    _server(80)
{
    g_instance = this;
}

bool CameraController::begin() {
    tm_camera.mode = CAM_INIT;
    tm_esp32cam.camera_enabled = initializeCamera();
    if (!tm_esp32cam.camera_enabled) {
        sprintf(buffer, "Camera OV2640 initialization failed");
        publish_event(STS_THIS, SS_CAMERA, EVENT_ERROR, buffer);
        tm_camera.mode = CAM_FAIL;
        return false;
    }
    set_camera_mode(CAM_IDLE);
    var.camera_interval = (uint16_t)(1000.0f / cfg_esp32cam.camera_image_rate);

    sprintf(buffer, "Camera OV2640 initialized");
    publish_event(STS_THIS, SS_CAMERA, EVENT_INIT, buffer);

    tm_camera.http_server_enabled = initializeHttpServer();

    if(!tm_camera.http_server_enabled) {
        cfg_esp32cam.wifi_images_enable = false;
        cfg_esp32cam.wifi_video_enable = false;
    }
    if(!tm_esp32cam.sd_enabled) {
        cfg_esp32cam.sd_images_enable = false;
        cfg_esp32cam.sd_video_enable = false;
    } 
    
    return true;
}

void CameraController::process() {

    _server.handleClient();

    uint32_t now = millis();

    switch (tm_camera.mode) {
        case CAM_SINGLE:
            handleFrameCapture();
            tm_esp32cam.sd_active = true;
            set_camera_mode(CAM_IDLE);    
            break;
        case CAM_IMAGES:
            if (now - var.last_camera_time >= var.camera_interval) {
                var.last_camera_time = now;
                handleFrameCapture();
                tm_esp32cam.sd_active = true;
            }
            break;
        case CAM_VIDEO:
            if (now - var.last_camera_time >= var.camera_interval) {
                var.last_camera_time = now;
                handleFrameCapture();
                tm_esp32cam.sd_active = true;
            }
            break;
    }
}

bool CameraController::initializeCamera() {

    camera.pinout.aithinker();

    camera.brownout.disable();

    switch(tm_camera.resolution) {
        case QQVGA_160x120:     camera.resolution.qqvga(); 
                                _frameWidth = 160;
                                _frameHeight = 120;
                                break;
        case QVGA_320x240:      camera.resolution.qvga(); 
                                _frameWidth = 320;
                                _frameHeight = 240;
                                break;
        case HVGA_480x320:      camera.resolution.hvga(); 
                                _frameWidth = 480;
                                _frameHeight = 320;
                                break;
        case VGA_640x480:       camera.resolution.vga(); 
                                _frameWidth = 640;
                                _frameHeight = 480;
                                break;
        case SVGA_800x600:      camera.resolution.svga(); 
                                _frameWidth = 800;
                                _frameHeight = 600;
                                break;
        case XGA_1024x768:      camera.resolution.xga(); 
                                _frameWidth = 1024;
                                _frameHeight = 768;
                                break;
        case SXGA_1280x1024:    camera.resolution.sxga();
                                _frameWidth = 1280;
                                _frameHeight = 1024;
                                break; 
        case UXGA_1600x1200:    camera.resolution.uxga(); 
                                _frameWidth = 1600;
                                _frameHeight = 1200;
                                break;
        default:                camera.resolution.vga(); 
                                tm_camera.resolution = VGA_640x480;
                                _frameWidth = 640;
                                _frameHeight = 480;
                                break;
    }

    camera.quality.high();

    if (!camera.begin().isOk()) {
        return false;
    }
    return true;
}

bool CameraController::initializeHttpServer() {
    if (tm_esp32cam.wifi_sta_enabled || tm_esp32cam.wifi_ap_enabled) {

        if(cfg_esp32cam.wifi_images_enable) {
            _server.on(
                "/images",
                HTTP_GET,
                [this]() {
                    handleHttpSnapshot();
                }
            );
            sprintf(buffer, "Image server initialized: http://%s/images", WiFi.localIP().toString().c_str());
            publish_event(STS_THIS, SS_CAMERA, EVENT_INIT, buffer);   
            tm_camera.wifi_images_enabled = true;
        }

        if(cfg_esp32cam.wifi_video_enable) {
            _server.on(
                "/video",
                HTTP_GET,
                [this]() {
                    handleHttpStream();
                }
            );
            sprintf(buffer, "Video server initialized: http://%s/video (video mode needed)", WiFi.localIP().toString().c_str());
            publish_event(STS_THIS, SS_CAMERA, EVENT_INIT, buffer);
            tm_camera.wifi_video_enabled = true;
        }

        _server.begin();
    
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

    tm_camera.frame_ctr++;
    tm_camera.frame_rate++;

    return true;
}

void CameraController::processFrame(
    const FrameData& frame
) {
    if (tm_camera.sd_images_enabled) {
        saveSnapshotToSd(
            frame.data,
            frame.size
        );
    }

    if (tm_camera.wifi_images_enabled || tm_camera.wifi_video_enabled) {
        uploadImageWifi(
            frame.data,
            frame.size
        );
        if(tm_camera.mode == CAM_VIDEO) {
            tm_camera.wifi_video_active = true;
            tm_summary.wifi_video_active = true;
        }
        else {
            tm_camera.wifi_images_active = true;
            tm_summary.wifi_images_active = true;
        }
    }

    if (tm_camera.sd_video_enabled) {
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

    if (!tm_esp32cam.sd_enabled or !cfg_esp32cam.write_fs_enable)
        return;

    String filename =
        generateJpegFilename();

    File file =
        SD_MMC.open(
            filename,
            FILE_WRITE
        );

    if (!file) {
        return;
    }

    file.write(
        jpegData,
        jpegSize
    );

    file.close();

    tm_esp32cam.sd_active = true;
    tm_camera.sd_images_active = true;
    tm_summary.sd_images_active = true;

    sprintf(tm_camera.filename, "%s", filename.c_str());
}

void CameraController::uploadImageWifi(
    const uint8_t* jpegData,
    size_t jpegSize
) {
    (void)jpegData;
    (void)jpegSize;

    // TODO: WiFi upload hook not implemented
}

void CameraController::startRecording() {

    if (!tm_esp32cam.sd_enabled or !cfg_esp32cam.write_fs_enable)
        return;

    if (_videoRecorder.isRecording())
        return;

    String filename =
        generateAviFilename();

    if (_videoRecorder.begin(
            filename,
            _frameWidth,
            _frameHeight,
            (uint16_t)cfg_esp32cam.camera_image_rate
        )) {

        sprintf(buffer, "Recording video to %s on SD", filename.c_str());
        publish_event(STS_THIS, SS_CAMERA, EVENT_INFO, buffer);
        tm_esp32cam.sd_active = true;
        strcpy(tm_camera.filename, filename.c_str());
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

    return String(var.today_directory) + "/" +
           getTimestampString() +
           ".avi";
}

String CameraController::generateJpegFilename() const {

    return String(var.today_directory) + "/" +
           getTimestampString() +
           ".jpg";
}

String CameraController::getTimestampString() const {

    struct timeval current_time;
    struct tm timeinfo;

    if (gettimeofday(&current_time, nullptr) != 0 ||
        localtime_r(&current_time.tv_sec, &timeinfo) == nullptr)
        return String(millis());

    char buffer[32];

    strftime(
        buffer,
        sizeof(buffer),
        "%H%M%S",
        &timeinfo
    );

    char timestamp[40];
    snprintf(
        timestamp,
        sizeof(timestamp),
        "%s_%03lu",
        buffer,
        (unsigned long)(current_time.tv_usec / 1000L)
    );

    return String(timestamp);
}

void CameraController::handleHttpSnapshot() {

    tm_camera.http_server_active = true;
    tm_esp32cam.wifi_active = true;
    
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

    tm_camera.wifi_images_active = true;
    tm_summary.wifi_images_active = true;
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
}

void CameraController::handleHttpStream() {

    if (!tm_camera.wifi_video_enabled) {

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

    while (client.connected() && tm_camera.wifi_video_enabled) {

        if (!camera.capture().isOk()) {
            delay(10);
            continue;
        }

        tm_camera.frame_rate++;
        tm_camera.frame_ctr++;

        streamFrameToClient(
            client,
            camera.frame->buf,
            camera.frame->len
        );

        tm_camera.wifi_video_active = true;
        tm_summary.wifi_video_active = true;

        delay(30);
    }
}
