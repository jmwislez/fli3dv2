#ifndef VIDEO_RECORDER_H
#define VIDEO_RECORDER_H

#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>

class VideoRecorder {
public:
    VideoRecorder();

    bool begin(
        const String& filename,
        uint16_t width,
        uint16_t height,
        uint16_t fps
    );

    bool addFrame(
        const uint8_t* jpegData,
        uint32_t jpegSize
    );

    void stop();

    bool isRecording() const;

    uint32_t getFrameCount() const;

private:
    struct IndexEntry {
        uint32_t offset;
        uint32_t size;
    };

    File _aviFile;
    File _idxFile;

    String _aviFilename;
    String _idxFilename;

    uint16_t _width;
    uint16_t _height;
    uint16_t _fps;

    uint32_t _frameCount;

    uint32_t _moviDataStart;

    bool _recording;

    void writeAviHeader();
    void patchAviHeader();
    void writeIdx1();

    void writeLE16(uint16_t value);
    void writeLE32(uint32_t value);

    uint32_t filePosition();
};

#endif