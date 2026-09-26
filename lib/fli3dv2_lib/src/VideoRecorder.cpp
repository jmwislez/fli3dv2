/* Vibe coded using Copilot */

#include "VideoRecorder.h"
#include "fli3dv2.h"

VideoRecorder::VideoRecorder() :
    _width(0),
    _height(0),
    _fps(0),
    _frameCount(0),
    _moviDataStart(0),
    _recording(false) {
}

bool VideoRecorder::begin(
    const String& filename,
    uint16_t width,
    uint16_t height,
    uint16_t fps
) {
    if (_recording)
        return false;

    _width = width;
    _height = height;
    _fps = fps;

    _frameCount = 0;

    _aviFilename = filename;

    _idxFilename = filename;
    _idxFilename.replace(".avi", ".idx");

    _aviFile = SD_MMC.open(
        _aviFilename,
        FILE_WRITE
    );

    if (!_aviFile)
        return false;

    _idxFile = SD_MMC.open(
        _idxFilename,
        FILE_WRITE
    );

    if (!_idxFile) {
        _aviFile.close();
        return false;
    }

    writeAviHeader();

    _recording = true;

    return true;
}

bool VideoRecorder::addFrame(
    const uint8_t* jpegData,
    uint32_t jpegSize
) {
    if (!_recording)
        return false;

    uint32_t chunkStart =
        _aviFile.position();

    _aviFile.write(
        (const uint8_t*)"00dc",
        4
    );

    writeLE32(jpegSize);

    _aviFile.write(
        jpegData,
        jpegSize
    );

    if (jpegSize & 1)
        _aviFile.write((uint8_t)0);

    IndexEntry e;

    e.offset =
        chunkStart - _moviDataStart;

    e.size =
        jpegSize;

    _idxFile.write(
        (uint8_t*)&e,
        sizeof(IndexEntry)
    );

    _frameCount++;
    tm_camera.sd_video_active = true;
    tm_summary.sd_video_active = true;
    tm_esp32cam.sd_active = true;

    return true;
}

void VideoRecorder::stop() {
    if (!_recording)
        return;

    writeIdx1();

    patchAviHeader();

    _aviFile.flush();
    _aviFile.close();

    _idxFile.close();

    SD_MMC.remove(_idxFilename);

    _recording = false;

    strcpy (tm_camera.filename, "");
}

bool VideoRecorder::isRecording() const {
    return _recording;
}

uint32_t VideoRecorder::getFrameCount() const {
    return _frameCount;
}

void VideoRecorder::writeLE16(uint16_t value) {
    _aviFile.write((uint8_t*)&value, 2);
}

void VideoRecorder::writeLE32(uint32_t value) {
    _aviFile.write((uint8_t*)&value, 4);
}

uint32_t VideoRecorder::filePosition() {
    return _aviFile.position();
}

void VideoRecorder::writeAviHeader() {

    _aviFile.write((const uint8_t*)"RIFF", 4);
    writeLE32(0);
    _aviFile.write((const uint8_t*)"AVI ", 4);

    _aviFile.write((const uint8_t*)"LIST", 4);
    writeLE32(192);
    _aviFile.write((const uint8_t*)"hdrl", 4);

    _aviFile.write((const uint8_t*)"avih", 4);
    writeLE32(56);

    writeLE32(1000000UL / _fps);

    writeLE32(0);
    writeLE32(0);

    writeLE32(0x10);

    writeLE32(0);

    writeLE32(0);

    writeLE32(1);

    writeLE32(0);

    writeLE32(_width);
    writeLE32(_height);

    for (int i = 0; i < 4; i++)
        writeLE32(0);

    _aviFile.write((const uint8_t*)"LIST", 4);
    writeLE32(116);
    _aviFile.write((const uint8_t*)"strl", 4);

    _aviFile.write((const uint8_t*)"strh", 4);
    writeLE32(56);

    _aviFile.write((const uint8_t*)"vids", 4);
    _aviFile.write((const uint8_t*)"MJPG", 4);

    writeLE32(0);
    writeLE16(0);
    writeLE16(0);

    writeLE32(0);

    writeLE32(1);
    writeLE32(_fps);

    writeLE32(0);

    writeLE32(0);

    writeLE32(0);

    writeLE32(0);

    writeLE32(0);

    writeLE16(0);
    writeLE16(0);
    writeLE16(_width);
    writeLE16(_height);

    _aviFile.write((const uint8_t*)"strf", 4);
    writeLE32(40);

    writeLE32(40);

    writeLE32(_width);
    writeLE32(_height);

    writeLE16(1);
    writeLE16(24);

    _aviFile.write((const uint8_t*)"MJPG", 4);

    writeLE32(_width * _height * 3);

    writeLE32(0);
    writeLE32(0);

    writeLE32(0);
    writeLE32(0);

    _aviFile.write((const uint8_t*)"LIST", 4);

    writeLE32(0);

    _aviFile.write((const uint8_t*)"movi", 4);

    _moviDataStart =
        _aviFile.position();
}

void VideoRecorder::writeIdx1() {

    _idxFile.flush();

    File idxRead =
        SD_MMC.open(_idxFilename);

    if (!idxRead)
        return;

    uint32_t entryCount =
        idxRead.size() /
        sizeof(IndexEntry);

    uint32_t idxSize =
        entryCount * 16;

    _aviFile.write(
        (const uint8_t*)"idx1",
        4
    );

    writeLE32(idxSize);

    IndexEntry e;

    while (
        idxRead.read(
            (uint8_t*)&e,
            sizeof(IndexEntry)
        ) == sizeof(IndexEntry)
    ) {

        _aviFile.write(
            (const uint8_t*)"00dc",
            4
        );

        writeLE32(0x10);

        writeLE32(e.offset);

        writeLE32(e.size);
    }

    idxRead.close();
}

void VideoRecorder::patchAviHeader() {

    uint32_t fileSize =
        _aviFile.size();

    _aviFile.seek(4);

    uint32_t riffSize =
        fileSize - 8;

    writeLE32(riffSize);

    _aviFile.seek(48);

    writeLE32(_frameCount);

    _aviFile.seek(140);

    writeLE32(_frameCount);

    uint32_t moviSize =
        fileSize -
        _moviDataStart -
        8;

    _aviFile.seek(
        _moviDataStart - 8
    );

    writeLE32(moviSize);
}
