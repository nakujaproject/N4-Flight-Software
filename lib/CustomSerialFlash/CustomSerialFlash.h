#ifndef CUSTOM_SERIAL_FLASH_H
#define CUSTOM_SERIAL_FLASH_H

#include <Arduino.h>
#include <SPI.h>
#include <algorithm> // ✅ For std::min

#define SECTOR_SIZE 4096
#define CHIP_SIZE 1048576  // 1MB flash
#define FILE_SYSTEM_START 0x10000
#define METADATA_ADDRESS 0x00000
#define MAX_FILES 10
#define MAX_FILENAME_LEN 16

struct FileEntry {
    char filename[MAX_FILENAME_LEN];
    uint32_t start_address;
    uint32_t size;
    uint32_t crc;
};

class SerialFlashFile;

class SerialFlashChip {
public:
    SerialFlashChip() : next_free_address(FILE_SYSTEM_START) {}

    bool begin(uint8_t pin, uint8_t wp = 255, uint8_t hold = 255);
    bool create(const char *filename, uint32_t length);
    bool createErasable(const char* filename, uint32_t length);
    bool exists(const char *filename);
    bool remove(const char *filename);     
    SerialFlashFile open(const char *filename);

    void eraseAll();
    void eraseBlock(uint32_t address);    
    void eraseSector(uint32_t address);
    void formatFileSystem();
    void listFiles();

    void read(uint32_t addr, void *buf, uint32_t len);
    void write(uint32_t addr, const void *buf, uint32_t len);

    void readID(uint8_t *buf) { buf[0] = 0xC8; buf[1] = 0x40; buf[2] = 0x13; }
    uint32_t capacity(const uint8_t *id = nullptr) { return CHIP_SIZE; }
    uint32_t blockSize() { return SECTOR_SIZE; }
    bool ready(uint32_t timeout = 500) { return waitReady(timeout); }
    void wait(uint32_t timeout = 500) { waitReady(timeout); }

private:
    uint8_t cs_pin, wp_pin, hold_pin;
    FileEntry file_table[MAX_FILES];
    uint32_t next_free_address;
    bool first_run = true;

    bool initFileSystem();
    void saveFileTable();
    void writeEnable();
    bool waitReady(uint32_t timeout = 500);
    uint32_t calculateCRC(const void* data, uint32_t len);

    friend class SerialFlashFile;
};

class SerialFlashFile {
public:
    SerialFlashFile() : chip(nullptr), address(0), length(0), position_(0) {}
    SerialFlashFile(SerialFlashChip *chip, uint32_t addr, uint32_t len)
        : chip(chip), address(addr), length(len), position_(0) {}

    operator bool() { return chip != nullptr; }
    uint32_t size() const { return length; }
    uint32_t getAddress() const { return address; }

    bool erase(); // ✅ Public erase method for zeroing file contents

    void read(uint32_t addr, void *buf, uint32_t len) {
        chip->read(address + addr, buf, len);
    }

    void write(uint32_t addr, const void *buf, uint32_t len) {
        chip->write(address + addr, buf, len);
    }

    size_t write(const void *buf, size_t size) {
        size_t written = 0;
        if (position_ < length) {
            size_t to_write = std::min(size, (size_t)(length - position_));
            write(position_, buf, to_write);
            position_ += to_write;
            written = to_write;
        }
        return written;
    }

    size_t readBytes(void *buf, size_t size) {
        size_t total = 0;
        if (position_ < length) {
            size_t to_read = std::min(size, (size_t)(length - position_));
            chip->read(address + position_, buf, to_read);
            position_ += to_read;
            total = to_read;
        }
        return total;
    }

    void seek(uint32_t pos) { position_ = std::min(pos, length); }
    uint32_t position() const { return position_; }

    void close() {
        chip = nullptr;
        address = 0;
        length = 0;
        position_ = 0;
    }

private:
    SerialFlashChip *chip;
    uint32_t address;
    uint32_t length;
    uint32_t position_;
};

extern SerialFlashChip SerialFlash;

#endif
