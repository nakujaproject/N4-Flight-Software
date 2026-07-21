//CustomSerialFlash.cpp
#include <algorithm> 
#include "CustomSerialFlash.h"


SerialFlashChip SerialFlash;

bool SerialFlashChip::begin(uint8_t pin, uint8_t wp, uint8_t hold) {
    cs_pin = pin;
    wp_pin = wp;
    hold_pin = hold;

    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);

    if (wp_pin != 255) {
        pinMode(wp_pin, OUTPUT);
        digitalWrite(wp_pin, HIGH);
    }
    if (hold_pin != 255) {
        pinMode(hold_pin, OUTPUT);
        digitalWrite(hold_pin, HIGH);
    }

    SPI.begin();
    delay(50);

    if (!initFileSystem()) {
        formatFileSystem();
        first_run = true;
    } else {
        first_run = false;
    }

    return true;
}

void SerialFlashChip::eraseAll() {
    //Serial.println("[eraseAll] Erasing entire flash...");
    for (uint32_t addr = 0; addr < CHIP_SIZE; addr += SECTOR_SIZE) {
        eraseSector(addr);
    }
    ///Serial.println("[eraseAll] Done.");
}

bool SerialFlashChip::create(const char *filename, uint32_t length) {
    Serial.printf("[create] Requested file: %s, size: %lu\n", filename, length);
    Serial.printf("[create] Next free address: 0x%06lX\n", next_free_address);

    if (strlen(filename) >= MAX_FILENAME_LEN - 1) {
        Serial.println("[create] Error: Filename too long.");
        return false;
    }

    if (exists(filename)) {
        Serial.println("[create] Error: File already exists.");
        return false;
    }

    int free_slot = -1;
    for (int i = 0; i < MAX_FILES; i++) {
        if (file_table[i].filename[0] == 0) {
            free_slot = i;
            break;
        }
    }

    if (free_slot == -1) {
        Serial.println("[create] Error: No free file table slot.");
        return false;
    }

    if ((next_free_address + length + sizeof(FileEntry)) > CHIP_SIZE) {
        Serial.printf("[create] Error: Not enough space! Needed: 0x%06lX\n", next_free_address + length);
        return false;
    }

    if (next_free_address % SECTOR_SIZE == 0) {
        eraseSector(next_free_address);
    }

    strncpy(file_table[free_slot].filename, filename, MAX_FILENAME_LEN);
    file_table[free_slot].start_address = next_free_address;
    file_table[free_slot].size = length;
    file_table[free_slot].crc = 0;

    saveFileTable();

    //Serial.println("[create] File successfully created.");
    next_free_address += length;
    return true;
}

bool SerialFlashChip::createErasable(const char* filename, uint32_t length) {
    //Serial.printf("[createErasable] Requested file: %s, size: %lu\n", filename, length);
    //Serial.printf("[createErasable] Next free address: 0x%06lX\n", next_free_address);

    if (strlen(filename) >= MAX_FILENAME_LEN - 1) {
        Serial.println("[createErasable] ❌ Filename too long.");
        return false;
    }

    if (exists(filename)) {
        Serial.println("[createErasable] ❌ File already exists.");
        return false;
    }

    int free_slot = -1;
    for (int i = 0; i < MAX_FILES; i++) {
        if (file_table[i].filename[0] == 0) {
            free_slot = i;
            break;
        }
    }

    if (free_slot == -1) {
        Serial.println("[createErasable] ❌ No free file table slot.");
        return false;
    }

    if ((next_free_address + length) > CHIP_SIZE) {
        Serial.printf("[createErasable] ❌ Not enough space. Needed end: 0x%06lX\n", next_free_address + length);
        return false;
    }

    // Erase all sectors touched by this file
    uint32_t start = next_free_address;
    uint32_t end   = next_free_address + length;
    uint32_t erase_addr = start & ~(SECTOR_SIZE - 1); // round down to sector boundary

    while (erase_addr < end) {
        Serial.printf("[createErasable] Erasing sector @ 0x%06lX\n", erase_addr);
        eraseSector(erase_addr);
        erase_addr += SECTOR_SIZE;
    }

    // Fill the file table entry
    strncpy(file_table[free_slot].filename, filename, MAX_FILENAME_LEN);
    file_table[free_slot].start_address = start;
    file_table[free_slot].size = length;
    file_table[free_slot].crc = 0;

    saveFileTable();
    next_free_address = end;

    //Serial.println("[createErasable] ✅ File created and sectors erased.");
    return true;
}


void SerialFlashChip::listFiles() {
    Serial.println("\n[File System Contents]");
    Serial.println("======================");
    bool found = false;
    for (int i = 0; i < MAX_FILES; i++) {
        if (file_table[i].filename[0] != 0) {
            Serial.printf("%-16s @ 0x%06lX (%lu bytes)\n",
                file_table[i].filename,
                file_table[i].start_address,
                file_table[i].size);
            found = true;
        }
    }
    if (!found) Serial.println("No files found.");
    Serial.printf("Next free: 0x%06lX\n", next_free_address);
}

bool SerialFlashChip::exists(const char *filename) {
    for (int i = 0; i < MAX_FILES; i++) {
        if (file_table[i].filename[0] != 0 && strcmp(file_table[i].filename, filename) == 0) {
            return true;
        }
    }
    return false;
}


bool SerialFlashChip::remove(const char *filename) {
    for (int i = 0; i < MAX_FILES; i++) {
        if (strcmp(file_table[i].filename, filename) == 0) {
            memset(&file_table[i], 0, sizeof(FileEntry));
            saveFileTable();  // ✅ this must be called!
            Serial.printf("[remove] ✅ Removed file: %s\n", filename);
            return true;
        }
    }
    Serial.printf("[remove] ❌ File not found: %s\n", filename);
    return false;
}



SerialFlashFile SerialFlashChip::open(const char *filename) {
    for (int i = 0; i < MAX_FILES; i++) {
        if (strcmp(file_table[i].filename, filename) == 0) {
            return SerialFlashFile(this, file_table[i].start_address, file_table[i].size);
        }
    }
    return SerialFlashFile(); // Invalid
}

void SerialFlashChip::write(uint32_t addr, const void *buf, uint32_t len) {
    if (addr >= CHIP_SIZE) return;
    len = std::min(len, CHIP_SIZE - addr);

    const uint8_t* p = (const uint8_t*)buf;
    while (len > 0) {
        writeEnable();
        uint32_t chunk = std::min(len, static_cast<uint32_t>(256));
        uint32_t page_end = (addr | 0xFF) + 1;
        chunk = std::min(chunk, page_end - addr);

        digitalWrite(cs_pin, LOW);
        SPI.transfer(0x02); // Page program
        SPI.transfer(addr >> 16);
        SPI.transfer(addr >> 8);
        SPI.transfer(addr);
        for (uint32_t i = 0; i < chunk; i++) {
            SPI.transfer(*p++);
        }
        digitalWrite(cs_pin, HIGH);
        if (!waitReady()) break;
        addr += chunk;
        len -= chunk;
    }
}

bool SerialFlashChip::initFileSystem() {
    read(METADATA_ADDRESS, &file_table, sizeof(file_table));
    //Serial.println("[initFileSystem] Reading file table...");

    // Validate CRCs and remove invalid entries
    for (int i = 0; i < MAX_FILES; i++) {
        if (file_table[i].filename[0] != 0) {
            FileEntry temp = file_table[i];
            temp.crc = 0;
            uint32_t expected_crc = calculateCRC(&temp, sizeof(temp));

            if (file_table[i].crc != expected_crc || file_table[i].size == 0) {
                Serial.printf("[initFileSystem] ⚠️ Invalid file entry (slot %d, name: %s). Erasing.\n", i, file_table[i].filename);
                memset(&file_table[i], 0, sizeof(FileEntry));
            }
        }
    }

    // Calculate next free address
    next_free_address = FILE_SYSTEM_START;
    for (int i = 0; i < MAX_FILES; i++) {
        if (file_table[i].filename[0] != 0) {
            uint32_t end = file_table[i].start_address + file_table[i].size;
            if (end > next_free_address) next_free_address = end;
        }
    }

    //Serial.printf("[initFileSystem] ✅ Initialized. Next free address: 0x%06lX\n", next_free_address);
    return true;
}


void SerialFlashChip::formatFileSystem() {
    for (uint32_t addr = 0; addr < 0x4000; addr += SECTOR_SIZE) {
        eraseSector(addr);
    }
    memset(file_table, 0, sizeof(file_table));
    next_free_address = FILE_SYSTEM_START;
    saveFileTable();
}

void SerialFlashChip::saveFileTable() {
    for (int i = 0; i < MAX_FILES; i++) {
        if (file_table[i].filename[0] != 0) {
            FileEntry temp = file_table[i];
            temp.crc = 0;
            file_table[i].crc = calculateCRC(&temp, sizeof(temp));
        }
    }
    eraseSector(METADATA_ADDRESS);
    write(METADATA_ADDRESS, &file_table, sizeof(file_table));
}

void SerialFlashChip::writeEnable() {
    digitalWrite(cs_pin, LOW);
    SPI.transfer(0x06); // Write Enable command
    digitalWrite(cs_pin, HIGH);
    delayMicroseconds(5);
}

bool SerialFlashChip::waitReady(uint32_t timeout) {
    uint32_t start = millis();
    while (millis() - start < timeout) {
        digitalWrite(cs_pin, LOW);
        SPI.transfer(0x05); // Read Status Register
        uint8_t status = SPI.transfer(0x00);
        digitalWrite(cs_pin, HIGH);
        if (!(status & 0x01)) return true; // Ready
        delay(1);
    }
    return false;
}


bool SerialFlashFile::erase() {
    if (!chip || length == 0) return false;

    uint8_t buffer[256];
    memset(buffer, 0xFF, sizeof(buffer));

    uint32_t remaining = length;
    uint32_t offset = 0;

    while (remaining > 0) {
        uint32_t chunk = std::min((uint32_t)256, (uint32_t)remaining);
        chip->write(address + offset, buffer, chunk);
        offset += chunk;
        remaining -= chunk;
    }

    seek(0);  // Reset write pointer
    return true;
}



uint32_t SerialFlashChip::calculateCRC(const void* data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    const uint8_t* p = (const uint8_t*)data;
    while (len--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++) {
            crc = (crc >> 1) ^ (0xEDB88320UL & -(crc & 1));
        }
    }
    return ~crc;
}

void SerialFlashChip::eraseSector(uint32_t addr) {
    if (addr >= CHIP_SIZE) return;

    writeEnable();
    digitalWrite(cs_pin, LOW);
    SPI.transfer(0x20); // Sector Erase
    SPI.transfer(addr >> 16);
    SPI.transfer(addr >> 8);
    SPI.transfer(addr);
    digitalWrite(cs_pin, HIGH);

    if (!waitReady(5000)) {
       // Serial.println("[eraseSector] Timeout!");
    }
}

void SerialFlashChip::read(uint32_t addr, void* buf, uint32_t len) {
    if (addr >= CHIP_SIZE) return;
    len = std::min(len, CHIP_SIZE - addr);

    digitalWrite(cs_pin, LOW);
    SPI.transfer(0x03); // Read command
    SPI.transfer(addr >> 16);
    SPI.transfer(addr >> 8);
    SPI.transfer(addr);

    uint8_t* p = (uint8_t*)buf;
    while (len--) {
        *p++ = SPI.transfer(0x00);
    }

    digitalWrite(cs_pin, HIGH);
}

