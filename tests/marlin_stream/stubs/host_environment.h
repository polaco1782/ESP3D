#pragma once
#include <map>
#include <vector>

#include "Arduino.h"
// Substitute the platform boundaries using their existing include guards.
#define _ESP3D_CONFIG_H
#define COMMANDS_H
#define _SETTINGS_ESP3D_H
#define _ESP_FILESYSTEM_H
#define _ESP_SD_H
#ifdef TEST_MARLIN_WORKER
#define ARDUINO_ARCH_ESP32
#define _SERIAL_SERVICES_H
#define COMMUNICATION_PROTOCOL 1
#define RAW_SERIAL 1
#define ESP_SERIAL_BRIDGE_OUTPUT
#include <atomic>
using TestFlag = std::atomic<bool>;
using TestCount = std::atomic<unsigned>;
#else
#define ARDUINO_ARCH_ESP8266
using TestFlag = bool;
using TestCount = unsigned;
#endif
#define GCODE_HOST_FEATURE
#define FILESYSTEM_FEATURE
#define SD_DEVICE 1
#define MARLIN 20
#define MARLIN_EMBEDDED 30
#define SMOOTHIEWARE 40
#define ESP_SD_FS_HEADER "/SD"
#define ESP_FLASH_FS_HEADER "/FS"
#ifndef TEST_MARLIN_WORKER
#define portMAX_DELAY 0
#endif
#define esp3d_log(...) ((void)0)
#define esp3d_log_e(...) ((void)0)
#include "esp3d/src/core/esp3d_message.h"

extern std::map<std::string, std::string> testFiles;
extern TestFlag failRead;
extern TestCount fileReads, fileCloses, sdReleases, sdTouches;
class FakeFile {
  bool opened = false;
  std::string data;
  size_t position = 0;

 public:
  FakeFile() = default;
  explicit FakeFile(const char* path) {
    auto it = testFiles.find(path);
    if (it != testFiles.end()) {
      data = it->second;
      opened = true;
    }
  }
  bool isOpen() const { return opened; }
  size_t size() const { return data.size(); }
  void close() {
    if (opened) ++fileCloses;
    opened = false;
  }
  int available() const { return opened && position < data.size(); }
  int read() {
    return available() && !failRead
               ? static_cast<unsigned char>(data[position++])
               : -1;
  }
  size_t read(uint8_t* buffer, size_t size) {
    ++fileReads;
    if (failRead || !opened) return 0;
    size = std::min(size, data.size() - position);
    memcpy(buffer, data.data() + position, size);
    position += size;
    return size;
  }
};
using ESP_File = FakeFile;
using ESP_SDFile = FakeFile;
class ESP_FileSystem {
 public:
  static bool exists(const char* name) { return testFiles.count(name); }
  static FakeFile open(const char* name) { return FakeFile(name); }
};
class ESP_SD : public ESP_FileSystem {
 public:
  static bool accessFS() { return true; }
  static void releaseFS() { ++sdReleases; }
  static void touchWatchdog() { ++sdTouches; }
};
class ESP3DSettings {
 public:
  static uint8_t target;
  static uint8_t GetFirmwareTarget() { return target; }
};
struct Transmission {
  std::string wire;
  ESP3DAuthenticationLevel auth;
};
extern std::vector<Transmission> testTX;
extern std::vector<std::string> testReports;
class ESP3DCommands {
 public:
  ESP3DClientType getOutputClient() { return ESP3DClientType::serial_bridge; }
  bool dispatch(uint8_t* data, size_t size, ESP3DClientType target,
                ESP3DRequest, ESP3DMessageType, ESP3DClientType,
                ESP3DAuthenticationLevel auth) {
    if (target == getOutputClient())
      testTX.push_back({std::string((char*)data, size), auth});
    else
      testReports.emplace_back((char*)data, size);
    return true;
  }
  bool dispatch(const char* data, ESP3DClientType target, ESP3DRequest request,
                ESP3DMessageType type, ESP3DClientType origin,
                ESP3DAuthenticationLevel auth) {
    return dispatch((uint8_t*)data, strlen(data), target, request, type, origin,
                    auth);
  }
  bool is_esp_command(uint8_t* data, size_t size) {
    return size >= 4 && memcmp(data, "[ESP", 4) == 0;
  }
  void process(ESP3DMessage* msg) {
    dispatch(msg->data, msg->size, msg->target, msg->request_id, msg->type,
             msg->origin, msg->authentication_level);
    esp3d_message_manager.deleteMsg(msg);
  }
};
extern ESP3DCommands esp3d_commands;

#ifdef TEST_MARLIN_WORKER
class ESP3DSerialService {
 public:
  size_t writeBytes(const uint8_t* data, size_t size);
};
extern ESP3DSerialService esp3d_serial_service, serial_bridge_service;
#endif
