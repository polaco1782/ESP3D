/*
  gcode_host.h -  gcode host functions class

  Copyright (c) 2014 Luc Lebosse. All rights reserved.

  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with This code; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _GCODE_HOST_H
#define _GCODE_HOST_H

#include <Arduino.h>
#include "../../core/esp3d_message.h"
#include "../authentication/authentication_service.h"
#include "./gcode_script_fifo.h"
#include "marlin_stream.h"
#if defined(ARDUINO_ARCH_ESP32)
#include <atomic>
#include "freertos/queue.h"
#include "freertos/task.h"
#endif

#define ERROR_NO_ERROR 0
#define ERROR_TIME_OUT 1
#define ERROR_CANNOT_SEND_DATA 2
#define ERROR_LINE_NUMBER 3
#define ERROR_ACK_NUMBER 4
#define ERROR_MEMORY_PROBLEM 5
#define ERROR_RESEND 6
#define ERROR_NUMBER_MISMATCH 7
#define ERROR_LINE_IGNORED 8
#define ERROR_FILE_SYSTEM 9
#define ERROR_CHECKSUM 10
#define ERROR_UNKNOW 11
#define ERROR_FILE_NOT_FOUND 12
#define ERROR_STREAM_ABORTED 13

#define HOST_NO_STREAM 100
#define HOST_START_STREAM 101
#define HOST_READ_LINE 102
#define HOST_PROCESS_LINE 103
#define HOST_WAIT4_ACK 104
#define HOST_PAUSE_STREAM 105
#define HOST_RESUME_STREAM 106
#define HOST_STOP_STREAM 107
#define HOST_ERROR_STREAM 108
#define HOST_ABORT_STREAM 109
#define HOST_PAUSING_STREAM 110

#define TYPE_SCRIPT_STREAM 0
#define TYPE_FS_STREAM 1
#define TYPE_SD_STREAM 2

#define ESP_HOST_BUFFER_SIZE 255

class GcodeHost {
  // Lock order: host -> ingress. UART ingress never takes the host lock and
  // never calls the network dispatcher. No HTTP work runs under the worker.
  class Guard {
   public:
    explicit Guard(const GcodeHost& host) : _host(host) {
#if defined(ARDUINO_ARCH_ESP32)
      if (_host._hostMutex) xSemaphoreTakeRecursive(_host._hostMutex, portMAX_DELAY);
#endif
    }
    ~Guard() {
#if defined(ARDUINO_ARCH_ESP32)
      if (_host._hostMutex) xSemaphoreGiveRecursive(_host._hostMutex);
#endif
    }
   private:
    const GcodeHost& _host;
  };
 public:
  GcodeHost();
  ~GcodeHost();
  bool begin();
  void end();
  void handle();
  bool push(const uint8_t* sbuf, size_t len);
  void flush();
  /*bool sendCommand(const char* command, bool checksum = false, bool wait4ack =
   * true, const char * ack=nullptr);*/
  uint32_t currentCommandNumber() { Guard guard(*this); return _commandNumber; }
  void setCommandNumber(uint32_t n) { Guard guard(*this); _commandNumber = n; }
  bool resetCommandNumbering();
  uint8_t Checksum(const char* command, uint32_t commandSize);
  String CheckSumCommand(const char* command, uint32_t commandnb);

  bool dispatch(ESP3DMessage* message);
  bool ownsPrinter() const;
  bool queueTemperature(const char* command);
  ESP3DClientType printerClient() const { Guard guard(*this); return _marlinOutput; }
  void emergencyStop();
  String diagnostics();
#if defined(ARDUINO_ARCH_ESP32)
  // Called by the UART event task before the main-loop message FIFO.
  bool captureSerial(ESP3DClientType origin, const uint8_t* data, size_t size);
  void recordSerialError(ESP3DClientType origin, unsigned error);
  bool backgroundOwnsSD() const { return _workerSDLease.load(); }
  int workerCore() const { return _workerCore; }
  bool workerUsesSerial(ESP3DClientType origin) const {
    Guard guard(*this);
    return _workerActive.load() && _marlinOutput == origin;
  }
#endif

  /*bool wait_for_ack(uint32_t timeout = DEFAULT_TIMOUT, bool checksum=false,
   * const char * ack=nullptr);*/

  uint32_t getCommandNumber(String& response);

  uint8_t getErrorNum() { Guard guard(*this); return _error; }

  void setErrorNum(uint8_t error) { Guard guard(*this); _error = error; }

  uint8_t getStatus() { Guard guard(*this); return _step; }

  size_t totalSize() { Guard guard(*this); return _totalSize; }
  size_t processedSize() { Guard guard(*this); return _processedSize; }
  uint8_t getFSType() { Guard guard(*this); return _fsType; }
  const char* fileName() {
    if (_fileName.length() == 0) {
      return nullptr;
    }
    return _fileName.c_str();
  }
  bool processScript(const char* line, ESP3DAuthenticationLevel auth_type =
                                           ESP3DAuthenticationLevel::admin);
  bool processFile(const char* filename, ESP3DAuthenticationLevel auth_type =
                                             ESP3DAuthenticationLevel::admin);
  bool abort();
  bool pause();
  bool resume();
  void startStream();
  void readNextCommand();
  void endStream();
  void processCommand();
  bool isCommand();
  bool isAckNeeded();
  bool isAck(String& line);

 private:
#if defined(ARDUINO_ARCH_ESP32)
  struct RxPacket {
    uint32_t generation;
    uint32_t checksum;
    ESP3DClientType origin;
    uint8_t size;
    uint8_t data[64];
  };
  enum class Control : uint8_t { Pause, Resume, Cancel };
  SemaphoreHandle_t _hostMutex = nullptr;
  SemaphoreHandle_t _ingressMutex = nullptr;
  SemaphoreHandle_t _workerStopped = nullptr;
  QueueHandle_t _workerRx = nullptr, _workerUI = nullptr, _workerControls = nullptr;
  TaskHandle_t _workerTask = nullptr;
  std::atomic<bool> _workerActive{false}, _workerSDLease{false};
  std::atomic<bool> _workerShutdown{false}, _workerEmergency{false}, _workerOverflow{false};
  bool _background = false, _workerStart = false, _workerPause = false;
  // Access only under ingress mutex; packet generations reject stale callbacks.
  bool _capture = false;
  uint32_t _generation = 0;
  ESP3DClientType _captureOrigin = ESP3DClientType::no_client;
  // Ingress-only writes, protected by _ingressMutex. Frozen when capture ends.
  uint8_t _rxTail[256]{};
  size_t _rxTailNext = 0, _rxTailSize = 0;
  uint32_t _rxBytes = 0, _rxPackets = 0, _rxQueueOverflows = 0;
  uint32_t _uartErrors[6]{};  // Arduino hardwareSerial_error_t values 1..5.
  uint32_t _packetMismatches = 0;  // Worker/host lock.
  static uint32_t packetChecksum(const RxPacket& packet);
  int _workerCore = -1;
  uint8_t _uiLine[256];
  size_t _uiSize = 0;
  bool _uiDiscard = false;
  bool beginWorker();
  void stopWorker();
  void startWorker(bool paused);
  void setCapture(bool enabled);
  void notifyWorker();
  bool controlWorker(Control control);
  void publishWorkerUI();
  void queueWorkerUI(const RxPacket& packet);
  static void workerEntry(void* context);
#endif
  static bool sendMarlin(void* context, const char* data, size_t size);
  void handleMarlin();
  int readMarlinCommand();
  MarlinStream _marlin;
  bool _useMarlin = false;
  ESP3DClientType _marlinOutput = ESP3DClientType::no_client;
  bool _marlinPending = false;
  bool _marlinEOF = false;
  bool _lineComment = false;
  bool _parenComment = false;
  uint8_t _readBuffer[512];
  size_t _readSize = 0, _readPosition = 0;
  char _line[MarlinStream::kLineSize + 1];
  size_t _lineSize = 0;
  size_t _pendingEnd = 0;
  ESP3DScriptFIFO _scriptList;
  ESP3DAuthenticationLevel _auth;
  uint8_t _buffer[ESP_HOST_BUFFER_SIZE + 1];
  size_t _bufferSize;
  size_t _totalSize;
  size_t _processedSize;
  uint32_t _commandNumber;
  uint32_t _needCommandNumber;
  uint8_t _error;
  uint8_t _step;
  uint8_t _nextStep;
  uint32_t _currentPosition;
  String _fileName;
  String _script;
  uint8_t _fsType;
  String _currentCommand;
  String _response;
  ESP3DAuthenticationLevel _auth_type;
  uint64_t _startTimeOut;
  bool _needRelease;
};

extern GcodeHost esp3d_gcode_host;

#endif  //_GCODE_HOST_H
