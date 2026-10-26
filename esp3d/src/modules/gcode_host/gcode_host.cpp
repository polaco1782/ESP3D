/*
  gcode_host.cpp -  gcode host functions class

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
// #define ESP_LOG_FEATURE LOG_OUTPUT_SERIAL0
#include "../../include/esp3d_config.h"
#if defined(GCODE_HOST_FEATURE)
#include "../../core/esp3d_commands.h"
#include "../../core/esp3d_settings.h"
#include "gcode_host.h"
#if defined(ARDUINO_ARCH_ESP32)
#include "../serial/serial_service.h"
#endif

#if defined(FILESYSTEM_FEATURE)
#include "../filesystem/esp_filesystem.h"
ESP_File FSfileHandle;
#endif  // FILESYSTEM_FEATURE
#if defined(SD_DEVICE)
#include "../filesystem/esp_sd.h"
ESP_SDFile SDfileHandle;
#endif  // FILESYSTEM_FEATURE

#define ESP_HOST_TIMEOUT 16000
#define MAX_TRY_2_SEND 5

GcodeHost esp3d_gcode_host;

GcodeHost::GcodeHost() : _step(HOST_NO_STREAM), _fsType(TYPE_SCRIPT_STREAM), _needRelease(false) { end(); }

GcodeHost::~GcodeHost() {
  end();
#if defined(ARDUINO_ARCH_ESP32)
  if (_workerRx) vQueueDelete(_workerRx);
  if (_workerUI) vQueueDelete(_workerUI);
  if (_workerControls) vQueueDelete(_workerControls);
  if (_workerStopped) vSemaphoreDelete(_workerStopped);
  if (_ingressMutex) vSemaphoreDelete(_ingressMutex);
  if (_hostMutex) vSemaphoreDelete(_hostMutex);
#endif
}

bool GcodeHost::begin() {
  end();
#if defined(ARDUINO_ARCH_ESP32)
  return beginWorker();
#else
  return true;
#endif
}

bool GcodeHost::dispatch(ESP3DMessage* message) { 
  Guard guard(*this);
  if (!message || _step == HOST_NO_STREAM) {
    return false;
  }
#if defined(ARDUINO_ARCH_ESP32)
  // UI copies (including old main-loop FIFO entries) must never ACK twice.
  if (_background) return false;
#endif
  if (message->origin != (_useMarlin ? _marlinOutput : esp3d_commands.getOutputClient())) return false;
  if (message->size > 0 && message->data) {
    push(message->data, message->size);
    esp3d_message_manager.deleteMsg(message);
    return true;
  }
  return false;
 }

void GcodeHost::end() {
#if defined(ARDUINO_ARCH_ESP32)
  stopWorker();
#endif
  Guard guard(*this);
  if (_step != HOST_NO_STREAM) { _error = ERROR_STREAM_ABORTED; endStream(); }
  _commandNumber = 0;
  _error = ERROR_NO_ERROR;
  _step = HOST_NO_STREAM;
  _currentPosition = 0;
  _bufferSize = 0;
  _totalSize = 0;
  _processedSize = 0;
  _needRelease = false;
#if defined(AUTHENTICATION_FEATURE)
  _auth = ESP3DAuthenticationLevel::guest;
#else
  _auth = ESP3DAuthenticationLevel::admin;
#endif  // AUTHENTICATION_FEATURE
}

bool GcodeHost::push(const uint8_t *sbuf, size_t len) {
  Guard guard(*this);
  if (_step == HOST_NO_STREAM) {
    return false;
  }
#if defined(ARDUINO_ARCH_ESP32)
  if (_background) return false;
#endif
  if (_useMarlin) {
    _marlin.receive(sbuf, len, millis());
    return true;
  }
  esp3d_log("Push got %d bytes", len);
  for (size_t i = 0; i < len; i++) {
    // it is a line process it
    if (sbuf[i] == '\n') {
      flush();
    } else {
      // fill buffer until it is full
      if (_bufferSize < ESP_HOST_BUFFER_SIZE) {
        _buffer[_bufferSize++] = sbuf[i];
      } else {
        // buffer is full flush it
        flush();
        _buffer[_bufferSize++] = sbuf[i];
      }
      _buffer[_bufferSize] = 0;
    }
  }
  flush();

  return true;
}

bool GcodeHost::isAck(String &line) {
  if (line.indexOf("ok") != -1) {
    esp3d_log("got ok");
    return true;
  }
  if (ESP3DSettings::GetFirmwareTarget() == SMOOTHIEWARE) {
    if (line.indexOf("smoothie out") != -1) {
      esp3d_log("got smoothie out");
      return true;
    }
  }
  return false;
}

void GcodeHost::flush() {
  // analyze buffer and do action if needed
  // look for \n, ok , error, ack
  // then clean buffer accordingly
  if (_bufferSize == 0) {
    return;
  }
  _response = (const char *)_buffer;
  esp3d_log("Stream got the response: %s", _response.c_str());
  _response.toLowerCase();
  if (isAck(_response)) {
    // check if we have proper ok response
    // like if numbering is enabled
    if (_step == HOST_WAIT4_ACK) {
      _step = HOST_READ_LINE;
    } else {
      esp3d_log("Got ok but out of the query");
    }
  } else {
    if (_response.indexOf("error") != -1) {
      esp3d_log_e("Got error");
      _step = HOST_ERROR_STREAM;
    }
  }
  // What is have processing to do
  // what if have resend
  _bufferSize = 0;
}

void GcodeHost::startStream() {
  Guard guard(*this);
  const bool pauseOnStart = _nextStep == HOST_PAUSE_STREAM;
  _needRelease = false;
  if (_fsType == TYPE_SCRIPT_STREAM) {
    _totalSize = _script.length();
    esp3d_log("Script line %s opened, size is %d", _script.c_str(), _totalSize);
  }
#if defined(FILESYSTEM_FEATURE)
  if (_fsType == TYPE_FS_STREAM) {
    if (ESP_FileSystem::exists(_fileName.c_str())) {
      FSfileHandle = ESP_FileSystem::open(_fileName.c_str());
    }
    if (FSfileHandle.isOpen()) {
      _totalSize = FSfileHandle.size();
      esp3d_log("File %s opened, size is %d", _fileName.c_str(), _totalSize);
    } else {
      _error = ERROR_FILE_NOT_FOUND;
      _step = HOST_ERROR_STREAM;
      esp3d_log_e("File not found: %s", _fileName.c_str());
      return;
    }
  }
#endif  // FILESYSTEM_FEATURE
#if defined(SD_DEVICE)
  if (_fsType == TYPE_SD_STREAM) {
    if (!ESP_SD::accessFS()) {
      _error = ERROR_FILE_NOT_FOUND;
      _step = HOST_ERROR_STREAM;
      _needRelease = false;
      esp3d_log_e("File not found: %s", _fileName.c_str());
      return;
    }
    _needRelease = true;
    if (ESP_SD::exists(_fileName.c_str())) {
      SDfileHandle = ESP_SD::open(_fileName.c_str());
    }
    if (SDfileHandle.isOpen()) {
      _totalSize = SDfileHandle.size();
      esp3d_log("File %s opened, size is %d", _fileName.c_str(), _totalSize);
    } else {
      _error = ERROR_FILE_NOT_FOUND;
      _step = HOST_ERROR_STREAM;
      esp3d_log_e("File not found: %s", _fileName.c_str());
      return;
    }
  }
#endif  // SD_DEVICE
  _currentPosition = 0;
  _response = "";
  _currentPosition = 0;
  _error = ERROR_NO_ERROR;
  _step = HOST_READ_LINE;
  _nextStep = HOST_READ_LINE;
  _processedSize = 0;
  if (_useMarlin) {
    _readSize = _readPosition = _lineSize = 0;
    _marlinPending = _marlinEOF = _lineComment = _parenComment = false;
#if defined(ARDUINO_ARCH_ESP32)
    if (_background) {
      startWorker(pauseOnStart);
      return;
    }
#endif
    if (!_marlin.begin(sendMarlin, this, millis())) {
      _error = static_cast<uint8_t>(_marlin.error());
      _step = HOST_ERROR_STREAM;
    } else {
      if (pauseOnStart) _marlin.pause();
      _step = pauseOnStart ? HOST_PAUSING_STREAM : HOST_WAIT4_ACK;
    }
  }
}

bool GcodeHost::ownsPrinter() const {
  Guard guard(*this);
  return (_useMarlin && _step != HOST_NO_STREAM) ||
         _marlin.state() == MarlinStream::State::Fault;
}

void GcodeHost::emergencyStop() {
#if defined(ARDUINO_ARCH_ESP32)
  if (_workerActive.load()) {
    _workerEmergency.store(true);
    notifyWorker();
    return;
  }
#endif
  Guard guard(*this);
  // A job can own TX before its first handle() has opened the source.
  if (_step == HOST_START_STREAM || _marlin.state() == MarlinStream::State::Fault)
    sendMarlin(this, "\nM112\n", 6);
  _marlin.fail(MarlinStream::Error::Printer, "emergency stop requested");
}

bool GcodeHost::queueTemperature(const char* command) {
  Guard guard(*this);
  if (!_useMarlin || _step == HOST_NO_STREAM || _step == HOST_START_STREAM ||
      _step == HOST_ABORT_STREAM || !_marlin.queueTemperature(command)) return false;
#if defined(ARDUINO_ARCH_ESP32)
  notifyWorker();
#endif
  return true;
}

bool GcodeHost::sendMarlin(void* context, const char* data, size_t size) {
  GcodeHost* host = static_cast<GcodeHost*>(context);
#if defined(ARDUINO_ARCH_ESP32)
  // Worker TX never enters HTTP/WebSocket routing or allocates ESP3D messages.
  if (host->_background) {
    ESP3DSerialService* serial = nullptr;
#if COMMUNICATION_PROTOCOL == RAW_SERIAL
    if (host->_marlinOutput == ESP3DClientType::serial) serial = &esp3d_serial_service;
#endif
#if defined(ESP_SERIAL_BRIDGE_OUTPUT)
    if (host->_marlinOutput == ESP3DClientType::serial_bridge) serial = &serial_bridge_service;
#endif
    return serial && serial->writeBytes((const uint8_t*)data, size) == size;
  }
#endif
  return esp3d_commands.dispatch((uint8_t*)data, size,
      host->_marlinOutput, no_id, ESP3DMessageType::unique,
      ESP3DClientType::stream, host->_auth);
}

// A bounded read-ahead buffer avoids one SD operation per character. Preserve
// partial lines between scheduler calls, including arbitrarily long comments.
int GcodeHost::readMarlinCommand() {
  for (size_t work = 0; work < sizeof(_readBuffer); ++work) {
    int c = -1;
    if (_currentPosition < _totalSize) {
      if (_fsType == TYPE_SCRIPT_STREAM) {
        c = _script[_currentPosition];
        if (c == ';') c = '\n';
      } else {
        if (_readPosition == _readSize) {
          size_t remaining = _totalSize - _currentPosition;
          size_t requested = remaining < sizeof(_readBuffer) ? remaining : sizeof(_readBuffer);
          _readSize = 0;
#if defined(FILESYSTEM_FEATURE)
          if (_fsType == TYPE_FS_STREAM) _readSize = FSfileHandle.read(_readBuffer, requested);
#endif
#if defined(SD_DEVICE)
          if (_fsType == TYPE_SD_STREAM) _readSize = SDfileHandle.read(_readBuffer, requested);
#endif
          _readPosition = 0;
          if (!_readSize || _readSize > requested) {
            _marlin.fail(MarlinStream::Error::Read, "file read failed before expected EOF");
            return -2;
          }
        }
        c = _readBuffer[_readPosition++];
      }
      ++_currentPosition;
    }
    if (c == '\n' || c == -1) {
      if (_parenComment) {
        _marlin.fail(MarlinStream::Error::Input, "unterminated parenthesis comment");
        return -2;
      }
      _lineComment = false;
      _line[_lineSize] = 0;
      _currentCommand = _line;
      _currentCommand.trim();
      _lineSize = 0;
      _pendingEnd = _currentPosition;
      if (c == -1) _marlinEOF = true;
      if (_currentCommand.length()) return 1;
      if (c == -1) return -1;
    } else if (!_lineComment) {
      if (c == ';' && !_parenComment) _lineComment = true;
      else if (c == '(' && !_parenComment) {
        _parenComment = true;
        // Keep adjacent tokens separated after stripping a comment.
        if (_lineSize < MarlinStream::kLineSize) _line[_lineSize++] = ' ';
      } else if (c == ')' && _parenComment) _parenComment = false;
      else if (!_parenComment && c != '\r') {
        if (_lineSize == MarlinStream::kLineSize || (c < 32 && c != '\t') || c > 126) {
          _marlin.fail(MarlinStream::Error::Input, "invalid or overlong input line");
          return -2;
        }
        _line[_lineSize++] = c == '\t' ? ' ' : c;
      }
    }
  }
  return 0;
}

String GcodeHost::diagnostics() {
  Guard guard(*this);
  // Hex encodes untrusted UART bytes, including control bytes and quotes.
  const auto hex = [](const uint8_t* bytes, size_t count) {
    static const char digits[] = "0123456789ABCDEF";
    String result;
    result.reserve(count * 2);
    for (size_t i = 0; i < count; ++i) {
      result += digits[bytes[i] >> 4];
      result += digits[bytes[i] & 15];
    }
    return result;
  };
  String result = "{\"version\":1,\"error\":" + String((uint8_t)_marlin.error()) +
      ",\"reason\":\"" + _marlin.reason() + "\",\"last_ack\":" + String(_marlin.lastAck()) +
      ",\"expected_ack\":" + String(_marlin.expectedAck()) +
      ",\"pending\":" + String(_marlin.pending()) + ",\"replies\":[";
  for (size_t i = 0; i < _marlin.replyCount(); ++i) {
    const auto& reply = _marlin.reply(i);
    if (i) result += ',';
    result += "{\"length\":" + String(reply.length) + ",\"hex\":\"" +
              hex(reply.data, reply.size) + "\"}";
  }
  result += ']';
#if defined(ARDUINO_ARCH_ESP32)
  if (_ingressMutex && _background) {
    uint8_t tail[sizeof(_rxTail)];
    uint32_t errors[6], bytes, packets, overflows;
    size_t size;
    xSemaphoreTake(_ingressMutex, portMAX_DELAY);
    size = _rxTailSize;
    for (size_t i = 0; i < size; ++i)
      tail[i] = _rxTail[(_rxTailNext + sizeof(_rxTail) - size + i) % sizeof(_rxTail)];
    memcpy(errors, _uartErrors, sizeof(errors));
    bytes = _rxBytes;
    packets = _rxPackets;
    overflows = _rxQueueOverflows;
    xSemaphoreGive(_ingressMutex);
    result += ",\"worker_core\":" + String(_workerCore) +
        ",\"rx_bytes\":" + String(bytes) + ",\"rx_packets\":" + String(packets) +
        ",\"queue_overflows\":" + String(overflows) +
        ",\"packet_mismatches\":" + String(_packetMismatches) +
        ",\"uart_errors_since_start\":{\"break\":" + String(errors[1]) +
        ",\"buffer_full\":" + String(errors[2]) + ",\"fifo_overflow\":" + String(errors[3]) +
        ",\"frame\":" + String(errors[4]) + ",\"parity\":" + String(errors[5]) +
        "},\"rx_tail_hex\":\"" + hex(tail, size) + "\"";
  }
#endif
  result += '}';
  return result;
}

void GcodeHost::handleMarlin() {
  // Configuration belongs to the main task; it checks changes in handle().
#if defined(ARDUINO_ARCH_ESP32)
  if (!_background)
#endif
  if (esp3d_commands.getOutputClient() != _marlinOutput)
    _marlin.fail(MarlinStream::Error::Write, "printer output changed during stream");
#if defined(SD_DEVICE)
  // The SD ownership watchdog also needs service during long heating/pauses.
  if (_fsType == TYPE_SD_STREAM && _needRelease) ESP_SD::touchWatchdog();
#endif
  _marlin.tick(millis());
  for (size_t work = 0; work < MarlinStream::kSlots && _marlin.ready(); ++work) {
    if (!_marlinPending) {
      if (_marlinEOF) { _marlin.finish(); break; }
      int result = readMarlinCommand();
      if (result == 0 || result == -2) break;
      if (result == -1) { _marlin.finish(); break; }
      _marlinPending = true;
    }
    if (!_marlin.enqueue(_currentCommand.c_str(), _pendingEnd, millis())) break;
    _marlinPending = false;
  }
  _processedSize = _marlin.acknowledged();
  switch (_marlin.state()) {
    case MarlinStream::State::Paused: _step = HOST_PAUSE_STREAM; break;
    case MarlinStream::State::Pausing: _step = HOST_PAUSING_STREAM; break;
    case MarlinStream::State::Cancelling: _step = HOST_ABORT_STREAM; break;
    case MarlinStream::State::Complete:
      _processedSize = _totalSize;
      _step = HOST_STOP_STREAM;
      break;
    case MarlinStream::State::Cancelled:
      _error = ERROR_STREAM_ABORTED;
      _step = HOST_STOP_STREAM;
      break;
    case MarlinStream::State::Fault:
      _error = static_cast<uint8_t>(_marlin.error());
      _step = HOST_ERROR_STREAM;
      break;
    default: _step = HOST_WAIT4_ACK; break;
  }
}

void GcodeHost::endStream() {
  Guard guard(*this);
  esp3d_log("Ending Stream");
#if defined(FILESYSTEM_FEATURE)
  if (_fsType == TYPE_FS_STREAM) {
    if (FSfileHandle.isOpen()) {
      FSfileHandle.close();
    }
  }
#endif  // FILESYSTEM_FEATURE
#if defined(SD_DEVICE)
  if (_fsType == TYPE_SD_STREAM) {
    if (SDfileHandle.isOpen()) {
      SDfileHandle.close();
    }
    if (_needRelease) {
      ESP_SD::releaseFS();
      _needRelease = false;
    }
  }
#endif  // SD_DEVICE
#if defined(ARDUINO_ARCH_ESP32)
  _workerSDLease.store(false);
#endif
  _step = HOST_NO_STREAM;
  if (_error != ERROR_NO_ERROR) _scriptList.clear();
  if (!_scriptList.isEmpty()){
    ScriptEntry scr = _scriptList.pop();
    processScript(scr.script.c_str(),scr.auth_type);
  }
  //TODO: do same for files
}

void GcodeHost::readNextCommand() {
  _currentCommand = "";
  _step = HOST_PROCESS_LINE;
  if (_fsType == TYPE_SCRIPT_STREAM) {
    esp3d_log("Reading next command from script");
    if (_currentPosition < _script.length()) {
      if (_script.indexOf(';', _currentPosition) != -1) {
        _currentCommand = _script.substring(
            _currentPosition, _script.indexOf(';', _currentPosition));
        _currentPosition = _script.indexOf(';', _currentPosition) + 1;
      } else {
        _currentCommand = _script.substring(_currentPosition);
        _currentPosition = _script.length();
      }
      _processedSize = _currentPosition;
      esp3d_log("Command is %s", _currentCommand.c_str());
    } else {
      _step = HOST_STOP_STREAM;
    }
  }
#if defined(FILESYSTEM_FEATURE)
  if (_fsType == TYPE_FS_STREAM) {
    bool processing = true;
    while (processing) {
      // to handle file without endline
      int c = FSfileHandle.read();
      if (c == -1) {
        processing = false;
      } else {
        _processedSize++;
        _currentPosition++;
        if (!((char)c == '\n')) {
          _currentCommand += (char)c;
        } else {
          processing = false;
        }
      }
    }
    if (_currentCommand.length() == 0) {
      if (FSfileHandle.available()) {
        _step = HOST_READ_LINE;
      } else {
        _step = HOST_STOP_STREAM;
      }
    }
  }
#endif  // FILESYSTEM_FEATURE
#if defined(SD_DEVICE)
  if (_fsType == TYPE_SD_STREAM) {
    bool processing = true;
    while (processing) {
      // to handle file without endline
      int c = SDfileHandle.read();
      if (c == -1) {
        processing = false;
      } else {
        _processedSize++;
        _currentPosition++;
        if ((char)c != '\n' ) {
          _currentCommand += (char)c;
        } else {
          processing = false;
        }
      }
    }
    if (_currentCommand.length() == 0) {
      if (SDfileHandle.available()) {
        _step = HOST_READ_LINE;
      } else {
        _step = HOST_STOP_STREAM;
      }
    }
  }
#endif  // SD_DEVICE
}

bool GcodeHost::isCommand() {
  // clean the command
  if (_currentCommand.indexOf(';') != -1) {
    _currentCommand =
        _currentCommand.substring(0, _currentCommand.indexOf(';'));
  }
  _currentCommand.trim();
  if (_currentCommand.length() == 0 || _currentCommand.startsWith(";")) {
    return false;
  }
  return true;
}
bool GcodeHost::isAckNeeded() {
  // TODO: what command do not need for ack ?
  return true;
}
void GcodeHost::processCommand() {
  if (!isCommand()) {
    esp3d_log_e("Command %s is not valid", _currentCommand.c_str());
    _step = HOST_READ_LINE;
  } else {
    esp3d_log("Command %s is valid", _currentCommand.c_str());
    if (!_currentCommand.endsWith("\n")) {
      _currentCommand += "\n";
    }
    bool isESPcmd = esp3d_commands.is_esp_command(
        (uint8_t *)_currentCommand.c_str(), _currentCommand.length());
    if (isESPcmd) {
      ESP3DMessage *msg = esp3d_message_manager.newMsg(
          ESP3DClientType::no_client, esp3d_commands.getOutputClient(),
          (uint8_t *)_currentCommand.c_str(), _currentCommand.length(), _auth);
      if (msg) {
        // process command
        esp3d_commands.process(msg);
        esp3d_log("Command is ESP command: %s", _currentCommand.c_str());
        _step = HOST_READ_LINE;
      } else {
        esp3d_log_e("Cannot create message");
        _step = HOST_ERROR_STREAM;
      }

    } else {
      ESP3DMessage *msg = esp3d_message_manager.newMsg(
          ESP3DClientType::stream, esp3d_commands.getOutputClient(),
          (uint8_t *)_currentCommand.c_str(), _currentCommand.length(), _auth);
      if (msg) {
        // process command
        esp3d_commands.process(msg);
        esp3d_log("Command is GCODE command: %s", _currentCommand.c_str());
        _startTimeOut = millis();
        if (isAckNeeded()) {
          _step = HOST_WAIT4_ACK;
          esp3d_log("Command wait for ack");
        } else {
          _step = HOST_READ_LINE;
        }
      } else {
        esp3d_log_e("Cannot create message");
        _step = HOST_ERROR_STREAM;
      }
    }
  }
}

void GcodeHost::handle() {
#if defined(ARDUINO_ARCH_ESP32)
  publishWorkerUI();  // Main task only; never hold the host mutex across network IO.
#endif
  Guard guard(*this);
#if defined(ARDUINO_ARCH_ESP32)
  if (_workerActive.load()) {
    if (esp3d_commands.getOutputClient() != _marlinOutput) {
      _workerEmergency.store(true);
      notifyWorker();
    }
    return;
  }
#endif
  if (_step == HOST_NO_STREAM) {
    return;
  }
  if (_useMarlin && _step != HOST_START_STREAM &&
      _step != HOST_STOP_STREAM && _step != HOST_ERROR_STREAM) {
    handleMarlin();
    return;
  }
  switch (_step) {
    case HOST_START_STREAM:
      startStream();
      break;
    case HOST_READ_LINE:
      if (_nextStep == HOST_PAUSE_STREAM) {
        _step = HOST_PAUSE_STREAM;
        _nextStep = HOST_READ_LINE;
      } else {
        readNextCommand();
      }
      break;
    case HOST_PROCESS_LINE:
      processCommand();
      break;
    case HOST_WAIT4_ACK:
      if (millis() - _startTimeOut > ESP_HOST_TIMEOUT) {
        esp3d_log("Timeout waiting for ack");
        _error = ERROR_TIME_OUT;
        _step = HOST_ERROR_STREAM;
      }
      break;
    case HOST_PAUSE_STREAM:
      // TODO pause stream
      break;
    case HOST_RESUME_STREAM:
      // Any extra action to resume stream?
      _step = HOST_READ_LINE;
      break;
    case HOST_STOP_STREAM:
      endStream();
      break;
    case HOST_ERROR_STREAM: {
      String Error;
      if (_error == ERROR_NO_ERROR) {
        // TODO check _response to put right error
        _error = ERROR_UNKNOW;
      }
      esp3d_log_e("Error %d", _error);
      Error = "error: stream failed: " + String(_error);
      if (_useMarlin) Error += String(" - ") + _marlin.reason();
      Error += "\n";
      esp3d_commands.dispatch(Error.c_str(), ESP3DClientType::all_clients,
                              no_id, ESP3DMessageType::unique,
                              ESP3DClientType::system,
                              ESP3DAuthenticationLevel::admin);
      _step = HOST_STOP_STREAM;
    } break;
    default:  // Not handled step
      esp3d_log("Not handled step %d", _step);
      break;
  }
}

bool GcodeHost::abort() {
  Guard guard(*this);
  if (_step == HOST_NO_STREAM) {
    return false;
  }
#if defined(ARDUINO_ARCH_ESP32)
  if (_workerActive.load()) {
    if (_step == HOST_ABORT_STREAM) return true;
    if (!controlWorker(Control::Cancel)) return false;
    _step = HOST_ABORT_STREAM;
    return true;
  }
#endif
  if (_useMarlin) {
    _scriptList.clear();
    if (_step == HOST_START_STREAM) {
      _error = ERROR_STREAM_ABORTED;
      _step = HOST_NO_STREAM;
      return true;
    }
    bool accepted = _marlin.cancel(millis());
    if (accepted) _step = HOST_ABORT_STREAM;
    return accepted;
  }
  esp3d_log("Aborting script");
  // TODO: what to do in addition ?
  _error = ERROR_STREAM_ABORTED;
  // we do not use step to do faster abort
  endStream();
  return true;
}

bool GcodeHost::pause() {
  Guard guard(*this);
  if (_step == HOST_NO_STREAM) {
    return false;
  }
#if defined(ARDUINO_ARCH_ESP32)
  if (_workerActive.load()) {
    if (_step == HOST_PAUSE_STREAM || _step == HOST_PAUSING_STREAM ||
        _step == HOST_ABORT_STREAM) return false;
    if (!controlWorker(Control::Pause)) return false;
    _step = HOST_PAUSING_STREAM;
    return true;
  }
#endif
  if (_useMarlin) {
    if (_step == HOST_START_STREAM) { _nextStep = HOST_PAUSE_STREAM; return true; }
    bool accepted = _marlin.pause();
    if (accepted) _step = HOST_PAUSING_STREAM;
    return accepted;
  }
  _nextStep = HOST_PAUSE_STREAM;
  return true;
}

bool GcodeHost::resume() {
  Guard guard(*this);
  if (_step != HOST_PAUSE_STREAM) {
    return false;
  }
#if defined(ARDUINO_ARCH_ESP32)
  if (_workerActive.load()) {
    if (!controlWorker(Control::Resume)) return false;
    _step = HOST_WAIT4_ACK;
    return true;
  }
#endif
  if (_useMarlin) {
    if (!_marlin.resume()) return false;
    _step = HOST_WAIT4_ACK;
    return true;
  }
  _step = _nextStep;
  return true;
}

uint8_t GcodeHost::Checksum(const char *command, uint32_t commandSize) {
  uint8_t checksum_val = 0;
  if (command == NULL) {
    return 0;
  }
  for (uint32_t i = 0; i < commandSize; i++) {
    checksum_val = checksum_val ^ ((uint8_t)command[i]);
  }
  return checksum_val;
}

String GcodeHost::CheckSumCommand(const char *command, uint32_t commandnb) {
  String commandchecksum = "N" + String((uint32_t)commandnb) + " " + command;
  uint8_t crc = Checksum(commandchecksum.c_str(), commandchecksum.length());
  commandchecksum += "*" + String(crc);
  return commandchecksum;
}

bool GcodeHost::resetCommandNumbering() {
  String resetcmd = "M110 N0";
  if (ESP3DSettings::GetFirmwareTarget() == SMOOTHIEWARE) {
    resetcmd = "N0 M110\n";
  } else {
    resetcmd = "M110 N0\n";
  }
  _commandNumber = 1;

  return esp3d_commands.dispatch(
      resetcmd.c_str(), esp3d_commands.getOutputClient(), no_id,
      ESP3DMessageType::unique, ESP3DClientType::stream,
      ESP3DAuthenticationLevel::admin);
}

uint32_t GcodeHost::getCommandNumber(String &response) {
  uint32_t l = 0;
  String sresend = "Resend:";
  if (ESP3DSettings::GetFirmwareTarget() == SMOOTHIEWARE) {
    sresend = "rs N";
  }
  int pos = response.indexOf(sresend);
  if (pos == -1) {
    esp3d_log_e("Cannot find label %d", _error);
    return -1;
  }
  pos += sresend.length();
  int pos2 = response.indexOf("\n", pos);
  String snum = response.substring(pos, pos2);
  // remove potential unwished char
  snum.replace("\r", "");
  l = snum.toInt();
  esp3d_log_e("Command number to resend is %s", String((uint32_t)l).c_str());
  return l;
}

bool GcodeHost::processScript(const char *line,
                              ESP3DAuthenticationLevel auth_type) {
  Guard guard(*this);
  if (!line || ownsPrinter()) return false;
  if (_step != HOST_NO_STREAM) {
      esp3d_log("Streaming already in progress, put to queue");
      String s = line;
      _scriptList.push(line, auth_type);
      return true;
  }
  _script = line;
  _script.trim();
  esp3d_log("Processing script: %s", _script.c_str());
  if (_script.length() == 0) {
    esp3d_log_e("No script to process");
    return false;
  }

  _fsType = TYPE_SCRIPT_STREAM;
  _nextStep = HOST_READ_LINE;
  _step = HOST_START_STREAM;
  _auth = _auth_type = auth_type;
  _useMarlin = ESP3DSettings::GetFirmwareTarget() == MARLIN ||
               ESP3DSettings::GetFirmwareTarget() == MARLIN_EMBEDDED;
  _marlinOutput = esp3d_commands.getOutputClient();
#if defined(ARDUINO_ARCH_ESP32)
  _background = _useMarlin && _fsType != TYPE_FS_STREAM && (_marlinOutput == ESP3DClientType::serial ||
                              _marlinOutput == ESP3DClientType::serial_bridge);
  if (_background && !_workerTask) {
    _step = HOST_NO_STREAM;
    _error = ERROR_MEMORY_PROBLEM;
    return false;
  }
#endif
  if (_useMarlin && _marlinOutput != ESP3DClientType::serial &&
      _marlinOutput != ESP3DClientType::serial_bridge &&
      _marlinOutput != ESP3DClientType::usb_serial) {
    _step = HOST_NO_STREAM;
    _error = ERROR_CANNOT_SEND_DATA;
    return false;
  }
  return true;
}

bool GcodeHost::processFile(const char *filename,
                            ESP3DAuthenticationLevel auth_type) {
  Guard guard(*this);
  if (!filename || !filename[0] || _step != HOST_NO_STREAM || ownsPrinter()) return false;
  bool target_found = false;
  // sanity check
  _fileName = filename[0] != '/' ? "/" : "";
  _fileName += filename;
  _fileName.trim();
  esp3d_log("Processing file: %s", filename);
  if (_fileName.length() == 0) {
    esp3d_log_e("No file to process");
    return false;
  }
  if (_step != HOST_NO_STREAM) {
    esp3d_log_e("Streaming already in progress");
    return false;
  }
  // TODO UD = USB DISK
#if defined(SD_DEVICE)
  if (_fileName.startsWith(ESP_SD_FS_HEADER)) {
    esp3d_log("Processing SD file");
    target_found = true;
    _fileName =
        _fileName.substring(strlen(ESP_SD_FS_HEADER), _fileName.length());
    _fsType = TYPE_SD_STREAM;
  }
#endif  // SD_DEVICE
#if defined(FILESYSTEM_FEATURE)
  if (!target_found && _fileName.startsWith(ESP_FLASH_FS_HEADER)) {
    target_found = true;
    _fileName =
        _fileName.substring(strlen(ESP_FLASH_FS_HEADER), _fileName.length());
    esp3d_log("Processing /FS file %s", _fileName.c_str());
    _fsType = TYPE_FS_STREAM;
  }
  // if no header it is also an FS file
  if (!target_found) {
    esp3d_log("Processing FS file %s", _fileName.c_str());
    _fsType = TYPE_FS_STREAM;
    target_found = true;
  }
#endif  // FILESYSTEM_FEATURE
  // it is not a file so it is a script
  if (!target_found) {
    target_found = true;
    _fsType = TYPE_SCRIPT_STREAM;
    // remove the /
    _script = &_fileName[1];
    esp3d_log("Processing Script file %s", _script.c_str());
    _fileName = "";
  }
  _nextStep = HOST_READ_LINE;
  _step = HOST_START_STREAM;
  _auth = _auth_type = auth_type;
  _useMarlin = ESP3DSettings::GetFirmwareTarget() == MARLIN ||
               ESP3DSettings::GetFirmwareTarget() == MARLIN_EMBEDDED;
  _marlinOutput = esp3d_commands.getOutputClient();
#if defined(ARDUINO_ARCH_ESP32)
  _background = _useMarlin && _fsType != TYPE_FS_STREAM && (_marlinOutput == ESP3DClientType::serial ||
                              _marlinOutput == ESP3DClientType::serial_bridge);
  if (_background && !_workerTask) {
    _step = HOST_NO_STREAM;
    _error = ERROR_MEMORY_PROBLEM;
    return false;
  }
#endif
  if (_useMarlin && _marlinOutput != ESP3DClientType::serial &&
      _marlinOutput != ESP3DClientType::serial_bridge &&
      _marlinOutput != ESP3DClientType::usb_serial) {
    _step = HOST_NO_STREAM;
    _error = ERROR_CANNOT_SEND_DATA;
    return false;
  }
  return true;
}

#endif  // GCODE_HOST_FEATURE
