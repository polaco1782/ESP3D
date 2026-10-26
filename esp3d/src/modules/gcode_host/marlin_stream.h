// SPDX-License-Identifier: LGPL-2.1-or-later
// Buffered Marlin protocol engine. No Arduino/RTOS dependencies: the same code
// runs in firmware and the native fault-injection tests.
#pragma once
#include <stddef.h>
#include <stdint.h>

class MarlinStream {
 public:
  static constexpr size_t kSlots = 8;
  // Include newline; leaves room for the NUL in Marlin's MAX_CMD_SIZE=96.
  static constexpr size_t kLineSize = 96;
  // Conservative for a printer with at least 128 bytes of serial RX storage.
  // B/P in ADVANCED_OK are NOT RX byte credits.
  static constexpr size_t kWindowBytes = 96;
  static constexpr uint32_t kIdleTimeout = 30000;
  static constexpr uint32_t kAckTimeout = 120000;
  static constexpr uint32_t kWaitTimeout = 24UL * 60 * 60 * 1000;
  enum class State {
    Idle,
    Sync,
    Running,
    Pausing,
    Paused,
    Finishing,
    Cancelling,
    Complete,
    Cancelled,
    Fault
  };
  enum class Error : uint8_t {
    None = 0,
    Timeout = 1,
    Write = 2,
    Ack = 4,
    Resend = 6,
    Input = 8,
    Read = 9,
    Printer = 11
  };
  using Send = bool (*)(void*, const char*, size_t);

  MarlinStream();
  bool begin(Send send, void* context, uint32_t now);
  // False means backpressure unless state()==Fault. Never drop the caller's
  // line.
  bool enqueue(const char* command, size_t sourceEnd, uint32_t now);
  // Bounded manual M104/M140 queue. No writes here: tick() sends these with
  // normal numbering, byte credits and resend retention, ahead of new file data.
  bool queueTemperature(const char* command);
  void receive(const uint8_t* bytes, size_t size, uint32_t now);
  void tick(uint32_t now);
  bool pause();
  bool resume();
  bool cancel(uint32_t now);
  void finish();
  void fail(Error error, const char* reason);
  State state() const { return _state; }
  Error error() const { return _error; }
  const char* reason() const { return _reason; }
  uint32_t lastAck() const { return _lastAck; }
  uint32_t expectedAck() const { return _count ? _queue[0].number : _next; }
  // Diagnostic prefixes of the last four complete parser input lines, oldest
  // first. Never interpreted as commands; retained across a latched fault.
  struct Reply { uint8_t data[96]; size_t size; size_t length; };
  size_t replyCount() const { return _replyCount; }
  const Reply& reply(size_t index) const {
    return _replies[(_replyNext + 4 - _replyCount + index) % 4];
  }
  size_t acknowledged() const { return _acknowledged; }
  size_t inFlightBytes() const { return _bytes; }
  size_t pending() const { return _count; }
  bool ready() const {
    return _state == State::Running && !_recovering && !_awaitResend &&
           !_temperatureCount;
  }
  bool terminal() const;

 private:
  struct Entry {
    char wire[kLineSize + 1];
    uint32_t number;
    size_t length;
    size_t sourceEnd;
    bool sent;
    bool wait;
    bool temperature;
  };
  Entry _queue[kSlots];
  char _temperatures[4][48]{};
  size_t _temperatureCount = 0;
  size_t _count, _bytes, _acknowledged;
  uint32_t _next, _lastAck, _activityAt, _progressAt, _recoveryAt;
  bool _hasAck, _recovering, _awaitResend;
  uint8_t _retries, _control;
  char _response[256];
  size_t _responseSize;
  bool _responseOverflow;
  Reply _replies[4]{};
  size_t _replyNext = 0, _replyCount = 0;
  State _state;
  Error _error;
  const char* _reason;
  Send _send;
  void* _context;
  bool append(const char* command, size_t sourceEnd, uint32_t now,
              bool reset = false);
  bool write(Entry& entry, uint32_t now);
  void line(char* response, uint32_t now);
  void ack(uint32_t now);
  void resend(uint32_t number, uint32_t now);
};
