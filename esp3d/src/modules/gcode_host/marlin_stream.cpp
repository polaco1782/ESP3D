// SPDX-License-Identifier: LGPL-2.1-or-later
#include "marlin_stream.h"

#include <ctype.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

namespace {
bool starts(const char* s, const char* prefix) {
  while (*prefix) {
    if (tolower(static_cast<unsigned char>(*s++)) != *prefix++) return false;
  }
  return true;
}
const char* spaces(const char* p) {
  while (*p == ' ' || *p == '\t' || *p == '\r') ++p;
  return p;
}
bool number(const char*& p, uint32_t& n) {
  n = 0;
  if (!isdigit(static_cast<unsigned char>(*p))) return false;
  while (isdigit(static_cast<unsigned char>(*p))) {
    unsigned digit = *p++ - '0';
    if (n > (UINT32_MAX - digit) / 10) return false;
    n = n * 10 + digit;
  }
  return !*p || isspace(static_cast<unsigned char>(*p));
}
// Parse the command itself, never parameter substrings or comments.
int code(const char* command, char family) {
  if (toupper(static_cast<unsigned char>(*command++)) != family) return -1;
  unsigned value = 0;
  if (!isdigit(static_cast<unsigned char>(*command))) return -1;
  while (isdigit(static_cast<unsigned char>(*command))) {
    value = value * 10 + (*command++ - '0');
    if (value > 9999) return -1;
  }
  if (*command && !isspace(static_cast<unsigned char>(*command)) &&
      !isalpha(static_cast<unsigned char>(*command)))
    return -1;
  return static_cast<int>(value);
}
bool blocking(const char* command) {
  const int g = code(command, 'G'), m = code(command, 'M');
  return g == 4 || g == 28 || g == 29 || g == 30 || m == 0 || m == 1 ||
         m == 105 || m == 109 || m == 190 || m == 191 || m == 192 || m == 226 ||
         m == 303 || m == 400 || m == 600 || m == 601 || m == 701 || m == 702;
}
}  // namespace

MarlinStream::MarlinStream()
    : _count(0),
      _bytes(0),
      _acknowledged(0),
      _next(1),
      _lastAck(0),
      _activityAt(0),
      _progressAt(0),
      _recoveryAt(0),
      _hasAck(false),
      _recovering(false),
      _awaitResend(false),
      _retries(0),
      _control(0),
      _responseSize(0),
      _responseOverflow(false),
      _state(State::Idle),
      _error(Error::None),
      _reason(""),
      _send(nullptr),
      _context(nullptr) {}

bool MarlinStream::terminal() const {
  return _state == State::Idle || _state == State::Complete ||
         _state == State::Cancelled || _state == State::Fault;
}

bool MarlinStream::begin(Send send, void* context, uint32_t now) {
  // Protocol/IO faults latch until ESP3D is restarted. CLEAR_ERROR must not
  // enable an unsynchronized print after an emergency stop or lost connection.
  if (!terminal() || _state == State::Fault || !send) return false;
  *this = MarlinStream();
  _send = send;
  _context = context;
  _state = State::Sync;
  return append("M110 N0", 0, now, true);
}

void MarlinStream::fail(Error error, const char* reason) {
  if (_state == State::Fault) return;
  _error = error;
  _reason = reason;
  _state = State::Fault;
  // Never replay a possibly executed move on a timeout. Stop the printer
  // instead. EMERGENCY_PARSER is required for this during heating/planner
  // waits. Leading newline also terminates a potentially partial failed UART
  // write.
  if (_send) _send(_context, "\nM112\n", 6);
}

bool MarlinStream::write(Entry& e, uint32_t now) {
  if (!_send(_context, e.wire, e.length)) {
    fail(Error::Write, "printer write failed; reset printer and ESP3D");
    return false;
  }
  if (_bytes == 0) _activityAt = _progressAt = now;
  _bytes += e.length;
  e.sent = true;
  return true;
}

bool MarlinStream::append(const char* command, size_t end, uint32_t now,
                          bool reset) {
  Entry entry{};
  const size_t length = strlen(command);
  if (length == 0 || length >= kLineSize || _next >= 0x7FFFFFFFUL) {
    fail(Error::Input, "empty/overlong command or line number exhausted");
    return false;
  }
  // Files are unnumbered G/M/T commands. Host commands, line resets, binary,
  // checksums and escaped/quoted syntax cannot enter this protocol session.
  if (!reset) {
    if (code(command, 'G') < 0 && code(command, 'M') < 0 &&
        code(command, 'T') < 0) {
      fail(Error::Input, "expected an unnumbered G, M or T command");
      return false;
    }
    int m = code(command, 'M');
    if (m == 110 || m == 108 || m == 112 || m == 410 || m == 524 || m == 999 ||
        m == 23 || m == 24 || m == 25 || m == 28 || m == 29 || m == 32) {
      fail(Error::Input, "job contains a host/SD/transport control command");
      return false;
    }
    for (size_t i = 0; i < length; ++i) {
      const unsigned char c = command[i];
      if (c < 32 || c > 126 || strchr("*;()\\\"", c)) {
        fail(Error::Input, "unsupported command syntax");
        return false;
      }
    }
  }
  entry.number = reset ? 0 : _next;
  int n = snprintf(entry.wire, sizeof(entry.wire), "N%lu %s",
                   static_cast<unsigned long>(entry.number), command);
  if (n < 0 || static_cast<size_t>(n) >= sizeof(entry.wire)) {
    fail(Error::Input, "numbered command exceeds Marlin MAX_CMD_SIZE=96");
    return false;
  }
  uint8_t checksum = 0;
  for (int i = 0; i < n; ++i) checksum ^= entry.wire[i];
  int tail =
      snprintf(entry.wire + n, sizeof(entry.wire) - n, "*%u\n", checksum);
  if (tail < 0 || static_cast<size_t>(n + tail) > kLineSize) {
    fail(Error::Input, "numbered command exceeds Marlin MAX_CMD_SIZE=96");
    return false;
  }
  entry.length = n + tail;
  entry.sourceEnd = end;
  entry.wait = blocking(command);
  entry.temperature = code(command, 'M') == 105;
  if (_count == kSlots || _bytes + entry.length > kWindowBytes ||
      (_count && (_queue[0].wait || entry.wait)))
    return false;
  _queue[_count] = entry;
  if (!write(_queue[_count], now)) return false;
  ++_count;
  if (!reset) ++_next;
  return true;
}

bool MarlinStream::enqueue(const char* command, size_t end, uint32_t now) {
  return ready() && append(command, end, now);
}

bool MarlinStream::queueTemperature(const char* command) {
  if (!command || _temperatureCount == 4 ||
      (_state != State::Running && _state != State::Sync &&
       _state != State::Pausing && _state != State::Paused)) return false;
  const size_t length = strlen(command);
  if (length < 6 || length >= sizeof(_temperatures[0])) return false;
  // Only non-blocking setpoints, with S and optional hotend T. Reject compound
  // commands, comments, checksums and alternate Marlin modes (I/F/B/R).
  char clean[48];
  for (size_t i = 0; i <= length; ++i) {
    const unsigned char c = command[i];
    if (i < length && (c < 32 || c > 126)) return false;
    clean[i] = toupper(c);
  }
  if (strncmp(clean, "M104", 4) && strncmp(clean, "M140", 4)) return false;
  const bool hotend = clean[2] == '0';
  const char* p = clean + 4;
  if (*p != ' ') return false;
  bool haveS = false, haveT = false;
  while (*(p = spaces(p))) {
    const char parameter = *p++;
    if (parameter == 'S') {
      if (haveS) return false;
    } else if (parameter != 'T' || !hotend || haveT) return false;
    if (!isdigit(static_cast<unsigned char>(*p))) return false;
    unsigned value = 0;
    while (isdigit(static_cast<unsigned char>(*p))) {
      value = value * 10 + (*p++ - '0');
      if (value > (parameter == 'T' ? 255U : 999U)) return false;
    }
    if (parameter == 'S' && *p == '.') {
      ++p;
      if (!isdigit(static_cast<unsigned char>(*p))) return false;
      while (isdigit(static_cast<unsigned char>(*p))) ++p;
    }
    if (*p && *p != ' ') return false;
    if (parameter == 'S') haveS = true;
    else haveT = true;
  }
  if (!haveS) return false;
  strcpy(_temperatures[_temperatureCount++], clean);
  return true;
}

void MarlinStream::ack(uint32_t now) {
  const Entry entry = _queue[0];
  _bytes -= entry.length;
  _acknowledged = entry.sourceEnd;
  _lastAck = entry.number;
  _hasAck = true;
  for (size_t i = 1; i < _count; ++i) _queue[i - 1] = _queue[i];
  --_count;
  _activityAt = _progressAt = now;
  _retries = 0;
  if (_state == State::Sync) _state = State::Running;
}

void MarlinStream::resend(uint32_t requested, uint32_t now) {
  if (!_count || (_hasAck && requested <= _lastAck) ||
      requested < _queue[0].number || requested > _queue[_count - 1].number) {
    fail(Error::Resend, "resend outside retained unacknowledged window");
    return;
  }
  if (++_retries > 5) {
    fail(Error::Resend, "repeated resend without progress");
    return;
  }
  _bytes = 0;
  for (size_t i = 0; i < _count; ++i) {
    if (_queue[i].number >= requested) _queue[i].sent = false;
    if (_queue[i].sent) _bytes += _queue[i].length;
  }
  _awaitResend = false;
  _recovering = true;
  _recoveryAt = _activityAt = now;
}

void MarlinStream::line(char* response, uint32_t now) {
  const char* p = spaces(response);
  if (_recovering) _recoveryAt = now;
  if (starts(p, "start") &&
      (!p[5] || isspace(static_cast<unsigned char>(p[5])))) {
    fail(Error::Printer, "printer restarted during stream");
    return;
  }
  if (starts(p, "resend:") || starts(p, "rs ")) {
    p = spaces(p + (starts(p, "resend:") ? 7 : 3));
    if (*p == 'N' || *p == 'n') ++p;
    uint32_t requested;
    if (number(p, requested))
      resend(requested, now);
    else
      fail(Error::Resend, "malformed resend");
    return;
  }
  if (starts(p, "error:")) {
    // Marlin sends an Error line BEFORE Resend. Hold transmission until the
    // actual resend identifies the retained suffix; other errors are fatal.
    if (starts(spaces(p + 6), "checksum mismatch") ||
        starts(spaces(p + 6), "no checksum with line number") ||
        starts(spaces(p + 6), "line number is not last line number+1")) {
      _awaitResend = true;
      _recoveryAt = now;
    } else
      fail(Error::Printer, "printer reported an error");
    return;
  }
  if (starts(p, "//action:cancel") && !*spaces(p + 15)) {
    // A display cancel is a host request, not a printer fault. Stop feeding
    // the file and use the same acknowledged shutdown as ESP701 ABORT.
    cancel(now);
    return;
  }
  if (starts(p, "!!") || starts(p, "//action:disconnect")) {
    fail(Error::Printer, "printer stopped/cancelled the job");
    return;
  }
  if (starts(p, "busy:") || starts(p, "echo:busy:") || starts(p, "t:") ||
      starts(p, "t0:") || starts(p, "b:")) {
    _activityAt =
        now;  // Liveness only: never releases a command or byte credit.
    return;
  }
  if (!starts(p, "ok") || (p[2] && !isspace(static_cast<unsigned char>(p[2]))))
    return;
  p = spaces(p + 2);
  if (*p == 'N' || *p == 'n') {
    ++p;
    uint32_t n;
    if (!number(p, n)) {
      fail(Error::Ack, "malformed numbered acknowledgment");
      return;
    }
    if (_hasAck && n <= _lastAck) return;  // Duplicate; never frees new credit.
    if (!_count || !_queue[0].sent || n != _queue[0].number) {
      fail(Error::Ack, "out-of-order acknowledgment");
      return;
    }
    ack(now);
  } else if (_count && _queue[0].sent && _queue[0].temperature &&
             (starts(p, "t:") || starts(p, "t0:") || starts(p, "b:"))) {
    // M105 bypasses ADVANCED_OK in Marlin. It is always sent in isolation.
    ack(now);
  }
  // Plain ok from resend, old polling, or M108 is never a numbered ACK.
}

void MarlinStream::receive(const uint8_t* bytes, size_t size, uint32_t now) {
  for (size_t i = 0; i < size && !terminal(); ++i) {
    char c = static_cast<char>(bytes[i]);
    if (c == '\n') {
      Reply& saved = _replies[_replyNext];
      saved.length = _responseSize;
      saved.size = _responseSize < sizeof(saved.data) ? _responseSize : sizeof(saved.data);
      memcpy(saved.data, _response, saved.size);
      _replyNext = (_replyNext + 1) % 4;
      if (_replyCount < 4) ++_replyCount;
      if (_responseOverflow) {
        fail(Error::Ack, "printer response line too long");
      } else {
        _response[_responseSize] = 0;
        line(_response, now);
      }
      _responseSize = 0;
      _responseOverflow = false;
    } else if (c != '\r') {
      if (_responseSize + 1 < sizeof(_response) && c != 0)
        _response[_responseSize++] = c;
      else
        _responseOverflow = true;
    }
  }
}

void MarlinStream::tick(uint32_t now) {
  if (terminal()) return;
  if (_awaitResend && uint32_t(now - _recoveryAt) > 2000) {
    fail(Error::Resend, "checksum/sequence error without resend");
    return;
  }
  if (_count && (uint32_t(now - _activityAt) > kIdleTimeout ||
                 uint32_t(now - _progressAt) >
                     (_queue[0].wait ? kWaitTimeout : kAckTimeout))) {
    fail(Error::Timeout, _state == State::Sync
                             ? "no numbered sync ACK; ADVANCED_OK required"
                             : "printer acknowledgment timed out");
    return;
  }
  if (_awaitResend) return;
  if (_recovering) {
    // Let trailing rejected bytes / Resend + plain ok drain. Commands below
    // the requested line may still be executing: await their numbered ACKs.
    if (_count && !_queue[0].sent && uint32_t(now - _recoveryAt) >= 100) {
      for (size_t i = 0; i < _count; ++i) {
        if (!_queue[i].sent && !write(_queue[i], now)) return;
      }
      _recovering = false;
    }
    return;
  }
  if (_temperatureCount && _state != State::Sync) {
    // Preserve file progress: an inserted control acknowledges no new bytes.
    const size_t end = _count ? _queue[_count - 1].sourceEnd : _acknowledged;
    if (append(_temperatures[0], end, now)) {
      --_temperatureCount;
      for (size_t i = 0; i < _temperatureCount; ++i)
        strcpy(_temperatures[i], _temperatures[i + 1]);
    }
    // Keep remaining controls ahead of new file lines and pause/EOF barriers.
    if (_temperatureCount || terminal()) return;
  }
  if (_count) return;
  if (_state == State::Pausing || _state == State::Finishing) {
    if (_control == 0) {
      if (append("M400", _acknowledged, now)) _control = 1;
    } else {
      _state = _state == State::Pausing ? State::Paused : State::Complete;
    }
  } else if (_state == State::Cancelling) {
    static const char* const shutdown[] = {"M400", "M104 S0", "M140 S0",
                                           "M107"};
    if (_control < 4) {
      if (append(shutdown[_control], _acknowledged, now)) ++_control;
    } else
      _state = State::Cancelled;
  }
}

bool MarlinStream::pause() {
  if (_state != State::Running && _state != State::Sync) return false;
  _state = State::Pausing;
  _control = 0;
  return true;
}
bool MarlinStream::resume() {
  if (_state != State::Paused) return false;
  _state = State::Running;
  _control = 0;
  return true;
}
bool MarlinStream::cancel(uint32_t now) {
  if (terminal()) return false;
  if (_state == State::Cancelling) return true;
  _state = State::Cancelling;
  _control = 0;
  _temperatureCount = 0;  // A queued setpoint must never reheat after cancel.
  // EMERGENCY_PARSER interrupts M109/M190 even when the command queue is full.
  // This does not discard accepted motion; M400 waits before heater shutdown.
  if (!_send(_context, "M108\n", 5)) {
    fail(Error::Write, "cannot interrupt printer wait");
    return false;
  }
  _activityAt = now;
  return true;
}
void MarlinStream::finish() {
  if (_state == State::Running) {
    _state = State::Finishing;
    _control = 0;
  }
}
