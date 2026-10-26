#include <cassert>
#include <cstring>
#include <deque>
#include <iostream>
#include <string>
#include <vector>

#include "marlin_stream.h"
using State = MarlinStream::State;
using Error = MarlinStream::Error;

struct Rig {
  MarlinStream stream;
  uint32_t now = 100;
  bool failWrite = false;
  std::vector<std::string> tx;
  static bool send(void* self, const char* bytes, size_t size) {
    auto& r = *static_cast<Rig*>(self);
    r.tx.emplace_back(bytes, size);
    return !r.failWrite;
  }
  void receive(const std::string& s) {
    stream.receive(reinterpret_cast<const uint8_t*>(s.data()), s.size(), now);
  }
  void start() {
    assert(stream.begin(send, this, now));
    assert(tx.size() == 1 && tx[0].find("N0 M110 N0*") == 0);
    receive("ok N0 P15 B15\r\n");
    assert(stream.ready());
  }
  bool add(const char* line, size_t end = 10) {
    return stream.enqueue(line, end, now);
  }
  void ack(unsigned n) { receive("ok N" + std::to_string(n) + " P15 B15\n"); }
  void tick(uint32_t elapsed = 1) {
    now += elapsed;
    stream.tick(now);
  }
};

void acknowledgments() {
  Rig r;
  r.start();
  assert(r.add("G1 X1", 10) && r.add("G1 X2", 20) && r.add("G1 X3", 30));
  const size_t bytes = r.stream.inFlightBytes();
  r.receive(
      "echo:book\nok\nok T:210.0 /210.0 B:60.0 /60.0\nbusy: processing\n");
  assert(r.stream.pending() == 3 && r.stream.inFlightBytes() == bytes);
  r.receive("o");
  r.receive("k N");
  r.receive("1 P10 B10\r");
  assert(r.stream.acknowledged() == 0);
  r.receive("\n");
  assert(r.stream.acknowledged() == 10);
  r.ack(1);
  assert(r.stream.pending() == 2);  // Duplicate.
  r.receive("ok N2 P15 B15\nok N3 P15 B15\n");
  assert(r.stream.pending() == 0 && r.stream.acknowledged() == 30);
  assert(r.add("M105", 40));
  assert(!r.add("G1 X4"));
  r.receive("ok\n");
  assert(r.stream.pending() == 1);
  r.receive("ok T:210.0 /210.0 B:60.0 /60.0\n");
  assert(r.stream.pending() == 0 && r.stream.acknowledged() == 40);
  assert(r.add("G1 X4"));
  r.ack(6);
  assert(r.stream.error() == Error::Ack);
}

void window() {
  Rig r;
  r.start();
  unsigned count = 0;
  while (r.add("G1 X10 Y20 E0.1", ++count)) {
    assert(r.stream.pending() <= MarlinStream::kSlots);
    assert(r.stream.inFlightBytes() <= MarlinStream::kWindowBytes);
  }
  assert(count > 2 && r.stream.state() == State::Running);
  r.ack(1);
  assert(r.add("G1 X10 Y20 E0.1", 100));
  for (const auto& wire : r.tx) {
    auto star = wire.find('*');
    assert(star != std::string::npos);
    uint8_t sum = 0;
    for (size_t i = 0; i < star; ++i) sum ^= wire[i];
    assert(std::stoul(wire.substr(star + 1)) == sum);
    assert(wire.back() == '\n' && wire.size() <= MarlinStream::kLineSize);
  }
}

void resends() {
  Rig r;
  r.start();
  assert(r.add("G1 X1", 10) && r.add("G1 X2", 20) && r.add("G1 X3", 30));
  auto originals = r.tx;
  r.receive("Error:checksum mismatch, Last Line: 1\n");
  assert(!r.add("G1 X4"));
  r.receive("Resend: 2\nok\n");
  r.tick(200);
  assert(r.tx.size() == originals.size());  // N1 still executing.
  r.ack(1);
  r.tick(100);
  assert(r.tx.size() == originals.size() + 2);
  assert(r.tx[4] == originals[2] && r.tx[5] == originals[3]);
  r.receive("ok\n");
  assert(r.stream.acknowledged() == 10);
  r.ack(2);
  r.ack(3);
  assert(r.stream.acknowledged() == 30);
  r.receive("Resend: 2\n");
  assert(r.stream.error() == Error::Resend);

  Rig repeated;
  repeated.start();
  assert(repeated.add("G1 X1"));
  for (int i = 0; i < 6; ++i) {
    repeated.receive("Resend: 1\nok\n");
    repeated.tick(100);
  }
  assert(repeated.stream.error() == Error::Resend);
  Rig missing;
  missing.start();
  assert(missing.add("G1 X1"));
  missing.receive(
      "Error:Line Number is not Last Line Number+1, Last Line: 0\n");
  missing.tick(2001);
  assert(missing.stream.error() == Error::Resend);
  Rig malformed;
  malformed.start();
  malformed.add("G1 X1");
  malformed.receive("Resend: abc\n");
  assert(malformed.stream.error() == Error::Resend);
  Rig outside;
  outside.start();
  outside.add("G1 X1");
  outside.receive("Resend: 99\n");
  assert(outside.stream.error() == Error::Resend);
}

void heatingAndTimeouts() {
  Rig r;
  r.start();
  assert(r.add("M109 S210", 50));
  assert(!r.add("G1 X1"));
  for (int i = 0; i < 600; ++i) {
    r.tick(2000);
    r.receive(i % 2 ? "echo:busy: processing\n"
                    : " T:180.0 /210.0 B:60.0 /60.0\n");
    assert(r.stream.state() == State::Running && r.stream.acknowledged() == 0);
  }
  r.ack(1);
  assert(r.add("M190 S60", 80));
  r.receive("busy: heating\n");
  r.tick(30001);
  assert(r.stream.error() == Error::Timeout && r.tx.back() == "\nM112\n");
  assert(!r.stream.begin(Rig::send, &r, r.now));  // Latched fault.

  Rig lostAck;
  lostAck.start();
  assert(lostAck.add("G1 X1"));
  for (int i = 0; i < 61; ++i) {
    lostAck.receive("T:200.0 /200.0\n");
    lostAck.tick(2000);
  }
  assert(lostAck.stream.error() ==
         Error::Timeout);  // Chatter can't hide a lost ACK.

  Rig sync;
  assert(sync.stream.begin(Rig::send, &sync, sync.now));
  sync.receive("ok\nok T:20 /0\n");
  sync.tick(30001);
  assert(sync.stream.error() == Error::Timeout && sync.tx.size() == 2);

  Rig wrap;
  wrap.now = UINT32_MAX - 100;
  wrap.start();
  wrap.add("G1 X1");
  wrap.tick(200);
  assert(wrap.stream.state() == State::Running);
  wrap.tick(30001);
  assert(wrap.stream.error() == Error::Timeout);
}

void controls() {
  Rig r;
  r.start();
  assert(r.add("G1 X1", 10) && r.add("G1 X2", 20));
  assert(r.stream.pause());
  assert(!r.add("G1 X3"));
  assert(!r.stream.resume());
  r.ack(1);
  r.tick();
  assert(r.tx.size() == 3);
  r.ack(2);
  r.tick();
  assert(r.tx.back().find(" M400*") != std::string::npos);
  assert(r.stream.state() == State::Pausing);
  r.ack(3);
  r.tick();
  assert(r.stream.state() == State::Paused);
  r.tick(1000000);
  assert(r.stream.state() == State::Paused);
  assert(r.stream.resume());
  assert(r.add("G1 X3", 30));
  r.stream.finish();
  r.tick();
  assert(r.stream.state() == State::Finishing);
  r.ack(4);
  r.tick();
  assert(r.tx.back().find(" M400*") != std::string::npos);
  r.ack(5);
  r.tick();
  assert(r.stream.state() == State::Complete);
  assert(r.stream.acknowledged() == 30);

  Rig c;
  c.start();
  assert(c.add("M109 S210", 15));
  assert(c.stream.cancel(c.now));
  assert(c.tx.back() == "M108\n");
  auto before = c.tx.size();
  assert(c.stream.cancel(c.now));
  assert(c.tx.size() == before);
  assert(!c.add("G1 X100"));
  c.receive("ok\n");
  assert(c.stream.pending() == 1);
  c.ack(1);
  c.tick();
  assert(c.tx.back().find(" M400*") != std::string::npos);
  c.ack(2);
  c.tick();
  assert(c.tx.back().find(" M104 S0*") != std::string::npos);
  c.ack(3);
  c.tick();
  assert(c.tx.back().find(" M140 S0*") != std::string::npos);
  c.ack(4);
  c.tick();
  assert(c.tx.back().find(" M107*") != std::string::npos);
  assert(c.stream.state() == State::Cancelling);
  c.ack(5);
  c.tick();
  assert(c.stream.state() == State::Cancelled);
  assert(
      c.stream.begin(Rig::send, &c, c.now));  // Clean cancel permits next job.

  Rig early;
  assert(early.stream.begin(Rig::send, &early, early.now));
  assert(early.stream.pause());
  early.ack(0);
  early.tick();
  early.ack(1);
  early.tick();
  assert(early.stream.state() == State::Paused);
}

void temperatureControls() {
  Rig r;
  r.start();
  assert(r.add("G1 X1", 10) && r.add("G1 X2", 20));
  const auto before = r.tx.size();
  assert(r.stream.queueTemperature("m104 s205.5 t0"));
  assert(r.stream.queueTemperature("M140 S65"));
  assert(r.tx.size() == before);  // Caller only queues; tick owns TX.
  assert(!r.add("G1 X3", 30));  // Setpoints take priority over new source.
  r.tick();
  r.tick();
  assert(r.tx[3].find("N3 M104 S205.5 T0*") == 0);
  assert(r.tx[4].find("N4 M140 S65*") == 0);
  assert(r.stream.acknowledged() == 0);
  r.ack(1);
  r.ack(2);
  assert(r.stream.acknowledged() == 20);
  const auto hotend = r.tx[3], bed = r.tx[4];
  r.receive("Error:checksum mismatch, Last Line: 2\nResend: 3\nok\n");
  r.tick(100);
  assert(r.tx[5] == hotend && r.tx[6] == bed);
  r.ack(3);
  r.ack(4);
  assert(r.stream.acknowledged() == 20);  // No progress regression or advance.
  assert(r.add("G1 X3", 30));
  r.ack(5);
  assert(r.stream.acknowledged() == 30);
  assert(r.stream.pause());
  r.tick();
  r.ack(6);
  r.tick();
  assert(r.stream.state() == State::Paused);
  assert(r.stream.queueTemperature("M104 T0 S0"));
  r.tick();
  r.ack(7);
  r.tick();
  assert(r.stream.state() == State::Paused && r.stream.acknowledged() == 30);

  Rig wait;
  wait.start();
  assert(wait.add("M109 S210", 42));
  assert(wait.stream.queueTemperature("M104 S200"));
  assert(wait.stream.queueTemperature("M140 S60"));
  assert(wait.stream.queueTemperature("M104 S201"));
  assert(wait.stream.queueTemperature("M104 S202"));
  assert(!wait.stream.queueTemperature("M104 S203"));  // Bounded admission.
  wait.tick();
  assert(wait.tx.size() == 2);  // Never bypass an executing heat wait.
  wait.ack(1);
  wait.tick();
  assert(wait.tx[2].find("N2 M104 S200*") == 0);
  assert(wait.stream.acknowledged() == 42);

  Rig invalid;
  invalid.start();
  for (const char* cmd : {"M105", "M109 S210", "M190 S60", "M104", "M104 T0",
      "M104 S200\nM112", "M104 S200;M112", "M104 S200*9", "M104 S200 G28",
      "M104 S200 S0", "M104 S200 I", "M104 S200 B250", "M140 S60 T0",
      "M104 S-1", "M104 Snan", "M104 S1e3", "M104 S200 T0.5",
      "M104 S99999999999", "M104 S200 T99999999999"}) {
    assert(!invalid.stream.queueTemperature(cmd));
    assert(invalid.stream.state() == State::Running && invalid.tx.size() == 1);
  }
  assert(!invalid.stream.queueTemperature(nullptr));
  // Temperature commands still respect the wire window when motion fills it.
  while (invalid.add("G1 X10 Y20 E0.1")) {}
  assert(invalid.stream.queueTemperature("M104 S210 T0"));
  auto sent = invalid.tx.size();
  invalid.tick();
  assert(invalid.tx.size() == sent);
  invalid.ack(1);
  invalid.tick();
  assert(invalid.tx.back().find(" M104 S210 T0*") != std::string::npos);
  assert(invalid.stream.inFlightBytes() <= MarlinStream::kWindowBytes);
}

void displayCancel() {
  Rig r;
  r.start();
  for (unsigned n = 1; n <= 313; ++n) {
    assert(r.add("G90", n));
    r.ack(n);
  }
  assert(r.add("G1 X10", 314));
  assert(r.stream.queueTemperature("M104 S220"));
  r.receive("//action:cancel\n//action:prompt_end\n//action:prompt_begin UI Aborted\n"
            "//action:prompt_button Dismiss\n//action:prompt_show\nok N314 P11 B14\n");
  assert(r.stream.state() == State::Cancelling && r.stream.error() == Error::None);
  assert(r.tx.back() == "M108\n");
  const auto count = r.tx.size();
  r.receive("//action:cancel\n");
  assert(r.tx.size() == count);  // Repeated display signal is idempotent.
  assert(!r.stream.queueTemperature("M140 S60"));
  assert(!r.add("G1 X20"));
  unsigned n = 315;
  for (const char* command : {"M400", "M104 S0", "M140 S0", "M107"}) {
    r.tick();
    assert(r.tx.back().find(std::string(" ") + command + "*") != std::string::npos);
    r.ack(n++);
  }
  r.tick();
  assert(r.stream.state() == State::Cancelled && r.stream.acknowledged() == 314);
  for (const auto& wire : r.tx) {
    assert(wire.find("M112") == std::string::npos);
    assert(wire.find("S220") == std::string::npos);
  }
  assert(r.stream.begin(Rig::send, &r, r.now));

  Rig failure;
  failure.start();
  failure.add("G28");
  failure.receive("//action:cancel\nError:Printer halted. kill() called!\n");
  assert(failure.stream.state() == State::Fault && failure.tx.back() == "\nM112\n");
}

void faults() {
  for (const char* input : {"N1 G1 X1*2", "M110 N0", "[ESP700]/SD/foo", "M112",
                            "M24", "G1 X1\nM104 S200", "M117 \"hello\""}) {
    Rig r;
    r.start();
    assert(!r.add(input));
    assert(r.stream.error() == Error::Input);
  }
  Rig longLine;
  longLine.start();
  std::string command = "G1 X" + std::string(90, '1');
  assert(!longLine.add(command.c_str()) &&
         longLine.stream.error() == Error::Input);
  Rig write;
  write.start();
  write.failWrite = true;
  assert(!write.add("G1 X1") && write.stream.error() == Error::Write);
  for (const char* response :
       {"start\n", "Error:THERMAL RUNAWAY\n", "!! STOPPED\n",
        "//action:disconnect\n", "ok N999999999999999999\n"}) {
    Rig r;
    r.start();
    r.add("G1 X1");
    r.receive(response);
    assert(r.stream.state() == State::Fault && r.tx.back() == "\nM112\n");
  }
  Rig overlong;
  overlong.start();
  overlong.add("G1 X1");
  overlong.receive(std::string(300, 'a') + "ok N1\n");
  assert(overlong.stream.error() == Error::Ack);
}

// Independent printer model: limited receive bytes + command slots, validates
// line sequence/checksum, drops the suffix after a corrupted frame and sends
// Marlin's Error/Resend/plain-ok sequence. Each relative move executes once.
void simulatedJob() {
  Rig r;
  assert(r.stream.begin(Rig::send, &r, r.now));
  std::deque<std::string> rx;
  std::deque<unsigned> commands;
  std::vector<unsigned> executed;
  size_t txCursor = 0, source = 0, rxBytes = 0;
  unsigned lastN = 0, injected = 0, pendingMax = 0;
  bool bad17 = false, bad113 = false, bad507 = false;
  for (unsigned time = 0; time < 30000 && r.stream.state() != State::Complete;
       ++time) {
    if (r.stream.ready() && source < 1000) {
      std::string cmd = "G1 X0.01 E0.001";
      if (r.stream.enqueue(cmd.c_str(), source + 1, r.now)) ++source;
    }
    if (source == 1000) r.stream.finish();
    while (txCursor < r.tx.size()) {
      const std::string wire = r.tx[txCursor++];
      assert(wire.find("M112") == std::string::npos);
      rx.push_back(wire);
      rxBytes += wire.size();
      assert(rxBytes <= 127);
    }
    // Printer parser works independently of slower execution.
    while (!rx.empty() && commands.size() < 4) {
      std::string wire = rx.front();
      rx.pop_front();
      rxBytes -= wire.size();
      unsigned n = std::stoul(wire.substr(1));
      auto star = wire.find('*');
      assert(star != std::string::npos);
      uint8_t sum = 0;
      for (size_t i = 0; i < star; ++i) sum ^= wire[i];
      assert(sum == std::stoul(wire.substr(star + 1)));
      bool corrupt =
          (n == 17 && !bad17) || (n == 113 && !bad113) || (n == 507 && !bad507);
      if (corrupt) {
        if (n == 17) bad17 = true;
        if (n == 113) bad113 = true;
        if (n == 507) bad507 = true;
        ++injected;
        rx.clear();
        rxBytes = 0;
        r.receive(
            "Error:checksum mismatch, Last Line: " + std::to_string(lastN) +
            "\nResend: " + std::to_string(lastN + 1) + "\nok\n");
        break;
      }
      assert(n == 0 || n == lastN + 1);
      lastN = n;
      commands.push_back(n);
    }
    pendingMax = std::max(pendingMax, static_cast<unsigned>(commands.size()));
    if (time % 5 == 0 && !commands.empty()) {
      auto n = commands.front();
      commands.pop_front();
      if (n && n <= 1000) executed.push_back(n);
      std::string response = "ok N" + std::to_string(n) + " P15 B0\n";
      // Every byte boundary is a possible receive callback boundary.
      for (char c : response) r.receive(std::string(1, c));
    }
    r.tick(10);
    assert(r.stream.state() != State::Fault);
  }
  assert(r.stream.state() == State::Complete &&
         r.stream.acknowledged() == 1000);
  assert(injected == 3 && pendingMax > 1 && executed.size() == 1000);
  for (size_t i = 0; i < executed.size(); ++i) assert(executed[i] == i + 1);
}
// Replay the printer trace, including every possible callback split in the
// damaged reply. A later ACK cannot prove that the missing command executed.
void damagedAckDiagnostics() {
  const std::string damaged = "ok3N212 P0 B13\n";
  for (size_t split = 0; split <= damaged.size(); ++split) {
    Rig r;
    r.start();
    for (unsigned n = 1; n <= 211; ++n) {
      assert(r.add("G90"));
      r.ack(n);
    }
    assert(r.add("G90") && r.add("G90"));
    const size_t sent = r.tx.size();
    r.receive(damaged.substr(0, split));
    r.receive(damaged.substr(split));
    assert(r.stream.lastAck() == 211 && r.stream.expectedAck() == 212);
    assert(r.stream.pending() == 2 && r.tx.size() == sent);
    r.receive("ok N213 P0 B14\n");
    assert(r.stream.error() == Error::Ack);
    assert(std::string(r.stream.reason()) == "out-of-order acknowledgment");
    assert(r.tx.size() == sent + 1 && r.tx.back() == "\nM112\n");
    assert(r.stream.replyCount() == 4);
    const auto& reply = r.stream.reply(2);
    assert(std::string((const char*)reply.data, reply.size) == "ok3N212 P0 B13");
    r.receive("echo:M112 Shutdown\nError:Printer halted. kill() called!\n");
    r.tick();
    assert(r.tx.size() == sent + 1);
    assert(std::string((const char*)reply.data, reply.size) == "ok3N212 P0 B13");
  }
  Rig valid;
  valid.start();
  for (unsigned n = 1; n <= 213; ++n) {
    assert(valid.add("G90"));
    valid.ack(n);
  }
  assert(valid.stream.lastAck() == 213 && valid.stream.pending() == 0);
  valid.receive(std::string(120, 'x') + "\n");
  const auto& longReply = valid.stream.reply(3);
  assert(longReply.length == 120 && longReply.size == 96);
  valid.stream.finish();
  valid.tick();
  valid.ack(214);
  valid.tick();
  assert(valid.stream.begin(Rig::send, &valid, valid.now));
  assert(valid.stream.replyCount() == 0);
}

int main() {
  acknowledgments();
  window();
  resends();
  heatingAndTimeouts();
  controls();
  temperatureControls();
  displayCancel();
  faults();
  simulatedJob();
  damagedAckDiagnostics();
  std::cout << "PASS: acknowledgments, byte window, resends, heating/timeouts, "
               "controls, faults, 1000-command simulated job\n";
}
