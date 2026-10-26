#include <cassert>
#include <iostream>

#include "gcode_host.h"
#include "host_environment.h"
uint32_t testMillis = 0;
std::map<std::string, std::string> testFiles;
bool failRead = false;
unsigned fileReads = 0, fileCloses = 0, sdReleases = 0, sdTouches = 0;
uint8_t ESP3DSettings::target = MARLIN;
std::vector<Transmission> testTX;
std::vector<std::string> testReports;
ESP3DCommands esp3d_commands;
ESP3DRequest no_id{};
ESP3DMessageManager esp3d_message_manager;
ESP3DMessageManager::ESP3DMessageManager() {}
ESP3DMessageManager::~ESP3DMessageManager() {}
ESP3DMessage* ESP3DMessageManager::newMsg(ESP3DClientType origin,
                                          ESP3DClientType target,
                                          const uint8_t* data, size_t size,
                                          ESP3DAuthenticationLevel auth) {
  auto* msg = new ESP3DMessage{};
  msg->origin = origin;
  msg->target = target;
  msg->authentication_level = auth;
  msg->data = new uint8_t[size + 1];
  memcpy(msg->data, data, size);
  msg->data[size] = 0;
  msg->size = size;
  return msg;
}
bool ESP3DMessageManager::deleteMsg(ESP3DMessage* msg) {
  delete[] msg->data;
  delete msg;
  return true;
}
void receive(GcodeHost& host, const std::string& data,
             ESP3DClientType origin = ESP3DClientType::serial_bridge) {
  auto* msg = esp3d_message_manager.newMsg(
      origin, ESP3DClientType::stream,
      reinterpret_cast<const uint8_t*>(data.data()), data.size(),
      ESP3DAuthenticationLevel::admin);
  if (!host.dispatch(msg)) esp3d_message_manager.deleteMsg(msg);
}
void ack(GcodeHost& host, unsigned n) {
  receive(host, "ok N" + std::to_string(n) + " P10 B10\n");
}
void tick(GcodeHost& host) {
  testMillis += 10;
  host.handle();
}
unsigned frameNumber(const std::string& s) {
  assert(s[0] == 'N');
  return std::stoul(s.substr(1));
}
void runToEnd(GcodeHost& host, size_t cursor = 0) {
  for (unsigned i = 0; i < 2000 && host.getStatus() != HOST_NO_STREAM; ++i) {
    tick(host);
    while (cursor < testTX.size()) {
      auto wire = testTX[cursor++].wire;
      if (wire[0] == 'N') ack(host, frameNumber(wire));
    }
  }
  assert(host.getStatus() == HOST_NO_STREAM);
}
void resetFixtures() {
  testTX.clear();
  testReports.clear();
  testFiles.clear();
  failRead = false;
  fileReads = fileCloses = sdReleases = sdTouches = 0;
}
int main() {
  {
    resetFixtures();
    // Long comments, CRLF, parenthesis comments, blank lines, no final newline,
    // and a command straddling the 512-byte read-ahead boundary.
    std::string file = ";" + std::string(1000, 'x') +
                       "\r\n\nG1 X1 (comment) Y2\r\n; tail\nG1 X3";
    testFiles["/job.gcode"] = file;
    GcodeHost host;
    assert(host.processFile("/SD/job.gcode", ESP3DAuthenticationLevel::user));
    assert(host.ownsPrinter());
    tick(host);
    assert(testTX.size() == 1);
    receive(host, "ok N0\n", ESP3DClientType::webui_websocket);
    tick(host);
    assert(testTX.size() == 1);  // Wrong-origin ACK cannot start source.
    ack(host, 0);
    runToEnd(host, 1);
    assert(host.getErrorNum() == ERROR_NO_ERROR && !host.ownsPrinter());
    assert(host.processedSize() == file.size() &&
           host.totalSize() == file.size());
    assert(testTX.size() == 4 &&
           testTX[1].wire.find("G1 X1   Y2") != std::string::npos);
    assert(testTX[2].wire.find("G1 X3") != std::string::npos &&
           testTX[3].wire.find("M400") != std::string::npos);
    for (const auto& t : testTX)
      assert(t.auth == ESP3DAuthenticationLevel::user);
    assert(fileReads == 3 && fileCloses == 1 && sdReleases == 1 &&
           sdTouches > 0);
  }
  {
    resetFixtures();
    testFiles["/job"] = "G1 X1\nM109 S200\nG1 X2\n";
    GcodeHost host;
    host.processFile("/SD/job");
    tick(host);
    ack(host, 0);
    tick(host);
    assert(testTX.size() ==
           2);  // Pending heater barrier retained, not skipped.
    assert(!host.processFile("/SD/other") &&
           std::string(host.fileName()) == "/job");
    assert(!host.processScript("G1 X999"));  // No queued surprise after cancel.
    assert(host.pause());
    assert(host.getStatus() == HOST_PAUSING_STREAM);
    ack(host, 1);
    tick(host);
    assert(testTX.back().wire.find("M400") != std::string::npos);
    ack(host, 2);
    tick(host);
    assert(host.getStatus() == HOST_PAUSE_STREAM);
    assert(host.resume());
    tick(host);
    assert(testTX.back().wire.find("M109 S200") != std::string::npos);
    assert(host.abort());
    assert(host.getStatus() == HOST_ABORT_STREAM &&
           testTX.back().wire == "M108\n");
    ack(host, 3);
    runToEnd(host, testTX.size());
    assert(host.getErrorNum() == ERROR_STREAM_ABORTED && !host.ownsPrinter());
    for (const auto& t : testTX)
      assert(t.wire.find("G1 X2") == std::string::npos);
    assert(fileCloses == 1 && sdReleases == 1);
  }
  {
    resetFixtures();
    testFiles["/job"] = "G1 X1\n";
    GcodeHost host;
    assert(host.processFile("/FS/job"));
    assert(host.pause());
    tick(host);
    ack(host, 0);
    tick(host);
    ack(host, 1);
    tick(host);
    assert(host.getStatus() == HOST_PAUSE_STREAM && testTX.size() == 2);
    assert(host.resume());
    runToEnd(host, 2);
    assert(host.getErrorNum() == 0);
    GcodeHost early;
    assert(early.processFile("/SD/job"));
    auto count = testTX.size();
    assert(early.abort() && early.getStatus() == HOST_NO_STREAM);
    tick(early);
    assert(testTX.size() == count);
  }
  {
    resetFixtures();
    testFiles["/tuning"] = "M109 S210\nG1 X2\n";
    GcodeHost host;
    assert(!host.queueTemperature("M104 S200"));
    assert(host.processFile("/SD/tuning"));
    tick(host);
    ack(host, 0);
    tick(host);
    assert(host.queueTemperature("M104 S200 T0"));
    assert(host.queueTemperature("M140 S65"));
    tick(host);
    assert(testTX.size() == 2);
    ack(host, 1);
    tick(host);
    assert(testTX[2].wire.find("N2 M104 S200 T0*") == 0);
    tick(host);
    assert(testTX[3].wire.find("N3 M140 S65*") == 0);
    receive(host, "//action:cancel\n");
    runToEnd(host, 2);
    assert(host.getErrorNum() == ERROR_STREAM_ABORTED && !host.ownsPrinter());
    assert(!host.queueTemperature("M104 S200"));
    for (const auto& t : testTX) assert(t.wire.find("M112") == std::string::npos);
    assert(fileCloses == 1 && sdReleases == 1);
  }
  {
    resetFixtures();
    testFiles["/bad"] = "G1 X1\n";
    GcodeHost host;
    host.processFile("/SD/bad");
    tick(host);
    ack(host, 0);
    failRead = true;
    tick(host);
    assert(testTX.back().wire == "\nM112\n");
    tick(host);
    tick(host);
    assert(host.getStatus() == HOST_NO_STREAM &&
           host.getErrorNum() == ERROR_FILE_SYSTEM);
    assert(fileCloses == 1 && sdReleases == 1 && host.ownsPrinter());
    host.setErrorNum(0);
    assert(!host.processFile("/SD/bad"));
  }
  {
    resetFixtures();
    testFiles["/bad"] = "G1 X" + std::string(200, '1');
    GcodeHost host;
    host.processFile("/SD/bad");
    tick(host);
    ack(host, 0);
    tick(host);
    tick(host);
    tick(host);
    assert(host.getErrorNum() == ERROR_LINE_IGNORED && fileCloses == 1);
  }
  {
    resetFixtures();
    GcodeHost host;
    assert(host.processScript("G1 X1;G1 X2"));
    runToEnd(host);
    assert(testTX.size() == 4 && host.getErrorNum() == 0);
  }
  {
    resetFixtures();
    ESP3DSettings::target = SMOOTHIEWARE;
    GcodeHost host;
    assert(host.processScript("G1 X1;G1 X2"));
    tick(host);
    tick(host);
    tick(host);
    assert(testTX[0].wire == "G1 X1\n");
    receive(host, "ok\n");
    tick(host);
    tick(host);
    assert(testTX[1].wire == "G1 X2\n");
    receive(host, "ok\n");
    tick(host);
    tick(host);
    assert(host.getStatus() == HOST_NO_STREAM);
  }
  std::cout
      << "PASS: source reads, origin isolation, auth, pause/resume/cancel, EOF "
         "barriers, read faults, cleanup, legacy host\n";
}
