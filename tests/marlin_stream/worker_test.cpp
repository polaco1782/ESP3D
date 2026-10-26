#include <cassert>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include "gcode_host.h"
#include "host_environment.h"
uint32_t testMillis = 0;
std::map<std::string, std::string> testFiles;
TestFlag failRead{false};
TestCount fileReads{0}, fileCloses{0}, sdReleases{0}, sdTouches{0};
uint8_t ESP3DSettings::target = MARLIN;
std::vector<Transmission>
    testTX;  // Main-task dispatch is separate from raw UART.
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
ESP3DSerialService esp3d_serial_service, serial_bridge_service;
std::mutex txMutex;
std::condition_variable txChanged;
std::vector<std::string> uartTX;
std::atomic<bool> blockWrite{false};
const std::thread::id mainThread = std::this_thread::get_id();
size_t ESP3DSerialService::writeBytes(const uint8_t* data, size_t size) {
  assert(std::this_thread::get_id() !=
         mainThread);  // No main-loop protocol TX.
  {
    std::lock_guard<std::mutex> lock(txMutex);
    uartTX.emplace_back((const char*)data, size);
  }
  txChanged.notify_all();
  while (blockWrite.load())
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  return size;
}
std::string frame(size_t index) {
  std::unique_lock<std::mutex> lock(txMutex);
  assert(txChanged.wait_for(lock, std::chrono::seconds(5),
                            [&] { return uartTX.size() > index; }));
  return uartTX[index];
}
void waitState(GcodeHost& host, uint8_t expected) {
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (host.getStatus() != expected &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  assert(host.getStatus() == expected);
}
void rx(GcodeHost& host, const std::string& s) {
  assert(host.captureSerial(ESP3DClientType::serial_bridge,
                            (const uint8_t*)s.data(), s.size()));
}
void ack(GcodeHost& host, unsigned n) {
  rx(host, "ok N" + std::to_string(n) + " P15 B15\n");
}
void clearFixtures() {
  joinTestTasks();
  std::lock_guard<std::mutex> lock(txMutex);
  uartTX.clear();
  testTX.clear();
  testReports.clear();
  testFiles.clear();
  fileReads = 0;
  fileCloses = 0;
  sdReleases = 0;
  sdTouches = 0;
  failRead = false;
}
void blockedMainLoop() {
  clearFixtures();
  GcodeHost host;
  assert(host.begin());
  assert(host.workerCore() == (CONFIG_FREERTOS_UNICORE ? 0 : 1 - testMainCore));
  std::string job;
  for (unsigned i = 0; i < 1000; ++i) job += "G1 X0.01 E0.001\n";
  testFiles["/job"] = job;
  assert(host.processFile("/SD/job"));
  host.handle();
  assert(host.backgroundOwnsSD());
  // The application's HTTP/main-loop task is now completely absent. A printer
  // thread only sees UART bytes and returns fragmented ACKs to the UART hook.
  std::thread printer([&] {
    for (unsigned n = 0; n <= 1001; ++n) {
      std::string wire = frame(n);
      assert(wire[0] == 'N' && std::stoul(wire.substr(1)) == n);
      auto star = wire.find('*');
      uint8_t sum = 0;
      for (size_t i = 0; i < star; ++i) sum ^= wire[i];
      assert(sum == std::stoul(wire.substr(star + 1)));
      std::string response = "ok N" + std::to_string(n) + " P15 B15\n";
      rx(host, response.substr(0, 2));
      rx(host, response.substr(2));
    }
  });
  printer.join();
  waitState(host, HOST_STOP_STREAM);
  assert(host.processedSize() == job.size());
  assert(testTX.empty() &&
         testReports.empty());  // Worker never called dispatcher.
  assert(host.backgroundOwnsSD() &&
         fileCloses == 0);  // Main owns file lifecycle.
  host.handle();
  assert(host.getStatus() == HOST_NO_STREAM && !host.backgroundOwnsSD());
  assert(fileCloses == 1 && sdReleases == 1);
  assert(!testReports.empty() &&
         testReports.size() <= 16);  // Bounded UI backlog.
  host.end();
}
void controlsWithoutMainLoop() {
  clearFixtures();
  GcodeHost host;
  assert(host.begin());
  assert(host.processScript("M109 S200;G1 X1;G1 X2"));
  host.handle();
  assert(frame(0).find("M110") != std::string::npos);
  ack(host, 0);
  assert(frame(1).find("M109") != std::string::npos);
  const char* unrelated = "ok N1\n";
  assert(!host.captureSerial(ESP3DClientType::serial, (const uint8_t*)unrelated,
                             strlen(unrelated)));
  assert(host.pause());
  waitState(host, HOST_PAUSING_STREAM);
  rx(host, "echo:busy: processing\n");
  ack(host, 1);
  assert(frame(2).find("M400") != std::string::npos);
  ack(host, 2);
  waitState(host, HOST_PAUSE_STREAM);
  assert(host.resume());
  assert(frame(3).find("G1 X1") != std::string::npos);
  assert(frame(4).find("G1 X2") != std::string::npos);
  assert(host.abort());
  assert(frame(5) == "M108\n");
  ack(host, 3);
  ack(host, 4);
  for (unsigned n = 5; n <= 8; ++n) {
    frame(n + 1);
    ack(host, n);
  }
  waitState(host, HOST_STOP_STREAM);
  host.handle();
  assert(host.getErrorNum() == ERROR_STREAM_ABORTED);
  host.end();
}
void temperatureAndDisplayCancel() {
  clearFixtures();
  GcodeHost host;
  assert(host.begin());
  assert(host.processScript("M109 S210;M190 S60;G1 X99"));
  host.handle();
  frame(0);
  ack(host, 0);
  assert(frame(1).find("M109") != std::string::npos);
  assert(host.queueTemperature("M104 S200 T0"));
  assert(host.queueTemperature("M140 S65"));
  ack(host, 1);
  assert(frame(2).find("N2 M104 S200 T0*") == 0);
  assert(frame(3).find("N3 M140 S65*") == 0);
  ack(host, 2);
  ack(host, 3);
  assert(frame(4).find("N4 M190 S60*") == 0);
  assert(host.queueTemperature("M104 S220"));
  rx(host, "//action:cancel\n//action:prompt_end\n");
  assert(frame(5) == "M108\n");
  assert(!host.queueTemperature("M140 S80"));
  ack(host, 4);
  for (unsigned n = 5; n <= 8; ++n) {
    frame(n + 1);
    ack(host, n);
  }
  waitState(host, HOST_STOP_STREAM);
  host.handle();
  assert(host.getErrorNum() == ERROR_STREAM_ABORTED && !host.ownsPrinter());
  {
    std::lock_guard<std::mutex> lock(txMutex);
    for (const auto& wire : uartTX) {
      assert(wire.find("M112") == std::string::npos);
      assert(wire.find("G1 X99") == std::string::npos);
      assert(wire.find("S220") == std::string::npos);
    }
  }
  host.end();
}
void overflowAndPriorityStop(bool emergency) {
  clearFixtures();
  GcodeHost host;
  assert(host.begin());
  blockWrite.store(true);
  assert(host.processScript("G1 X100"));
  host.handle();
  frame(0);
  if (emergency) {
    // Emergency request must return even while the worker owns its host lock
    // inside a delayed UART write.
    host.emergencyStop();
  } else {
    std::string burst(64 * 40, 'x');
    rx(host, burst);
  }
  blockWrite.store(false);
  assert(frame(1) == "\nM112\n");
  waitState(host, HOST_ERROR_STREAM);
  assert(host.getErrorNum() == (emergency ? ERROR_UNKNOW : ERROR_ACK_NUMBER));
  host.handle();
  host.handle();
  assert(host.ownsPrinter());
  host.end();
}
void lifecycleAndFailure() {
  clearFixtures();
  {
    GcodeHost host;
    assert(host.begin());
    assert(host.processScript("M109 S200"));
    host.handle();
    frame(0);
    ack(host, 0);
    frame(1);
    host.end();  // Stops worker, attempts M112, joins before UART/file
                 // teardown.
    assert(frame(2) == "\nM112\n");
    assert(host.getStatus() == HOST_NO_STREAM);
    assert(!host.captureSerial(ESP3DClientType::serial_bridge,
                               (const uint8_t*)"ok\n", 3));
    assert(host.begin());  // Restart lifecycle is supported but protocol fault
                           // stays latched.
    assert(!host.processScript("G1 X999"));
    host.end();
  }
  joinTestTasks();
  failTaskCreation = true;
  {
    GcodeHost host;
    assert(!host.begin());
    assert(!host.processScript("G1 X1"));
  }
  failTaskCreation = false;
}
std::string hexBytes(const std::string& bytes) {
  const char* digits = "0123456789ABCDEF";
  std::string result;
  for (unsigned char byte : bytes) {
    result += digits[byte >> 4];
    result += digits[byte & 15];
  }
  return result;
}
void damagedAckDiagnostics() {
  clearFixtures();
  GcodeHost host;
  assert(host.begin());
  std::string job;
  for (unsigned n = 1; n <= 220; ++n) job += "G90\n";
  testFiles["/diag"] = job;
  assert(host.processFile("/SD/diag"));
  host.handle();
  std::string received;
  for (unsigned n = 0; n <= 211; ++n) {
    frame(n);
    const std::string reply = "ok N" + std::to_string(n) + " P0 B13\n";
    rx(host, reply);
    received += reply;
  }
  frame(213);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::string(host.diagnostics().c_str()).find("\"last_ack\":211") ==
             std::string::npos && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  assert(std::string(host.diagnostics().c_str()).find("\"last_ack\":211") !=
         std::string::npos);
  const std::string damaged = "ok3N212 P0 B13\nok N213 P0 B14\n";
  // Overwrite each caller buffer immediately: queue and raw tail must own it.
  for (char byte : damaged) {
    uint8_t temporary = byte;
    assert(host.captureSerial(ESP3DClientType::serial_bridge, &temporary, 1));
    temporary = 0xff;
  }
  received += damaged;
  waitState(host, HOST_ERROR_STREAM);
  auto diagnostic = std::string(host.diagnostics().c_str());
  for (const char* field : {"\"last_ack\":211", "\"expected_ack\":212",
                            "\"packet_mismatches\":0", "\"queue_overflows\":0",
                            "\"frame\":0", "\"parity\":0"}) {
    if (diagnostic.find(field) == std::string::npos)
      std::cerr << "Missing " << field << " in " << diagnostic << '\n';
    assert(diagnostic.find(field) != std::string::npos);
  }
  assert(diagnostic.find("\"hex\":\"" + hexBytes("ok3N212 P0 B13") + "\"") !=
         std::string::npos);
  assert(diagnostic.find("\"rx_tail_hex\":\"" +
                         hexBytes(received.substr(received.size() - 256)) + "\"") !=
         std::string::npos);
  // Error callback counters are attributed only to the selected UART, and
  // remain available after main-loop cleanup and late driver error events.
  host.recordSerialError(ESP3DClientType::serial, 4);
  assert(std::string(host.diagnostics().c_str()) == diagnostic);
  for (unsigned error = 1; error <= 5; ++error)
    host.recordSerialError(ESP3DClientType::serial_bridge, error);
  host.recordSerialError(ESP3DClientType::serial_bridge, 0);
  host.recordSerialError(ESP3DClientType::serial_bridge, 6);
  diagnostic = host.diagnostics().c_str();
  assert(diagnostic.find("\"uart_errors_since_start\":{\"break\":1,\"buffer_full\":1,"
                         "\"fifo_overflow\":1,\"frame\":1,\"parity\":1}") !=
         std::string::npos);
  host.handle();
  host.handle();
  assert(host.getStatus() == HOST_NO_STREAM && host.ownsPrinter());
  assert(std::string(host.diagnostics().c_str()) == diagnostic);
  host.end();
}
int main() {
  blockedMainLoop();
  controlsWithoutMainLoop();
  temperatureAndDisplayCancel();
  overflowAndPriorityStop(false);
  overflowAndPriorityStop(true);
  lifecycleAndFailure();
  damagedAckDiagnostics();
  joinTestTasks();
  std::cout
      << "PASS: threaded worker completes 1000 commands without main-loop "
         "service, bounded UI, controls, RX overflow, emergency priority, "
         "shutdown, task creation failure, core selection\n";
}
