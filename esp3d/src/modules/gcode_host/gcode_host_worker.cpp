// SPDX-License-Identifier: LGPL-2.1-or-later
#include "../../include/esp3d_config.h"
#if defined(GCODE_HOST_FEATURE) && defined(ARDUINO_ARCH_ESP32)
#include "../../core/esp3d_commands.h"
#include "gcode_host.h"

uint32_t GcodeHost::packetChecksum(const RxPacket& packet) {
  uint32_t hash = 2166136261U;
  for (size_t i = 0; i < packet.size; ++i)
    hash = (hash ^ packet.data[i]) * 16777619U;
  return hash;
}

void GcodeHost::recordSerialError(ESP3DClientType origin, unsigned error) {
  if (!_ingressMutex || error == 0 || error >= 6) return;
  xSemaphoreTake(_ingressMutex, portMAX_DELAY);
  // Keep late driver error events too, until the next job resets the counters.
  if (_generation && origin == _captureOrigin) ++_uartErrors[error];
  xSemaphoreGive(_ingressMutex);
}

bool GcodeHost::beginWorker() {
  if (!_hostMutex) _hostMutex = xSemaphoreCreateRecursiveMutex();
  if (!_ingressMutex) _ingressMutex = xSemaphoreCreateMutex();
  if (!_workerStopped) _workerStopped = xSemaphoreCreateBinary();
  if (!_workerRx) _workerRx = xQueueCreate(32, sizeof(RxPacket));
  if (!_workerUI) _workerUI = xQueueCreate(16, sizeof(RxPacket));
  if (!_workerControls) _workerControls = xQueueCreate(8, sizeof(Control));
  if (!_hostMutex || !_ingressMutex || !_workerStopped || !_workerRx ||
      !_workerUI || !_workerControls)
    return false;
  _workerShutdown.store(false);
  // Match the actual application task's affinity, including single-core chips.
#if CONFIG_FREERTOS_UNICORE
  _workerCore = 0;
#else
  _workerCore = xPortGetCoreID() == 0 ? 1 : 0;
#endif
  return xTaskCreatePinnedToCore(workerEntry, "marlin_stream", 6144, this, 2,
                                 &_workerTask, _workerCore) == pdPASS;
}

void GcodeHost::notifyWorker() {
  if (!_ingressMutex) return;
  xSemaphoreTake(_ingressMutex, portMAX_DELAY);
  if (_workerTask) xTaskNotifyGive(_workerTask);
  xSemaphoreGive(_ingressMutex);
}

void GcodeHost::setCapture(bool enabled) {
  xSemaphoreTake(_ingressMutex, portMAX_DELAY);
  _capture = enabled;
  xSemaphoreGive(_ingressMutex);
}

void GcodeHost::startWorker(bool paused) {
  // Called with the host lock, after the main task has acquired/opened the
  // source. Only the worker reads it; the main task closes it after completion.
  xSemaphoreTake(_ingressMutex, portMAX_DELAY);
  _capture = false;
  xQueueReset(_workerRx);
  xQueueReset(_workerControls);
  _workerOverflow.store(false);
  _workerEmergency.store(false);
  _rxTailNext = _rxTailSize = 0;
  _rxBytes = _rxPackets = _rxQueueOverflows = 0;
  _packetMismatches = 0;
  memset(_uartErrors, 0, sizeof(_uartErrors));
  ++_generation;
  _captureOrigin = _marlinOutput;
  _capture = true;
  xSemaphoreGive(_ingressMutex);
  _uiSize = 0;
  _uiDiscard = false;
  _workerStart = true;
  _workerPause = paused;
  _workerSDLease.store(_fsType == TYPE_SD_STREAM);
  _workerActive.store(true);
  notifyWorker();
}

bool GcodeHost::captureSerial(ESP3DClientType origin, const uint8_t* data,
                              size_t size) {
  if (!_ingressMutex) return false;
  xSemaphoreTake(_ingressMutex, portMAX_DELAY);
  bool captured = _capture && origin == _captureOrigin;
  if (captured) {
    for (size_t i = 0; i < size; ++i) {
      _rxTail[_rxTailNext] = data[i];
      _rxTailNext = (_rxTailNext + 1) % sizeof(_rxTail);
      if (_rxTailSize < sizeof(_rxTail)) ++_rxTailSize;
    }
    _rxBytes += size;
    while (size) {
      RxPacket packet{};
      packet.origin = origin;
      packet.generation = _generation;
      packet.size = size > sizeof(packet.data) ? sizeof(packet.data) : size;
      memcpy(packet.data, data, packet.size);
      packet.checksum = packetChecksum(packet);
      if (xQueueSend(_workerRx, &packet, 0) != pdTRUE) {
        // Never silently lose a protocol byte. UI congestion has a different,
        // lossy queue and cannot cause this overflow.
        _workerOverflow.store(true);
        ++_rxQueueOverflows;
        break;
      }
      ++_rxPackets;
      data += packet.size;
      size -= packet.size;
    }
    if (_workerTask) xTaskNotifyGive(_workerTask);
  }
  xSemaphoreGive(_ingressMutex);
  return captured;
}

bool GcodeHost::controlWorker(Control control) {
  if (xQueueSend(_workerControls, &control, 0) != pdTRUE) return false;
  notifyWorker();
  return true;
}

void GcodeHost::queueWorkerUI(const RxPacket& packet) {
  // Drop whole display lines when HTTP is slow. Dropping arbitrary chunks could
  // splice unrelated temperature/status reports together in the WebUI parser.
  for (size_t i = 0; i < packet.size; ++i) {
    if (_uiSize < sizeof(_uiLine))
      _uiLine[_uiSize++] = packet.data[i];
    else
      _uiDiscard = true;
    if (packet.data[i] != '\n') continue;
    const size_t fragments =
        (_uiSize + sizeof(packet.data) - 1) / sizeof(packet.data);
    if (!_uiDiscard && uxQueueSpacesAvailable(_workerUI) >= fragments) {
      for (size_t offset = 0; offset < _uiSize;) {
        RxPacket display{};
        display.origin = packet.origin;
        display.generation = packet.generation;
        display.size = _uiSize - offset > sizeof(display.data)
                           ? sizeof(display.data)
                           : _uiSize - offset;
        memcpy(display.data, _uiLine + offset, display.size);
        xQueueSend(_workerUI, &display,
                   0);  // Single producer, capacity reserved above.
        offset += display.size;
      }
    }
    _uiSize = 0;
    _uiDiscard = false;
  }
}

void GcodeHost::publishWorkerUI() {
  if (!_workerUI) return;
  RxPacket packet;
  // Bounded drain; only this main-loop function enters the network dispatcher.
  for (unsigned i = 0; i < 16 && xQueueReceive(_workerUI, &packet, 0) == pdTRUE;
       ++i) {
    esp3d_commands.dispatch(
        packet.data, packet.size, ESP3DClientType::all_clients, no_id,
        ESP3DMessageType::core, packet.origin, ESP3DAuthenticationLevel::admin);
  }
}

void GcodeHost::workerEntry(void* context) {
  auto& host = *static_cast<GcodeHost*>(context);
  for (;;) {
    ulTaskNotifyTake(pdTRUE, host._workerActive.load() ? 1 : portMAX_DELAY);
    {
      Guard guard(host);
      if (host._workerActive.load()) {
        if (host._workerStart) {
          host._workerStart = false;
          if (!host._marlin.begin(sendMarlin, &host, millis())) {
            host._error = static_cast<uint8_t>(host._marlin.error());
            host._step = HOST_ERROR_STREAM;
          } else {
            if (host._workerPause) host._marlin.pause();
            host._step = HOST_WAIT4_ACK;
          }
        }
        if (host._workerEmergency.exchange(false) ||
            host._workerShutdown.load())
          host._marlin.fail(MarlinStream::Error::Printer,
                            "emergency stop / service shutdown");
        if (host._workerOverflow.exchange(false))
          host._marlin.fail(MarlinStream::Error::Ack,
                            "stream receive queue overflow");
        // Controls are independent of RX capacity; emergency requests cannot
        // wait behind a full RX queue or ordinary pause/cancel requests.
        Control control;
        for (unsigned i = 0; i < 8 && xQueueReceive(host._workerControls,
                                                    &control, 0) == pdTRUE;
             ++i) {
          switch (control) {
            case Control::Pause:
              host._marlin.pause();
              break;
            case Control::Resume:
              host._marlin.resume();
              break;
            case Control::Cancel:
              host._marlin.cancel(millis());
              break;
          }
        }
        RxPacket packet;
        for (unsigned i = 0;
             i < 32 && xQueueReceive(host._workerRx, &packet, 0) == pdTRUE;
             ++i) {
          if (packet.generation != host._generation) continue;
          if (packet.size > sizeof(packet.data) ||
              packet.checksum != packetChecksum(packet)) {
            ++host._packetMismatches;
            host._marlin.fail(MarlinStream::Error::Ack,
                              "UART worker packet integrity mismatch");
            break;
          }
          host._marlin.receive(packet.data, packet.size, millis());
          host.queueWorkerUI(packet);
        }
        if (host._workerOverflow.exchange(false))
          host._marlin.fail(MarlinStream::Error::Ack,
                            "stream receive queue overflow");
        host.handleMarlin();
        if (host._step == HOST_STOP_STREAM || host._step == HOST_ERROR_STREAM) {
          host.setCapture(false);
          host._workerActive.store(false);
        }
      }
      if (host._workerShutdown.load()) break;
    }
    // Let the idle task and system Wi-Fi tasks run even during continuous RX.
    if (host._workerActive.load()) vTaskDelay(1);
  }
  xSemaphoreGive(host._workerStopped);
  vTaskDelete(nullptr);
}

void GcodeHost::stopWorker() {
  if (!_workerTask) return;
  // Do not hold the host lock while waiting for its owning task to stop.
  _workerShutdown.store(true);
  setCapture(false);
  notifyWorker();
  xSemaphoreTake(_workerStopped, portMAX_DELAY);
  xSemaphoreTake(_ingressMutex, portMAX_DELAY);
  _workerTask = nullptr;
  xSemaphoreGive(_ingressMutex);
}
#endif
