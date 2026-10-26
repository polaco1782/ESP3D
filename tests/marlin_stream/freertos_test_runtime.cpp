// Host substitute for scheduling/queues only. The production worker task entry
// runs on a real std::thread; these fakes do not execute protocol logic.
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

struct TestSemaphore {
  int kind;
  std::timed_mutex normal;
  std::recursive_timed_mutex recursive;
  std::mutex mutex;
  std::condition_variable cv;
  bool signalled = false;
};
struct TestQueue {
  size_t capacity, itemSize;
  std::mutex mutex;
  std::deque<std::vector<uint8_t>> items;
};
struct TestTask {
  std::thread thread;
  std::mutex mutex;
  std::condition_variable cv;
  unsigned notifications = 0;
  int core;
};
struct Registry {
  std::vector<std::unique_ptr<TestSemaphore>> semaphores;
  std::vector<std::unique_ptr<TestQueue>> queues;
  std::vector<std::unique_ptr<TestTask>> tasks;
  ~Registry() {
    for (auto& t : tasks)
      if (t->thread.joinable()) t->thread.join();
  }
};
static Registry& registry() {
  static Registry r;
  return r;
}
static thread_local TestTask* currentTask = nullptr;
#ifndef TEST_MAIN_CORE
#define TEST_MAIN_CORE 0
#endif
int testMainCore = TEST_MAIN_CORE;
bool failTaskCreation = false;

static SemaphoreHandle_t semaphore(int kind) {
  std::unique_ptr<TestSemaphore> s(new TestSemaphore{});
  s->kind = kind;
  auto ptr = s.get();
  registry().semaphores.push_back(std::move(s));
  return ptr;
}
SemaphoreHandle_t xSemaphoreCreateMutex() { return semaphore(0); }
SemaphoreHandle_t xSemaphoreCreateRecursiveMutex() { return semaphore(1); }
SemaphoreHandle_t xSemaphoreCreateBinary() { return semaphore(2); }
BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t ticks) {
  if (s->kind == 2) {
    std::unique_lock<std::mutex> lock(s->mutex);
    if (ticks == portMAX_DELAY)
      s->cv.wait(lock, [&] { return s->signalled; });
    else if (!s->cv.wait_for(lock, std::chrono::milliseconds(ticks),
                             [&] { return s->signalled; }))
      return pdFALSE;
    s->signalled = false;
    return pdTRUE;
  }
  if (s->kind == 1) {
    if (ticks == portMAX_DELAY) {
      s->recursive.lock();
      return pdTRUE;
    }
    return s->recursive.try_lock_for(std::chrono::milliseconds(ticks));
  }
  if (ticks == portMAX_DELAY) {
    s->normal.lock();
    return pdTRUE;
  }
  return s->normal.try_lock_for(std::chrono::milliseconds(ticks));
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t s) {
  if (s->kind == 2) {
    std::lock_guard<std::mutex> lock(s->mutex);
    s->signalled = true;
    s->cv.notify_one();
  } else if (s->kind == 1)
    s->recursive.unlock();
  else
    s->normal.unlock();
  return pdTRUE;
}
// Reclaim after all test threads have joined, so a host condition_variable may
// finish notifying a waiter even when FreeRTOS would already permit deletion.
void vSemaphoreDelete(SemaphoreHandle_t) {}
QueueHandle_t xQueueCreate(UBaseType_t capacity, UBaseType_t size) {
  std::unique_ptr<TestQueue> q(new TestQueue{});
  q->capacity = capacity;
  q->itemSize = size;
  auto ptr = q.get();
  registry().queues.push_back(std::move(q));
  return ptr;
}
BaseType_t xQueueSend(QueueHandle_t q, const void* value, TickType_t ticks) {
  assert(ticks == 0);
  std::lock_guard<std::mutex> lock(q->mutex);
  if (q->items.size() == q->capacity) return pdFALSE;
  auto bytes = static_cast<const uint8_t*>(value);
  q->items.emplace_back(bytes, bytes + q->itemSize);
  return pdTRUE;
}
BaseType_t xQueueReceive(QueueHandle_t q, void* value, TickType_t ticks) {
  assert(ticks == 0);
  std::lock_guard<std::mutex> lock(q->mutex);
  if (q->items.empty()) return pdFALSE;
  memcpy(value, q->items.front().data(), q->itemSize);
  q->items.pop_front();
  return pdTRUE;
}
void xQueueReset(QueueHandle_t q) {
  std::lock_guard<std::mutex> lock(q->mutex);
  q->items.clear();
}
void vQueueDelete(QueueHandle_t) {}
UBaseType_t uxQueueSpacesAvailable(QueueHandle_t q) {
  std::lock_guard<std::mutex> lock(q->mutex);
  return q->capacity - q->items.size();
}
struct TaskExit {};
BaseType_t xTaskCreatePinnedToCore(void (*entry)(void*), const char*,
                                   uint32_t stack, void* arg,
                                   UBaseType_t priority, TaskHandle_t* handle,
                                   BaseType_t core) {
  if (failTaskCreation) return pdFALSE;
  assert(stack == 6144 && priority == 2 && core >= 0 && core <= 1);
  std::unique_ptr<TestTask> t(new TestTask{});
  t->core = core;
  *handle = t.get();
  auto ptr = t.get();
  registry().tasks.push_back(std::move(t));
  ptr->thread = std::thread([=] {
    currentTask = ptr;
    try {
      entry(arg);
      assert(false && "task returned instead of deleting itself");
    } catch (TaskExit&) {
    }
  });
  return pdPASS;
}
void xTaskNotifyGive(TaskHandle_t t) {
  std::lock_guard<std::mutex> lock(t->mutex);
  ++t->notifications;
  t->cv.notify_one();
}
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ticks) {
  auto t = currentTask;
  assert(t);
  std::unique_lock<std::mutex> lock(t->mutex);
  if (ticks == portMAX_DELAY)
    t->cv.wait(lock, [&] { return t->notifications; });
  else
    t->cv.wait_for(lock, std::chrono::milliseconds(ticks),
                   [&] { return t->notifications; });
  auto n = t->notifications;
  if (clear)
    t->notifications = 0;
  else if (n)
    --t->notifications;
  return n;
}
void vTaskDelay(TickType_t ticks) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ticks));
}
void vTaskDelete(TaskHandle_t t) {
  assert(!t && currentTask);
  throw TaskExit{};
}
int xPortGetCoreID() { return currentTask ? currentTask->core : testMainCore; }
void joinTestTasks() {
  for (auto& t : registry().tasks)
    if (t->thread.joinable()) t->thread.join();
}
