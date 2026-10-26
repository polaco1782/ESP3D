#pragma once
#include "FreeRTOS.h"
struct TestSemaphore;
using SemaphoreHandle_t = TestSemaphore*;
SemaphoreHandle_t xSemaphoreCreateMutex();
SemaphoreHandle_t xSemaphoreCreateRecursiveMutex();
SemaphoreHandle_t xSemaphoreCreateBinary();
BaseType_t xSemaphoreTake(SemaphoreHandle_t, TickType_t);
BaseType_t xSemaphoreGive(SemaphoreHandle_t);
inline BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t s, TickType_t t) {
  return xSemaphoreTake(s, t);
}
inline BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t s) {
  return xSemaphoreGive(s);
}
void vSemaphoreDelete(SemaphoreHandle_t);
