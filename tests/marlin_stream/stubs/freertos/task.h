#pragma once
#include "FreeRTOS.h"
struct TestTask;
using TaskHandle_t = TestTask*;
BaseType_t xTaskCreatePinnedToCore(void (*)(void*), const char*, uint32_t,
                                   void*, UBaseType_t, TaskHandle_t*,
                                   BaseType_t);
void xTaskNotifyGive(TaskHandle_t);
uint32_t ulTaskNotifyTake(BaseType_t, TickType_t);
void vTaskDelay(TickType_t);
void vTaskDelete(TaskHandle_t);
int xPortGetCoreID();
void joinTestTasks();
extern int testMainCore;
extern bool failTaskCreation;
