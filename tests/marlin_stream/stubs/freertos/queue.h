#pragma once
#include "FreeRTOS.h"
struct TestQueue;
using QueueHandle_t = TestQueue*;
QueueHandle_t xQueueCreate(UBaseType_t, UBaseType_t);
BaseType_t xQueueSend(QueueHandle_t, const void*, TickType_t);
BaseType_t xQueueReceive(QueueHandle_t, void*, TickType_t);
void xQueueReset(QueueHandle_t);
void vQueueDelete(QueueHandle_t);

UBaseType_t uxQueueSpacesAvailable(QueueHandle_t);
