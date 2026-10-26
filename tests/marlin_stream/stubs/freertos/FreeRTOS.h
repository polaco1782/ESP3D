#pragma once
#include <stddef.h>
#include <stdint.h>
using BaseType_t = int;
using UBaseType_t = unsigned;
using TickType_t = uint32_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) (ms)
#ifndef CONFIG_FREERTOS_UNICORE
#define CONFIG_FREERTOS_UNICORE 0
#endif
