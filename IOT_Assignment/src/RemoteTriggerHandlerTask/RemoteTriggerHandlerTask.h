#ifndef REMOTE_TRIGGER_HANDLER_TASK_H
#define REMOTE_TRIGGER_HANDLER_TASK_H

#include <ThingsBoard.h>
#include <array>
#include <Arduino.h>
#include "main.h"

extern volatile bool attributesChanged;
extern volatile int ledMode;
extern volatile bool ledState;
extern volatile uint16_t blinkingInterval;

extern const std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST;

RPC_Response setLedSwitchState(const RPC_Data &data);
void processSharedAttributes(const Shared_Attribute_Data &data);
void RemoteTriggerHandlerTask(void *pvParameters);

#endif // REMOTE_TRIGGER_HANDLER_TASK_H
