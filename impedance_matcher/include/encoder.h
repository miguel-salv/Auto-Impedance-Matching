#pragma once

#include <Arduino.h>

void encoder_init();
void encoder_poll(int* outDelta, bool* outPressed);
