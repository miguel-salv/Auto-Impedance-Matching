#pragma once

#include <Arduino.h>

bool ui_init_display();
bool ui_should_redraw(bool userInput, bool inAutoHome);
void ui_tick(int delta, bool pressed, bool doDraw);
