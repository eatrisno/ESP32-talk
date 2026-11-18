#pragma once
// Status LED (WS2812). Shows the most important state; short flashes override it.
#include <cstdint>

namespace led {

enum class Color : uint8_t { Off, Red, Green, Blue, Amber, Cyan, White };

void start();
void set(Color steady);            // background state
void flash(Color c, uint16_t ms);  // overrides the steady colour briefly

}  // namespace led
