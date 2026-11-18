#include "led.hpp"

#include "config.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

namespace led {
namespace {

led_strip_handle_t s_strip;
volatile Color s_steady = Color::Off, s_flash = Color::Off;
volatile int64_t s_flash_until = 0;

void rgb(Color c, uint8_t& r, uint8_t& g, uint8_t& b) {
    const uint8_t k = cfg::kLedBrightness;
    r = g = b = 0;
    switch (c) {
        case Color::Red: r = k; break;
        case Color::Green: g = k; break;
        case Color::Blue: b = k; break;
        case Color::Amber: r = k; g = (k > 1) ? (k / 2) : 1; break;
        case Color::Cyan: g = k; b = k; break;
        case Color::White: r = g = b = k; break;
        case Color::Off: break;
    }
}

void task(void*) {
    Color shown = Color::White;
    for (;;) {
        Color c = esp_timer_get_time() < s_flash_until ? s_flash : s_steady;
        if (c != shown) {
            uint8_t r, g, b;
            rgb(c, r, g, b);
            led_strip_set_pixel(s_strip, 0, r, g, b);
            led_strip_refresh(s_strip);
            shown = c;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

}  // namespace

void start() {
    led_strip_config_t sc = {};
    sc.strip_gpio_num = cfg::kPinLed;
    sc.max_leds = 1;
    sc.led_model = LED_MODEL_WS2812;
    sc.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    led_strip_rmt_config_t rc = {};
    rc.resolution_hz = 10 * 1000 * 1000;
    if (led_strip_new_rmt_device(&sc, &rc, &s_strip) != ESP_OK) return;  // no LED: run without
    led_strip_clear(s_strip);
    xTaskCreatePinnedToCore(task, "led", 2560, nullptr, 2, nullptr, 0);
}

void set(Color c) { s_steady = c; }

void flash(Color c, uint16_t ms) {
    s_flash = c;
    s_flash_until = esp_timer_get_time() + static_cast<int64_t>(ms) * 1000;
}

}  // namespace led
