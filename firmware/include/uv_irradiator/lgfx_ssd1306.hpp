#pragma once

#include <pico/stdlib.h>
#include <stdint.h>

#include <LovyanGFX.hpp>

#include "uv_irradiator/hw_config.hpp"

namespace uv_irradiator {

class LGFX_SSD1306 : public lgfx::LGFX_Device {
  lgfx::Panel_SSD1306 _panel_instance;
  lgfx::Bus_I2C _bus_instance;

 public:
  LGFX_SSD1306() {
    {
      auto cfg = _bus_instance.config();
      cfg.i2c_port = DISPLAY_I2C_PORT;
      cfg.freq_write = DISPLAY_I2C_FREQ_HZ;
      cfg.freq_read = DISPLAY_I2C_FREQ_HZ;
      cfg.pin_sda = DISPLAY_I2C_SDA_PORT;
      cfg.pin_scl = DISPLAY_I2C_SCL_PORT;
      cfg.i2c_addr = DISPLAY_I2C_DEV_ADDR;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    {
      auto cfg = _panel_instance.config();
      cfg.panel_width = DISPLAY_WIDTH;
      cfg.panel_height = DISPLAY_HEIGHT;
      cfg.offset_rotation = DISPLAY_ROTATION;
      _panel_instance.config(cfg);
    }

    setPanel(&_panel_instance);
  }

  void resetI2cBus() {
    constexpr int MAX_RETRIES = 3;
    constexpr int MAX_SCL_PULSES = 16;
    constexpr int SCL_PERIOD_US = 100;

    // change I2C pins to GPIO mode
    gpio_init(DISPLAY_I2C_SDA_PORT);
    gpio_init(DISPLAY_I2C_SCL_PORT);
    gpio_pull_up(DISPLAY_I2C_SDA_PORT);
    gpio_pull_up(DISPLAY_I2C_SCL_PORT);
    gpio_set_function(DISPLAY_I2C_SDA_PORT, GPIO_FUNC_SIO);
    gpio_set_function(DISPLAY_I2C_SCL_PORT, GPIO_FUNC_SIO);
    gpio_set_dir(DISPLAY_I2C_SDA_PORT, GPIO_IN);
    gpio_set_dir(DISPLAY_I2C_SCL_PORT, GPIO_IN);
    gpio_put(DISPLAY_I2C_SDA_PORT, 0);
    gpio_put(DISPLAY_I2C_SCL_PORT, 0);

    for (int i = 0; i < MAX_RETRIES; i++) {
      // check SDA state
      if (gpio_get(DISPLAY_I2C_SDA_PORT)) {
        // SDA is high, no need to reset
        break;
      }

      // send SCL pulses until SDA is high
      gpio_set_dir(DISPLAY_I2C_SDA_PORT, GPIO_IN);
      gpio_set_dir(DISPLAY_I2C_SCL_PORT, GPIO_OUT);
      sleep_us(SCL_PERIOD_US);
      for (int j = 0; j < MAX_SCL_PULSES; j++) {
        gpio_set_dir(DISPLAY_I2C_SCL_PORT, GPIO_IN);
        sleep_us(SCL_PERIOD_US / 2);
        gpio_set_dir(DISPLAY_I2C_SCL_PORT, GPIO_OUT);
        sleep_us(SCL_PERIOD_US / 2);
        if (gpio_get(DISPLAY_I2C_SDA_PORT)) break;
      }

      // send stop condition
      gpio_set_dir(DISPLAY_I2C_SDA_PORT, GPIO_OUT);
      sleep_us(SCL_PERIOD_US);
      gpio_set_dir(DISPLAY_I2C_SCL_PORT, GPIO_IN);
      sleep_us(SCL_PERIOD_US);
      gpio_set_dir(DISPLAY_I2C_SDA_PORT, GPIO_IN);
      sleep_us(SCL_PERIOD_US);
    }

    // cleanup
    gpio_set_function(DISPLAY_I2C_SDA_PORT, GPIO_FUNC_I2C);
    gpio_set_function(DISPLAY_I2C_SCL_PORT, GPIO_FUNC_I2C);
  }
};

}  // namespace uv_irradiator
