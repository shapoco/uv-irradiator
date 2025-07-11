#pragma once

#include <stdint.h>

// #define LGFX_USE_V1
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
};

}  // namespace uv_irradiator
