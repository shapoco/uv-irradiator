#pragma once

#include "uv_irradiator/hw_config.hpp"

#include "LovyanGFX.hpp"
#include "uv_irradiator/lgfx_ssd1306.hpp"

namespace uv_irradiator {

extern LGFX_SSD1306 display;

void hw_test_main(void);

}
