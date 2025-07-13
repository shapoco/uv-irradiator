#include <hardware/adc.h>
#include <hardware/clocks.h>
#include <hardware/i2c.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>

#include "LovyanGFX.hpp"

#include "uv_irradiator/hw_test.hpp"
#include "uv_irradiator/lgfx_ssd1306.hpp"

namespace uv_irradiator {

LGFX_SSD1306 display;

void hw_test_main(void) {
  float adc_offset = 0;

  for (int i = 0; i < LED_PWM_NUM_CHANNELS; i++) {
    constexpr uint32_t RESO = (1ul << LED_PWM_PRECISION);
    constexpr float DIV = SYS_CLK_FREQ_KHZ * 1000.0f / LED_PWM_FREQ_HZ / RESO;
    pwm_config pwm_cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&pwm_cfg, DIV);
    pwm_config_set_wrap(&pwm_cfg, RESO - 1);
    pwm_init(LED_PWM_CHANNELS[i], &pwm_cfg, true);
  }

  for (int i = 0; i < LED_NUM_PORTS; i++) {
    gpio_init(LED_PORTS[i]);
    gpio_set_dir(LED_PORTS[i], GPIO_OUT);
    gpio_set_function(LED_PORTS[i], GPIO_FUNC_PWM);
    pwm_set_gpio_level(LED_PORTS[i], 0);
  }

  for (int i = 0; i < SWITCH_NUM_PORTS; i++) {
    gpio_init(SWITCH_PORTS[i]);
    gpio_set_dir(SWITCH_PORTS[i], GPIO_IN);
    if (SWITCH_NEGATIVE[i]) {
      gpio_pull_up(SWITCH_PORTS[i]);
    } else {
      gpio_pull_down(SWITCH_PORTS[i]);
    }
  }

  adc_init();
  adc_gpio_init(TEMPERATURE_ADC_CHANNEL);

  {
    gpio_init(VOLUME_HIGH_PORT);
    gpio_set_dir(VOLUME_HIGH_PORT, GPIO_IN);
    adc_select_input(VOLUME_ADC_CHANNEL);
    for (int i = 0; i < 8; i++) {
      uint16_t dummy = adc_read();
    }
    uint32_t sum = 0;
    constexpr int NUM_SUM = 64;
    for (int i = 0; i < NUM_SUM; i++) {
      sum += adc_read();
    }
    adc_offset = (float)sum / NUM_SUM;
    gpio_set_dir(VOLUME_HIGH_PORT, GPIO_OUT);
    gpio_put(VOLUME_HIGH_PORT, true);
  }

  {
    constexpr uint32_t RESO = (1ul << FAN_PWM_PRECISION);
    constexpr float DIV = SYS_CLK_FREQ_KHZ * 1000.0f / FAN_PWM_FREQ_HZ / RESO;
    pwm_config pwm_cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&pwm_cfg, DIV);
    pwm_config_set_wrap(&pwm_cfg, RESO - 1);
    pwm_init(pwm_gpio_to_slice_num(FAN_PWM_PORT), &pwm_cfg, true);
    gpio_init(FAN_PWM_PORT);
    gpio_set_dir(FAN_PWM_PORT, GPIO_OUT);
    gpio_set_function(FAN_PWM_PORT, GPIO_FUNC_PWM);
    pwm_set_gpio_level(FAN_PWM_PORT, 0);
  }

  display.resetI2cBus();

  display.init();
  display.setRotation(DISPLAY_ROTATION);
  display.setColorDepth(1);
  display.clear();

  int text_size = 1;
  int x_value = 56;
  int line_height = text_size * 8 + 1;

  {
    display.setTextColor(Color::WHITE, Color::BLACK);
    display.setTextSize(text_size);
    int y = 0;

    display.setCursor(0, y);
    display.print("ADC OFST");
    y += line_height;

    display.setCursor(0, y);
    display.print("LEDs");
    y += line_height;

    display.setCursor(0, y);
    display.print("SWITCHes");
    y += line_height;

    display.setCursor(0, y);
    display.print("VOLUME");
    y += line_height;

    display.setCursor(0, y);
    display.print("TEMP.");
    y += line_height;

    display.setCursor(0, y);
    display.print("FAN SPD.");
    y += line_height;

    display.setCursor(0, y);
    display.print("CURRENT");
    y += line_height;
  }

  while (true) {
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    display.setTextColor(Color::WHITE, Color::BLACK);
    display.setTextSize(text_size);
    int y = 0;

    {
      float offset_mv = adc_offset * 3300.0f / (1 << 12);
      display.setCursor(x_value, y);
      display.printf("%3.1f (%3.1f mV)", adc_offset, offset_mv);
      y += line_height;
    }

    {
      constexpr int SLOPE_PERIOD = 1000;
      constexpr int PHASE_PERIOD = LED_NUM_PORTS + 2;
      constexpr uint32_t RESO = (1ul << LED_PWM_PRECISION);

      int phase_index =
          PHASE_PERIOD - 1 - (now_ms / (SLOPE_PERIOD * 2) % PHASE_PERIOD);
      int t = now_ms % (SLOPE_PERIOD * 2);
      if (t >= SLOPE_PERIOD) {
        t = SLOPE_PERIOD * 2 - t;
      }
      uint32_t brightness = (uint32_t)t * RESO / SLOPE_PERIOD;
      uint32_t pwm_level = (brightness * brightness) >> LED_PWM_PRECISION;
      pwm_level = (pwm_level * brightness) >> LED_PWM_PRECISION;

      int x = x_value;
      for (int i = LED_NUM_PORTS - 1; i >= 0; i--) {
        bool on = (phase_index >= LED_NUM_PORTS) || (i == phase_index);

        pwm_set_gpio_level(LED_PORTS[i], on ? pwm_level : 0);

        int r = line_height / 2 - 1;
        int cx = x + r;
        int cy = y + r;
        display.drawEllipse(cx, cy, r, r, Color::WHITE);
        display.fillEllipse(cx, cy, r - 1, r - 1,
                            on ? Color::WHITE : Color::BLACK);
        x += r * 2 + 2;
      }
      y += line_height;
    }

    {
      int x = x_value;
      for (int i = 0; i < SWITCH_NUM_PORTS; i++) {
        bool on = gpio_get(SWITCH_PORTS[i]) ^ SWITCH_NEGATIVE[i];
        int r = line_height / 2 - 1;
        int cx = x + r;
        int cy = y + r;
        display.drawEllipse(cx, cy, r, r, Color::WHITE);
        display.fillEllipse(cx, cy, r - 1, r - 1,
                            on ? Color::WHITE : Color::BLACK);
        x += r * 2 + 2;
      }
      y += line_height;
    }

    {
      adc_select_input(VOLUME_ADC_CHANNEL);
      float raw = adc_read() - adc_offset;
      int w = DISPLAY_WIDTH - x_value;
      int h = text_size * 8;
      int x_white = x_value + 1;
      int w_white = raw * (w - 2) / (1 << 12);
      int x_black = x_white + w_white;
      int w_black = (w - 2) - w_white;
      display.drawRect(x_value, y, w, h, Color::WHITE);
      display.fillRect(x_white, y + 1, w_white, h - 2, Color::WHITE);
      display.fillRect(x_black, y + 1, w_black, h - 2, Color::BLACK);
      y += line_height;
    }

    {
      adc_select_input(TEMPERATURE_ADC_CHANNEL);
      float raw = adc_read() - adc_offset;

      constexpr int NUM_LOG = 64;
      static float log[NUM_LOG];
      static int log_index = 0;
      log[log_index] = raw;
      log_index = (log_index + 1) % NUM_LOG;
      float sum = 0;
      for (int i = 0; i < NUM_LOG; i++) {
        sum += log[i];
      }
      raw = sum / NUM_LOG;

      float voltage = (raw * 3.3f) / (1 << 12);
      float temperature = (voltage - 0.424f) / 0.00625f;
      display.setCursor(x_value, y);
      display.printf("%6.2f degC", temperature);
      y += line_height;
    }

    {
      constexpr uint32_t RESO = (1ul << FAN_PWM_PRECISION);
      constexpr int SLOPE_PERIOD = 5000;

      int t = now_ms % SLOPE_PERIOD * 4;
      if (t < SLOPE_PERIOD) {
        // nothing to do
      } else if (t < SLOPE_PERIOD * 2) {
        t = SLOPE_PERIOD - 1;
      } else if (t < SLOPE_PERIOD * 3) {
        t = SLOPE_PERIOD * 3 - t;
      } else {
        t = 0;
      }

      int fan_speed = 0;
      int pwm_level = 0;
      if (t > 0) {
        fan_speed = t * RESO / SLOPE_PERIOD;
        pwm_level = FAN_MIN_SPEED + t * (RESO - FAN_MIN_SPEED) / SLOPE_PERIOD;
      }
      pwm_set_gpio_level(FAN_PWM_PORT, pwm_level);

      int w = DISPLAY_WIDTH - x_value;
      int h = text_size * 8;
      int x_white = x_value + 1;
      int w_white = (uint32_t)fan_speed * (w - 2) / RESO;
      int x_black = x_white + w_white;
      int w_black = (w - 2) - w_white;
      display.drawRect(x_value, y, w, h, Color::WHITE);
      display.fillRect(x_white, y + 1, w_white, h - 2, Color::WHITE);
      display.fillRect(x_black, y + 1, w_black, h - 2, Color::BLACK);
      y += line_height;
    }

    {
      adc_select_input(CURRENT_SENSOR_ADC_CHANNEL);

      constexpr int NUM_SUM = 64;
      float sum = 0;
      for (int i = 0; i < NUM_SUM; i++) {
        sum += adc_read() - adc_offset;
      }
      float raw = sum / NUM_SUM;

      float voltage = (raw * 3.3f) / (1 << 12);
      voltage -= CURRENT_SENSOR_OFFSET_MV / 1000.0f;
      float current = voltage / (0.1f * 11);
      display.setCursor(x_value, y);
      display.printf("%6.3f A", current);
      y += line_height;
    }

    display.display();
    sleep_ms(10);
  }
}

}  // namespace uv_irradiator
