#include <hardware/adc.h>
#include <hardware/clocks.h>
#include <hardware/i2c.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>

#include "uv_irradiator/hw_test.hpp"

namespace uv_irradiator {

static float adc_read_average(int channel, int num_samples) {
  adc_select_input(channel);
  uint32_t sum = 0;
  for (int i = 0; i < num_samples; i++) {
    sum += adc_read();
  }
  return (float)sum / num_samples;
}

void hw_test_main(void) {
  float adc_offset_lsbs = 0;
  float amp_offset_volt = 0;

  // 紫外線 LED ポート初期化
  for (int i = 0; i < UVLED_PWM_NUM_CHANNELS; i++) {
    constexpr uint32_t RESO = (1ul << UVLED_PWM_PRECISION);
    constexpr float DIV = SYS_CLK_FREQ_KHZ * 1000.0f / UVLED_PWM_FREQ_HZ / RESO;
    pwm_config pwm_cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&pwm_cfg, DIV);
    pwm_config_set_wrap(&pwm_cfg, RESO - 1);
    pwm_init(UVLED_PWM_CHANNELS[i], &pwm_cfg, true);
  }

  // UV LED ポートの初期化
  for (int i = 0; i < UVLED_NUM_PORTS; i++) {
    gpio_init(UVLED_PORTS[i]);
    gpio_set_dir(UVLED_PORTS[i], GPIO_OUT);
    gpio_set_function(UVLED_PORTS[i], GPIO_FUNC_PWM);
    pwm_set_gpio_level(UVLED_PORTS[i], 0);
  }

  // インジケータ初期化
  gpio_init(ERR_LED_PORT);
  gpio_init(OUT_LED_PORT);
  gpio_set_dir(ERR_LED_PORT, GPIO_OUT);
  gpio_set_dir(OUT_LED_PORT, GPIO_OUT);
  gpio_put(ERR_LED_PORT, true);
  gpio_put(OUT_LED_PORT, true);

  // スイッチ類の初期化
  for (int i = 0; i < SWITCH_NUM_PORTS; i++) {
    gpio_init(SWITCH_PORTS[i]);
    gpio_set_dir(SWITCH_PORTS[i], GPIO_IN);
    if (SWITCH_NEGATIVE[i]) {
      gpio_pull_up(SWITCH_PORTS[i]);
    } else {
      gpio_pull_down(SWITCH_PORTS[i]);
    }
  }

  // ADC初期化
  // センサ類のポートを使ってアナログ系の特性を取得しておく
  {
    adc_init();

    // ボリュームと温度計の電源を切る
    gpio_init(SENSOR_ENA_PORT);
    gpio_set_dir(SENSOR_ENA_PORT, GPIO_IN);
    sleep_ms(100);

    // 温度センサ端子の電荷を抜く (一応)
    gpio_put(TEMPERATURE_PORT, false);
    gpio_set_dir(TEMPERATURE_PORT, GPIO_OUT);
    sleep_ms(1);
    gpio_set_dir(TEMPERATURE_PORT, GPIO_IN);
    sleep_ms(100);

    adc_gpio_init(VOLUME_PORT);
    adc_gpio_init(CURRENT_PORT);
    adc_gpio_init(TEMPERATURE_PORT);

    // ADC オフセット電圧測定
    adc_offset_lsbs = adc_read_average(TEMPERATURE_ADC_CHANNEL, 64);

    // オペアンプ入力オフセット測定
    amp_offset_volt =
        (adc_read_average(VOLUME_ADC_CHANNEL, 64) - adc_offset_lsbs) *
        (3.3f / (1 << 12));

    // ボリュームと温度計の電源投入
    gpio_set_dir(SENSOR_ENA_PORT, GPIO_OUT);
    gpio_put(SENSOR_ENA_PORT, true);
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
  int x_value = 32;
  int line_height = text_size * 8 + 1;

  {
    display.setTextColor(Color::WHITE, Color::BLACK);
    display.setTextSize(text_size);
    int y = 0;

    display.setCursor(0, y);
    display.print("OFST");
    y += line_height;

    display.setCursor(0, y);
    display.print("LED");
    y += line_height;

    display.setCursor(0, y);
    display.print("SW");
    y += line_height;

    display.setCursor(0, y);
    display.print("VOL");
    y += line_height;

    display.setCursor(0, y);
    display.print("TEMP");
    y += line_height;

    display.setCursor(0, y);
    display.print("FAN");
    y += line_height;

    display.setCursor(0, y);
    display.print("CRNT");
    y += line_height;
  }

  while (true) {
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    {
      int t = now_ms / 1024;
      gpio_put(OUT_LED_PORT, !!(t & 1));
      gpio_put(ERR_LED_PORT, !!(t & 2));
    }

    display.setTextColor(Color::WHITE, Color::BLACK);
    display.setTextSize(text_size);
    int y = 0;

    {
      float adc_offset_mv = adc_offset_lsbs * 3300.0f / (1 << 12);
      display.setCursor(x_value, y);
      display.printf("%4.2fmV, %4.2fmV", adc_offset_mv, amp_offset_volt * 1000);
      y += line_height;
    }

    {
      constexpr int SLOPE_PERIOD = 1000;
      constexpr int PHASE_PERIOD = UVLED_NUM_PORTS + 2;
      constexpr uint32_t RESO = (1ul << UVLED_PWM_PRECISION);

      int phase_index =
          PHASE_PERIOD - 1 - (now_ms / (SLOPE_PERIOD * 2) % PHASE_PERIOD);
      int t = now_ms % (SLOPE_PERIOD * 2);
      if (t >= SLOPE_PERIOD) {
        t = SLOPE_PERIOD * 2 - t;
      }
      uint32_t brightness = (uint32_t)t * RESO / SLOPE_PERIOD;
      uint32_t pwm_level = (brightness * brightness) >> UVLED_PWM_PRECISION;
      pwm_level = (pwm_level * brightness) >> UVLED_PWM_PRECISION;

      int x = x_value;
      for (int i = UVLED_NUM_PORTS - 1; i >= 0; i--) {
        bool on = (phase_index >= UVLED_NUM_PORTS) || (i == phase_index);

        pwm_set_gpio_level(UVLED_PORTS[i], on ? pwm_level : 0);

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
      float raw = adc_read_average(VOLUME_ADC_CHANNEL, 1) - adc_offset_lsbs;

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
      float raw =
          adc_read_average(TEMPERATURE_ADC_CHANNEL, 16) - adc_offset_lsbs;

#if false
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
#endif

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
      constexpr float AMP_RH = 100 * 1000;
      constexpr float AMP_RL = 10 * 1000;
      constexpr float AMP_GAIN = (AMP_RH + AMP_RL) / AMP_RL;
      constexpr float R_SHUNT = 0.1f;

      float raw =
          adc_read_average(CURRENT_SENSOR_ADC_CHANNEL, 16) - adc_offset_lsbs;
      float voltage = (raw * 3.3f) / (1 << 12) - amp_offset_volt;
      float current = voltage / (R_SHUNT * AMP_GAIN);
      display.setCursor(x_value, y);
      display.printf("%6.3f A", current);
      y += line_height;
    }

    display.display();
    sleep_ms(10);
  }
}

}  // namespace uv_irradiator
