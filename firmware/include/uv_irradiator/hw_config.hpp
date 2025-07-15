#pragma once

namespace uv_irradiator {

static constexpr int SYS_CLK_FREQ_KHZ = 125 * 1000;

static constexpr int DISPLAY_WIDTH = 128;
static constexpr int DISPLAY_HEIGHT = 64;
static constexpr int DISPLAY_ROTATION = 2;
static constexpr int DISPLAY_I2C_PORT = 0;
static constexpr int DISPLAY_I2C_SDA_PORT = 16;
static constexpr int DISPLAY_I2C_SCL_PORT = 17;
static constexpr uint8_t DISPLAY_I2C_DEV_ADDR = 0x3c;
static constexpr uint32_t DISPLAY_I2C_FREQ_HZ = 400 * 1000;

enum Color : uint16_t {
  BLACK = 0x0000,
  WHITE = 0xffff,
};

static constexpr int UVLED_NUM_PORTS = 6;
static constexpr int UVLED_PORTS[] = {4, 5, 6, 7, 8, 9};
static constexpr int UVLED_PWM_NUM_CHANNELS = 3;
static constexpr int UVLED_PWM_CHANNELS[] = {2, 3, 4};
static constexpr int UVLED_PWM_FREQ_HZ = 10 * 1000;
static constexpr int UVLED_PWM_PRECISION = 12;

static constexpr int SWITCH_NUM_PORTS = 4;
static constexpr int SWITCH_PORTS[] = {18, 19, 20, 21};
static constexpr int SWITCH_NEGATIVE[] = {true, true, true, false};

static constexpr int FAULT_LED_PORT = 2;
static constexpr int UNLOCK_LED_PORT = 3;

static constexpr int FAN_PWM_PORT = 10;
static constexpr int FAN_PWM_FREQ_HZ = 25 * 1000;
static constexpr int FAN_PWM_PRECISION = 8;
static constexpr int FAN_MIN_SPEED = (1ul << FAN_PWM_PRECISION) / 4;

static constexpr int SENSOR_ENA_PORT = 22;

static constexpr int VOLUME_PORT = 26;
static constexpr int CURRENT_PORT = 27;
static constexpr int TEMPERATURE_PORT = 28;

static constexpr int VOLUME_ADC_CHANNEL = 0;
static constexpr int CURRENT_SENSOR_ADC_CHANNEL = 1;
static constexpr int TEMPERATURE_ADC_CHANNEL = 2;

}  // namespace uv_irradiator
