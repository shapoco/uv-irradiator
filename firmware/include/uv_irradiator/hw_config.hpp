#pragma once

namespace uv_irradiator {

static constexpr int SYS_CLK_FREQ_KHZ = 125 * 1000;

static constexpr int DISPLAY_WIDTH = 128;
static constexpr int DISPLAY_HEIGHT = 64;
static constexpr int DISPLAY_ROTATION = 2;
static constexpr int DISPLAY_I2C_PORT = 1;
static constexpr int DISPLAY_I2C_SDA_PORT = 14;
static constexpr int DISPLAY_I2C_SCL_PORT = 15;
static constexpr uint8_t DISPLAY_I2C_DEV_ADDR = 0x3c;
static constexpr uint32_t DISPLAY_I2C_FREQ_HZ = 400 * 1000;

enum Color : uint16_t {
  BLACK = 0x0000,
  WHITE = 0xffff,
};

static constexpr int LED_NUM_PORTS = 6;
static constexpr int LED_PORTS[] = {4, 5, 6, 7, 8, 9};
static constexpr int LED_PWM_NUM_CHANNELS = 3;
static constexpr int LED_PWM_CHANNELS[] = {2, 3, 4};
static constexpr int LED_PWM_FREQ_HZ = 10 * 1000;
static constexpr int LED_PWM_PRECISION = 12;

static constexpr int SWITCH_NUM_PORTS = 4;
static constexpr int SWITCH_PORTS[] = {16, 17, 18, 21};

static constexpr int FAN_PWM_PORT = 3;
static constexpr int FAN_PWM_FREQ_HZ = 25 * 1000;
static constexpr int FAN_PWM_PRECISION = 8;
static constexpr int FAN_MIN_SPEED = (1ul << FAN_PWM_PRECISION) / 4;

static constexpr int VOLUME_HIGH_PORT = 22;
static constexpr int CURRENT_SENSOR_OFFSET_MV = 17;

static constexpr int VOLUME_ADC_CHANNEL = 0;
static constexpr int TEMPERATURE_ADC_CHANNEL = 1;
static constexpr int CURRENT_SENSOR_ADC_CHANNEL = 2;

}  // namespace uv_irradiator
