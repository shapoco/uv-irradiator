#include <hardware/clocks.h>
#include <pico/stdlib.h>

#include "uv_irradiator/hw_test.hpp"

int main() {
  set_sys_clock_khz(uv_irradiator::SYS_CLK_FREQ_KHZ, true);
  sleep_ms(100);

  stdio_init_all();
  sleep_ms(100);

  uv_irradiator::hw_test_main();

  return 0;
}
