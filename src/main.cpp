
#define USE_SPI

#ifdef USE_SPI
#include "../examples/led_strip_spi_ws2812/main/led_strip_spi_ws2812_main.c"
#else
#include "../examples/led_strip_rmt_ws2812/main/led_strip_rmt_ws2812_main.c"
#endif

void setup() {}
void loop() {
    app_main();
}