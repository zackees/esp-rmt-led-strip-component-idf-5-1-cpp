
/// #define USE_SPI

#include <Arduino.h>

#ifdef USE_SPI
#include "main_spi.hpp"
#else
#include "main_rmt.hpp"
#endif



void setup() {
    delay(3000);
}
void loop() {
    app_main();
}

// spi_device_queue_trans
// spi_device_get_trans_result