#include "logger.hpp"
#include <driver/gpio.h>
#include <freertos/mpu_wrappers.h>

#define BLINK_GPIO GPIO_NUM_2

using namespace std::chrono_literals;

static bool s_led_state = false;

static espp::Logger logger({.tag = "Blink", .level = espp::Logger::Verbosity::DEBUG});

static void blink_led(void) {
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

extern "C" void app_main(void) {
  logger.info("Bootup");

  gpio_reset_pin(BLINK_GPIO);
  /* Set the GPIO as a push/pull output */
  gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

  // also print in the main thread
   while (1) {
    logger.info("Turning the LED {}!", s_led_state == true ? "ON" : "OFF");
    logger.info("blink_state:{}", s_led_state);
    blink_led();
    /* Toggle the LED state */
    s_led_state = !s_led_state;
    std::this_thread::sleep_for(1s);
  }
}
