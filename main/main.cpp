#include "logger.hpp"
#include <driver/gpio.h>
#include <freertos/mpu_wrappers.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <esp_err.h>
#include "interrupt.hpp"

#define BLINK_GPIO GPIO_NUM_2

#define ADC_ATTEN ADC_ATTEN_DB_12

using namespace std::chrono_literals;

static espp::Logger logger({.tag = "ESP32", .level = espp::Logger::Verbosity::DEBUG});
static int adc_raw[2][10];

static auto last = std::chrono::high_resolution_clock::now();
static float rpm = 0.0;
static int counts = 0;

extern "C" void app_main(void) {
  adc_oneshot_unit_handle_t adc1_handle;
  adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

  adc_oneshot_chan_cfg_t config = {
    .atten = ADC_ATTEN,
    .bitwidth = ADC_BITWIDTH_DEFAULT,
  };

  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config));

  auto callback = [&](const espp::Interrupt::Event &event) {
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last).count();
    last = now;
    counts++;
    float speed = 1000000.0/ ((float) elapsed);
    rpm = (speed/3.0)*60;
  };

  espp::Interrupt::PinConfig inter15 = {
      .gpio_num = GPIO_NUM_15,
      .callback = callback,
      .active_level = espp::Interrupt::ActiveLevel::HIGH,
      .interrupt_type = espp::Interrupt::Type::RISING_EDGE,
      .pullup_enabled = false,
      .pulldown_enabled = true,
      // flexible filter requiring configuration (default is provided as 5us
      // threshold in 10us window), but other configurations can be manually
      // set as below
      .filter_type = espp::Interrupt::FilterType::NONE
  };


  espp::Interrupt enc_interrupt({
        .isr_core_id = 1,
        .interrupts = {inter15},
        .task_config =
            {
                .name = "Encoder Task",
                .stack_size_bytes = 6192,
                .priority = 5,
            },
        .log_level = espp::Logger::Verbosity::WARN,
    });

  while (1) {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &adc_raw[0][0]));
    // logger.info("ADC1_CH6: {}", adc_raw[0][0]);
    logger.info("RPM: {}", rpm);
    std::this_thread::sleep_for(10ms);
  }
}
