#include "logger.hpp"
#include <driver/gpio.h>
#include <freertos/mpu_wrappers.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <esp_err.h>
#include "interrupt.hpp"
#include "led.hpp"

#define ENC_INTERRUPT_PIN GPIO_NUM_14
#define MOTOR_GPIO 15

#define ADC_ATTEN ADC_ATTEN_DB_12

using namespace std::chrono_literals;

static espp::Logger logger({.tag = "ESP32", .level = espp::Logger::Verbosity::DEBUG});
static int adc_raw[2][10];

static auto last = std::chrono::high_resolution_clock::now();
static float rpm = 0.0;
static int counts = 0;

// Set motor output speed
// speed - Range of [0.0,1.0], 1.0 being full speed
void set_motor_speed(float speed) {
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, (1.0 - speed) * 4096));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5));
}

extern "C" void app_main(void) {

  // ADC setup
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


  // Encoder interrupt stuff

  auto callback = [&](const espp::Interrupt::Event &event) {
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last).count();
    last = now;
    counts++;
    float speed = 1000000.0/ ((float) elapsed);
    rpm = (speed/4.0)*60;
  };

  espp::Interrupt::PinConfig enc_inter_pinconfig = {
      .gpio_num = ENC_INTERRUPT_PIN,
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
        .interrupts = {enc_inter_pinconfig},
        .task_config =
            {
                .name = "Encoder Task",
                .stack_size_bytes = 6192,
                .priority = 5,
            },
        .log_level = espp::Logger::Verbosity::WARN,
    });

  // PWM output stuff
  // technically a hardware LED controller but can be used for pwm outputs
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .duty_resolution  = LEDC_TIMER_12_BIT,
    .timer_num        = LEDC_TIMER_2,
    .freq_hz          = 4000,  // Set output frequency at 4 kHz
    .clk_cfg          = LEDC_AUTO_CLK,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  ledc_channel_config_t ledc_channel = {
    .gpio_num       = MOTOR_GPIO,
    .speed_mode     = LEDC_LOW_SPEED_MODE,
    .channel        = LEDC_CHANNEL_5,
    .timer_sel      = LEDC_TIMER_2,
    .duty           = 0, // Set duty to 0%
    .hpoint         = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

  // Main loop

  while (1) {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &adc_raw[0][0]));
    // logger.info("ADC1_CH6: {}", adc_raw[0][0]);

    set_motor_speed(1.0);
    std::this_thread::sleep_for(1s);


    logger.info("RPM: {}", rpm);
    std::this_thread::sleep_for(10ms);
  }
}
