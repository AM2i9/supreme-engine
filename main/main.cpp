#include "logger.hpp"
#include <driver/gpio.h>
#include <freertos/mpu_wrappers.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <esp_err.h>
#include "interrupt.hpp"
#include "led.hpp"
#include <atomic>

#define BUTTON_PIN GPIO_NUM_18
#define ENC_INTERRUPT_PIN GPIO_NUM_14
#define ENC_CPR 8.0
#define MOTOR_GPIO 15
#define SMOOTH_SIZE 50

#define Y_MIN 1150
#define Y_MAX 2500
#define X_MIN 1150
#define X_MAX 2500

#define ADC_ATTEN ADC_ATTEN_DB_12

using namespace std::chrono_literals;

static espp::Logger logger({.tag = "ESP32", .level = espp::Logger::Verbosity::DEBUG});
static int adc_raw[2][10];

static auto last = std::chrono::high_resolution_clock::now();
static float speed{0.0f};
static float rpm{0.0f};

static double weights[SMOOTH_SIZE];
static std::vector<double> rpm_buffer;

template <typename T,typename M> T deadzone(T val, M deadzone) {
    if (val > deadzone || val < -deadzone) return val;
    return (T)0;
}

template <typename T> T map_range(T value, T a_start, T a_end, T b_start, T b_end) {
  return (value - a_start) * (b_end - b_start) / (a_end - a_start) + b_start;
}

// Set motor output speed
// speed - Range of [0.0,1.0], 1.0 being full speed
void set_motor_speed(float speed) {
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, (1.0 - speed) * 4096));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5));
}

// Math is scary
void compute_weights() {
  double sigma = 8;
  double sum = 0;

  for (int i=0;i<SMOOTH_SIZE;i++) {
    weights[i] = std::exp(-(i * i) / (2 * std::pow(sigma, 2)));
    sum += weights[i];
  }

  for (size_t i = 0; i < SMOOTH_SIZE; ++i) {
    weights[i] /= sum;
  }
}

// Function to take in new sample, smooth the data using previous samples
// then return the most recent datapoint but smoothed
// smooth
double smooth_single(double sample) {
  if (rpm_buffer.size() >= SMOOTH_SIZE) {
    // remove first element of array if window too large
    rpm_buffer.erase(rpm_buffer.begin());
  }
  rpm_buffer.push_back(sample); // add new sample to end of array
  
  double result = 0.0;
  double weightSum = 0.0;

  // Apply weights
  for (size_t i = 0; i < rpm_buffer.size(); i++) {
    size_t rev_i = rpm_buffer.size() - 1 - i;
    result += weights[i] * rpm_buffer[rev_i];
    weightSum += weights[i];
  }

  // logger.info("buf: {} | result: {} | weightSum: {}", rpm_buffer, result, weightSum);

  return (weightSum > 0.0) ? result / weightSum : sample;
}

// MAIN STUFF =-=-=-=-=--=-

extern "C" void app_main(void) {

  // The button
  ESP_ERROR_CHECK(gpio_input_enable(BUTTON_PIN));
  ESP_ERROR_CHECK(gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLDOWN_ONLY));

  // ADC configuration

  adc_oneshot_chan_cfg_t config = {
    .atten = ADC_ATTEN,
    .bitwidth = ADC_BITWIDTH_12,
  };

  adc_oneshot_unit_handle_t adc2_handle;
  adc_oneshot_unit_init_cfg_t init_config2 = {
    .unit_id = ADC_UNIT_2,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_8, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_4, &config));


  // Encoder interrupt stuff

  auto callback = [&](const espp::Interrupt::Event &event) {
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last).count();
    last = now;
    speed = 1000000.0f / ((float) elapsed);
  };

  espp::Interrupt::PinConfig enc_inter_pinconfig = {
      .gpio_num = ENC_INTERRUPT_PIN,
      .callback = callback,
      .active_level = espp::Interrupt::ActiveLevel::HIGH,
      .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
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

  // Compute weights for smoothing
  compute_weights();

  // PI controller parameters
  const std::chrono::milliseconds sample_period(50); // 100 ms loop
  float kp = 0.0003f; // proportional gain (tune as needed)
  float ki = 0.0009f; // integral gain (tune as needed)
  float integrator = 0.0f;
  const float integrator_limit = 1000.0f;
  float target_rpm = 500.0f; // desired RPM (adjust as needed)

  bool button_pressed = false;

  while (1) {

    // joystick stuff

    int adc_ch4;
    int adc_ch8;

    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC_CHANNEL_4, &adc_ch4));
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC_CHANNEL_8, &adc_ch8));

    double y_axis = map_range(adc_ch4, Y_MIN, Y_MAX, -4096, 4096)/4096.0;
    double x_axis = map_range(adc_ch8, X_MIN, X_MAX, -4096, 4096)/4096.0;

    y_axis = deadzone(y_axis, 0.10);
    y_axis = deadzone(y_axis, 0.10);

    // spinner stuff

    rpm = ((speed / ENC_CPR) * 60.0f);

    float measured_rpm = smooth_single(rpm);
    float error = target_rpm - measured_rpm;
    // integrate error (simple forward Euler)
    integrator += error * (sample_period.count() / 1000.0f);
    if (integrator > integrator_limit) integrator = integrator_limit;
    if (integrator < -integrator_limit) integrator = -integrator_limit;

    float output = (kp * error) + (ki * integrator); // controller output in RPM->duty space
    if (output > 1.0f) output = 1.0f;
    if (output < 0.0f) output = 0.0f;

    // overcome static torque
    // basically map the output of the controller to above the output range needed to get the motor moving
    output = (output * (1.0 - 0.18)) + 0.18;

    set_motor_speed(output);

    // Set so code only runs when button is first pressed and doesn't loop while pressed
    // can probably be replaced with an interupt if need bee
    if (gpio_get_level(BUTTON_PIN) && !button_pressed) {
      button_pressed = true;
      
      // setpoint steps
      if (target_rpm == 0.0) {
        target_rpm = 500.0;
      } else if (target_rpm == 500.0) {
        target_rpm = 750.0;
      }else if (target_rpm == 750.0) {
        target_rpm = 1400;
      } else if (target_rpm == 1400) {
        target_rpm = 0.0;
      }

    } else if (!gpio_get_level(BUTTON_PIN) && button_pressed){
      button_pressed = false;
    }

    logger.info("Setpoint: {}, RAW_RPM: {}, RPM: {}, Out: {}, output_rpm: {}, yaxis: {:.2f}, yaxisraw: {}", target_rpm, rpm, measured_rpm, output, output * 1600.0, y_axis, adc_ch4);

    std::this_thread::sleep_for(sample_period);
  }
}