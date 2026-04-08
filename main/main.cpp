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
#include "task.hpp"
#include "wifi_ap.hpp"
#include <esp_http_server.h>
#include "memory.h"

#if CONFIG_ESP32_WIFI_NVS_ENABLED
#include "nvs_flash.h"
#endif

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

// GOAL LIST!!!!!!!!
// Welcome to the great bodge because we too lazy to figure out USB HID stuff
// -> WiFi AP
//     -> Wireless AP, connectable
// -> Websocket server
//     -> Connect from game
//     -> accept requests from game, taking in:
//          -> New setpoints
//     -> return system data:
//          -> Setpoint
//          -> Live RPM
//          -> How many times button has been pressed since last request
//          -> Joystick values

using namespace std::chrono_literals;

// PI controller parameters
const std::chrono::milliseconds sample_period(50); // 100 ms loop
float kp = 0.0003f; // proportional gain (tune as needed)
float ki = 0.0009f; // integral gain (tune as needed)
float integrator = 0.0f;
const float integrator_limit = 1000.0f;
uint16_t target_rpm = 0.0f; // desired RPM (adjust as needed)

static espp::Logger logger({.tag = "ESP32", .level = espp::Logger::Verbosity::DEBUG});

static auto last = std::chrono::high_resolution_clock::now();
static float speed{0.0f};
static float rpm{0.0f};

static double weights[SMOOTH_SIZE];
static std::vector<double> rpm_buffer;

int adc_ch4;
int adc_ch8;

uint16_t y_axis;
uint16_t x_axis;
std::atomic<uint8_t> button_presses = 0;

httpd_handle_t server;
httpd_config_t config;

template <typename T,typename M> T deadzone(T val, M deadzone, M zero) {
    if (val - zero > deadzone || val - zero < -deadzone) return val;
    return (T)zero;
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

struct packet_data {
  float target_rpm;
  double x_axis;
  double y_axis;
};

static esp_err_t ws_handler(httpd_req_t *req) {

  // logger.info("recc'd request or something");
  httpd_ws_frame_t ws_pkt;
  uint8_t *buf = NULL;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
  ws_pkt.type = HTTPD_WS_TYPE_TEXT;

  // this has to be here otherwise it doesn't work
  if (req->method == HTTP_GET) {
    logger.info("Handshake!!!!");
    return ESP_OK;
  }

  /* Set max_len = 0 to get the frame len */
  esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
  if (ret != ESP_OK) {
      logger.error("httpd_ws_recv_frame failed to get frame len with {}", ret);
      return ret;
  }

  // logger.info("Frame type: {}", (int) ws_pkt.type);

  // logger.info("pkt length: {}", ws_pkt.len);
  if (ws_pkt.len) {
      buf = (uint8_t *)calloc(1, ws_pkt.len+1);
      ws_pkt.payload = buf;
      /* Set max_len = ws_pkt.len to get the frame payload */
      ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
      if (ret != ESP_OK) {
          logger.error("httpd_ws_recv_frame failed with {}", ret);
          free(buf);
          return ret;
      }
      // logger.info("Got packet with message: {}", *ws_pkt.payload);
  }
  //! STuff needs to happen here or something
  // future self, you may do that for me if you would :)
  // did it :)
  // logger.info("Packet type: {}", ws_pkt.type);
  if (ws_pkt.type == HTTPD_WS_TYPE_BINARY &&
    ws_pkt.payload != NULL) {
    // logger.info("got packet: {}", ws_pkt.payload[1]);
    if (ws_pkt.payload[0] == 0) {
      // logger.info("requested data!!!");
      uint8_t return_payload[5];
      return_payload[0] = (uint8_t) x_axis & 0x00FF;
      return_payload[1] = (uint8_t) ((x_axis & 0xFF00) >> 8);
      return_payload[2] = (uint8_t) y_axis & 0x00FF;
      return_payload[3] = (uint8_t) ((y_axis & 0xFF00) >> 8);
      return_payload[4] = button_presses.load();
      ws_pkt.payload = return_payload;
      ws_pkt.len = sizeof(return_payload);

      // there is 100% a race condition here
      // but it makes the button randomly not work
      // and that is funny
      // so it's staying
      // if (button_presses.load() > 0) {
      //   button_presses.store(0);
      // }
    } else if (ws_pkt.payload[0] == 1) {
      // we are getting a new set speed
      target_rpm = ws_pkt.payload[1] + (ws_pkt.payload[2] << 8);
    }
  }

  ret = httpd_ws_send_frame(req, &ws_pkt);
  if (ret != ESP_OK) {
      logger.error("httpd_ws_send_frame failed with %d", ret);
  }
  free(buf);
  return ret;
}

static const httpd_uri_t ws = {
        .uri        = "/ws",
        .method     = HTTP_GET,
        .handler    = ws_handler,
        .user_ctx   = NULL,
        .is_websocket = true
};

// Webserver stuff
void start_webserver(void) {
  config = HTTPD_DEFAULT_CONFIG();

  // Start the httpd server
  logger.info("Starting server on port: '{}'", config.server_port);
  if (httpd_start(&server, &config) == ESP_OK) {
      // Registering the ws handler
      logger.info("Registering URI handlers");
      ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_register_uri_handler(server, &ws));
      logger.info("Done.");
      return;
  }

  logger.error("Error starting server!");
}

// MAIN STUFF =-=-=-=-=--=-

extern "C" void app_main(void) {

  ESP_ERROR_CHECK(esp_event_loop_create_default());

  #if CONFIG_ESP32_WIFI_NVS_ENABLED
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
  #endif

  // Wifi for game
  espp::WifiAp::Config ap_config{.ssid = "youjustlost",
                                .password = "thegame",
                                .log_level = espp::Logger::Verbosity::DEBUG};
  espp::WifiAp wifi_ap(ap_config);


  // websocket for game
  start_webserver();

  // ADC configuration

  adc_oneshot_chan_cfg_t adc_config = {
    .atten = ADC_ATTEN,
    .bitwidth = ADC_BITWIDTH_12,
  };

  adc_oneshot_unit_handle_t adc1_handle;
  adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &adc_config)); // VP pin
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &adc_config)); // VN pin

  ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_ch4));

  // The button
  ESP_ERROR_CHECK(gpio_input_enable(BUTTON_PIN));
  ESP_ERROR_CHECK(gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLDOWN_ONLY));

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

  bool button_pressed = false;

  // Create task for running the PI loop
  // not entirely thread safe but honestly should be fiiiiiinne
  espp::Task::configure_task_watchdog(1000ms, true);
  auto pi_periodic = [](std::mutex &m, std::condition_variable &cv) {
    rpm = ((speed / ENC_CPR) * 60.0f);

    float measured_rpm = smooth_single(rpm);
    float error = target_rpm - measured_rpm;
    // integrate error (simple forward Euler)
    integrator += error * (sample_period.count() / 1000.0f);
    if (integrator > integrator_limit) integrator = integrator_limit;
    if (integrator < 0) integrator = 0;

    float output = (kp * error) + (ki * integrator); // controller output in RPM->duty space
    if (output > 1.0f) output = 1.0f;
    if (output < 0.0f) output = 0.0f;

    // overcome static torque
    // basically map the output of the controller to above the output range needed to get the motor moving
    output = (output * (1.0 - 0.18)) + 0.18;

    set_motor_speed(output);

    // logger.info("Setpoint: {}, raw_rpm: {:.2f}, rpm: {}", target_rpm, rpm, measured_rpm);

    // Return false so task keeps running
    //threadsafe wait
    std::unique_lock<std::mutex> lk(m);
    cv.wait_for(lk, sample_period);

    return false;
  };

  auto pi_task = espp::Task({.callback = pi_periodic,
                          .task_config = {.name = "PILoop"},
                          .log_level = espp::Logger::Verbosity::DEBUG});
  pi_task.start();
  pi_task.start_watchdog();


  while (1) {

    // joystick stuff

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_ch4));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &adc_ch8));

    y_axis = map_range(adc_ch4, Y_MIN, Y_MAX, 0, 65535);
    x_axis = map_range(adc_ch8, X_MIN, X_MAX, 0, 65535);

    // really quick and lazy 5% deadzone
    if (y_axis > (32767 + 3276) || y_axis < (32767 - 3276)) {
      y_axis = 32767;
    }

    if (x_axis > (32767 + 3276) || x_axis < (32767 - 3276)) {
      x_axis = 32767;
    }

    // Set so code only runs when button is first pressed and doesn't loop while pressed
    // can probably be replaced with an interupt if need bee
    if (gpio_get_level(BUTTON_PIN) && !button_pressed) {
      button_pressed = true;
      button_presses.fetch_add(1);
      logger.info("button");

    } else if (!gpio_get_level(BUTTON_PIN) && button_pressed){
      button_pressed = false;
    }

    // logger.info("target_rpm: {}", target_rpm);
    // logger.info("yaxis: {}, xaxis: {}", y_axis, x_axis);
    // logger.info("{}", integrator);

    std::this_thread::sleep_for(10ms);
  }
}