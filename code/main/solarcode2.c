#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include <math.h>
#include "esp_adc_cal.h"
#define EX_UART_NUM UART_NUM_0
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/periph_ctrl.h"

////GPTIMER INIT
int seconds;
int timerOn;
typedef struct
{
  uint64_t event_count;
} example_queue_element_t;

static bool IRAM_ATTR example_timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
  BaseType_t high_task_awoken = pdFALSE;
  QueueHandle_t queue = (QueueHandle_t)user_data;
  // Retrieve count value and send to queue
  example_queue_element_t ele = {
      .event_count = edata->count_value};
  xQueueSendFromISR(queue, &ele, &high_task_awoken);
  // return whether we need to yield at the end of ISR
  return (high_task_awoken == pdTRUE);
}

// 14-Segment Display
#define SLAVE_ADDR 0x70              // alphanumeric address
#define OSC 0x21                     // oscillator cmd
#define HT16K33_BLINK_DISPLAYON 0x01 // Display on cmd
#define HT16K33_BLINK_OFF 0          // Blink off cmd
#define HT16K33_BLINK_CMD 0x80       // Blink cmd
#define HT16K33_CMD_BRIGHTNESS 0xE0  // Brightness cmd

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO 22        // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO 23        // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_0    // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE 0 // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE 0 // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ 100000   // i2c master clock freq
#define WRITE_BIT I2C_MASTER_WRITE          // i2c master write
#define READ_BIT I2C_MASTER_READ            // i2c master read
#define ACK_CHECK_EN true                   // i2c master will check ack
#define ACK_CHECK_DIS false                 // i2c master will not check ack
#define ACK_VAL 0x00                        // i2c ack value
#define NACK_VAL 0xFF                       // i2c nack value

#define DEFAULT_VREF 1100 // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 64  // Multisampling

// Please consult the datasheet of your servo before changing the following parameters
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500 // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE -90         // Minimum angle
#define SERVO_MAX_DEGREE 90          // Maximum angle

#define SERVO_PULSE_GPIO1 26 // GPIO connects to the PWM signal
#define SERVO_PULSE_GPIO2 25
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD 20000          // 20000 ticks, 20ms

static const char *TAG = "example";
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6; // GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

uint32_t get_voltage()
{
  uint32_t adc_reading = 0;
  // Multisampling
  for (int i = 0; i < NO_OF_SAMPLES; i++)
  {
    if (unit == ADC_UNIT_1)
    {
      adc_reading += adc1_get_raw((adc1_channel_t)channel);
    }
    else
    {
      int raw;
      adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
      adc_reading += raw;
    }
  }
  adc_reading /= NO_OF_SAMPLES;
  // Convert adc_reading to voltage in mV
  uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
  // vTaskDelay(pdMS_TO_TICKS(1000));
  return voltage;
}

static void check_efuse(void)
{
  // Check TP is burned into eFuse
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
  {
    printf("eFuse Two Point: Supported\n");
  }
  else
  {
    printf("eFuse Two Point: NOT supported\n");
  }

  // Check Vref is burned into eFuse
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
  {
    printf("eFuse Vref: Supported\n");
  }
  else
  {
    printf("eFuse Vref: NOT supported\n");
  }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
  {
    printf("Characterized using Two Point Value\n");
  }
  else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
  {
    printf("Characterized using eFuse Vref\n");
  }
  else
  {
    printf("Characterized using Default Vref\n");
  }
}

static const uint16_t alphafonttable[] = {
    0b0000000000000000, //
    0b0000000000000110, // !
    0b0000001000100000, // "
    0b0001001011001110, // #
    0b0001001011101101, // $
    0b0000110000100100, // %
    0b0010001101011101, // &
    0b0000010000000000, // '
    0b0010010000000000, // (
    0b0000100100000000, // )
    0b0011111111000000, // *
    0b0001001011000000, // +
    0b0000100000000000, // ,
    0b0000000011000000, // -
    0b0100000000000000, // .
    0b0000110000000000, // /
    0b0000110000111111, // 0
    0b0000000000000110, // 1
    0b0000000011011011, // 2
    0b0000000010001111, // 3
    0b0000000011100110, // 4
    0b0010000001101001, // 5
    0b0000000011111101, // 6
    0b0000000000000111, // 7
    0b0000000011111111, // 8
    0b0000000011101111, // 9
    0b0001001000000000, // :
    0b0000101000000000, // ;
    0b0010010000000000, // <
    0b0000000011001000, // =
    0b0000100100000000, // >
    0b0001000010000011, // ?
    0b0000001010111011, // @
    0b0000000011110111, // A
    0b0001001010001111, // B
    0b0000000000111001, // C
    0b0001001000001111, // D
    0b0000000011111001, // E
    0b0000000001110001, // F
    0b0000000010111101, // G
    0b0000000011110110, // H
    0b0001001000001001, // I
    0b0000000000011110, // J
    0b0010010001110000, // K
    0b0000000000111000, // L
    0b0000010100110110, // M
    0b0010000100110110, // N
    0b0000000000111111, // O
    0b0000000011110011, // P
    0b0010000000111111, // Q
    0b0010000011110011, // R
    0b0000000011101101, // S
    0b0001001000000001, // T
    0b0000000000111110, // U
    0b0000110000110000, // V
    0b0010100000110110, // W
    0b0010110100000000, // X
    0b0001010100000000, // Y
    0b0000110000001001, // Z
    0b0000000000111001, // [
    0b0010000100000000, //
    0b0000000000001111, // ]
    0b0000110000000011, // ^
    0b0000000000001000, // _
    0b0000000100000000, // `
    0b0001000001011000, // a
    0b0010000001111000, // b
    0b0000000011011000, // c
    0b0000100010001110, // d
    0b0000100001011000, // e
    0b0000000001110001, // f
    0b0000010010001110, // g
    0b0001000001110000, // h
    0b0001000000000000, // i
    0b0000000000001110, // j
    0b0011011000000000, // k
    0b0000000000110000, // l
    0b0001000011010100, // m
    0b0001000001010000, // n
    0b0000000011011100, // o
    0b0000000101110000, // p
    0b0000010010000110, // q
    0b0000000001010000, // r
    0b0010000010001000, // s
    0b0000000001111000, // t
    0b0000000000011100, // u
    0b0010000000000100, // v
    0b0010100000010100, // w
    0b0010100011000000, // x
    0b0010000000001100, // y
    0b0000100001001000, // z
    0b0000100101001001, // {
    0b0001001000000000, // |
    0b0010010010001001, // }
    0b0000010100100000, // ~
    0b0011111111111111,

};

// Function to initiate i2c -- note the MSB declaration!
static void i2c_example_master_init()
{
  // Debug
  printf("\n>> i2c Config\n");
  int err;

  // Port configuration
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

  /// Define I2C configurations
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;                        // Master mode
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;        // Default SDA pin
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;        // Default SCL pin
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ; // CLK frequency
  conf.clk_flags = 0;                                 // <-- UNCOMMENT IF YOU GET ERRORS (see readme.md)
  err = i2c_param_config(i2c_master_port, &conf);     // Configure
  if (err == ESP_OK)
  {
    printf("- parameters: ok\n");
  }

  // Install I2C driver
  err = i2c_driver_install(i2c_master_port, conf.mode,
                           I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                           I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
  // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
  if (err == ESP_OK)
  {
    printf("- initialized: yes\n\n");
  }

  // Dat in MSB mode
  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility  Functions //////////////////////////////////////////////////////////

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}

// Utility function to scan for i2c device
static void i2c_scanner()
{
  int32_t scanTimeout = 1000;
  printf("\n>> I2C scanning ..."
         "\n");
  uint8_t count = 0;
  for (uint8_t i = 1; i < 127; i++)
  {
    // printf("0x%X%s",i,"\n");
    if (testConnection(i, scanTimeout) == ESP_OK)
    {
      printf("- Device found at address: 0x%X%s", i, "\n");
      count++;
    }
  }
  if (count == 0)
    printf("- No I2C devices found!"
           "\n");
  printf("\n");
}

////////////////////////////////////////////////////////////////////////////////

// Alphanumeric Functions //////////////////////////////////////////////////////

// Turn on oscillator for alpha display
int alpha_oscillator()
{
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  return ret;
}

// Set blink rate to off
int no_blink()
{
  int ret;
  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
  i2c_master_stop(cmd2);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd2);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val)
{
  int ret;
  i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
  i2c_master_start(cmd3);
  i2c_master_write_byte(cmd3, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
  i2c_master_stop(cmd3);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  return ret;
}

////////////////////////////////////////////////////////////////////////////////

static void test_alpha_display()
{
  // Debug
  int ret;
  printf(">> Test Alphanumeric Display: \n");

  // Set up routines
  // Turn on alpha oscillator
  ret = alpha_oscillator();
  if (ret == ESP_OK)
  {
    printf("- oscillator: ok \n");
  }
  // Set display blink off
  ret = no_blink();
  if (ret == ESP_OK)
  {
    printf("- blink: off \n");
  }
  ret = set_brightness_max(0xF);
  if (ret == ESP_OK)
  {
    printf("- brightness: max \n");
  }

  // Write to characters to buffer

  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));

  esp_vfs_dev_uart_use_driver(UART_NUM_0);
}

static inline uint32_t example_angle_to_compare(int angle)
{
  return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

void app_main(void)
{

  // set timer
  example_queue_element_t ele;
  QueueHandle_t queue = xQueueCreate(10, sizeof(example_queue_element_t));
  if (!queue)
  {
    ESP_LOGE("Timer", "Creating queue failed");
    return;
  }

  ESP_LOGI("Timer", "Create timer handle");
  gptimer_handle_t gptimer = NULL;
  gptimer_config_t hardware_timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000, // 1MHz, 1 tick=1us
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&hardware_timer_config, &gptimer));

  gptimer_event_callbacks_t cbs = {
      .on_alarm = example_timer_on_alarm_cb,
  };
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, queue));

  ESP_LOGI("Timer", "Enable timer");
  ESP_ERROR_CHECK(gptimer_enable(gptimer));

  ESP_LOGI("Timer", "Start timer");
  gptimer_alarm_config_t alarm_config2 = {
      .reload_count = 0,
      .alarm_count = 1000000, // period = 1s
      .flags.auto_reload_on_alarm = true,
  };
  ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config2));

  ESP_ERROR_CHECK(gptimer_start(gptimer));

  ////////////////////////////////////////////////// SERVO TIMERS/////////////////////////////////////

  ESP_LOGI(TAG, "Create timer and operator");
  mcpwm_timer_handle_t timer = NULL;
  mcpwm_timer_handle_t timer2 = NULL;
  mcpwm_timer_config_t timer_config = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
      .period_ticks = SERVO_TIMEBASE_PERIOD,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
  };
  mcpwm_timer_config_t timer_config2 = {
      .group_id = 1,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
      .period_ticks = SERVO_TIMEBASE_PERIOD,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
  };
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config2, &timer2));

  mcpwm_oper_handle_t oper = NULL;
  mcpwm_operator_config_t operator_config = {
      .group_id = 0, // operator must be in the same group to the timer
  };
  mcpwm_oper_handle_t oper2 = NULL;
  mcpwm_operator_config_t operator_config2 = {
      .group_id = 1, // operator must be in the same group to the timer
  };
  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config2, &oper2));

  ESP_LOGI(TAG, "Connect timer and operator");
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper2, timer2));

  ESP_LOGI(TAG, "Create comparator and generator from the operator");
  mcpwm_cmpr_handle_t comparator = NULL;
  mcpwm_comparator_config_t comparator_config = {
      .flags.update_cmp_on_tez = true,
  };
  ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

  mcpwm_cmpr_handle_t comparator2 = NULL;
  mcpwm_comparator_config_t comparator_config2 = {
      .flags.update_cmp_on_tez = true,
  };
  ESP_ERROR_CHECK(mcpwm_new_comparator(oper2, &comparator_config2, &comparator2));

  mcpwm_gen_handle_t generator = NULL;
  mcpwm_generator_config_t generator_config = {
      .gen_gpio_num = SERVO_PULSE_GPIO1,
  };
  ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

  mcpwm_gen_handle_t generator2 = NULL;
  mcpwm_generator_config_t generator_config2 = {
      .gen_gpio_num = SERVO_PULSE_GPIO2,
  };
  ESP_ERROR_CHECK(mcpwm_new_generator(oper2, &generator_config2, &generator2));

  // set the initial compare value, so that the servo will spin to the center position
  // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));

  ESP_LOGI(TAG, "Set generator action on timer and compare event");
  // go high on counter empty
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator2,
                                                            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
  // go low on compare threshold
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                              MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator2,
                                                              MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator2, MCPWM_GEN_ACTION_LOW)));

  ESP_LOGI(TAG, "Enable and start timer");
  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

  ESP_ERROR_CHECK(mcpwm_timer_enable(timer2));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer2, MCPWM_TIMER_START_NO_STOP));

  ////////////////////////////////////////////////////////////////////////scan servo steps/////////////

  int angle = 0;
  int step = 2;
  int angle_1 = 0;
  int step_1 = 2;

  // Check if Two Point or Vref are burned into eFuse
  check_efuse();
  i2c_example_master_init();
  i2c_scanner();

  // Configure ADC
  if (unit == ADC_UNIT_1)
  {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, atten);
  }
  else
  {
    adc2_config_channel_atten((adc2_channel_t)channel, atten);
  }

  // Characterize ADC
  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  print_char_val_type(val_type);

  int current_position_horizontal = 0;
  int max_position_horizontal = 0;
  int current_position_vertical = 0;
  int max_position_vertical = 0;
  int index = 0;
  int counter = 0;
  int current_voltage_vertical = 0;
  int previous_voltage = 0;
  int max_voltage_horizontal = 0;
  int max_voltage_vertical = 0;
  int current_voltage_horizontal = 0;
  int count_1 = 0;
  int tangle = -90;
  int tangle_1 = -90;

  vTaskDelay(pdMS_TO_TICKS(2000));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(0)));
  vTaskDelay(pdMS_TO_TICKS(2000));

  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(90)));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(90)));

  // intitial scan:

  for (int i = 0; i < 45; i++)
  {
    tangle_1 = tangle_1 + 4;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(tangle_1)));
    vTaskDelay(30);
    current_voltage_horizontal = get_voltage();
    if (current_voltage_horizontal >= max_voltage_horizontal)
    {
      max_voltage_horizontal = current_voltage_horizontal;
      max_position_horizontal = tangle_1;
    }
  }
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(max_position_horizontal)));

  for (int j = 0; j < 45; j++)
  {
    tangle = tangle + 4;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(tangle)));
    vTaskDelay(30);
    current_voltage_vertical = get_voltage();
    if (current_voltage_vertical >= max_voltage_vertical)
    {
      max_voltage_vertical = current_voltage_vertical;
      max_position_vertical = tangle;
    }
  }

  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(max_position_vertical)));
  vTaskDelay(10);

  vTaskDelay(30);

  angle = max_position_vertical;
  angle_1 = max_position_horizontal;

  while (1)
  {

    if (angle == 90)
    {
      counter = 11;
    }
    if (angle == -90)
    {
      counter = 0;
    }
    if (angle_1 == 90)
    {
      count_1 = 11;
    }
    if (angle_1 == -90)
    {
      count_1 = 0;
    }

    ESP_LOGI(TAG, "Angle of rotation: %d", angle);
    ESP_LOGI(TAG, "Angle of rotation of Second Servos: %d", angle_1);

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle)));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(angle_1)));
    // Add delay, since it takes time for servo to rotate, usually 200ms/60degree rotation under 5V power supply
    //  vTaskDelay(pdMS_TO_TICKS(500));

    current_position_horizontal = angle;
    current_position_vertical = angle_1;

    if (index == 0)
    {
      if (counter < 10)
      {
        angle = angle + 2;
      }
      counter = counter + 1;
      current_voltage_vertical = get_voltage();
      if (current_voltage_vertical >= max_voltage_vertical)
      {
        max_voltage_vertical = current_voltage_vertical;
        max_position_vertical = angle;
      }
      if (counter == 10)
      {
        angle = angle - 20;
        counter = 11;
      }
      if (counter > 10)
      {
        angle = angle - 2;
        printf("the value of angle is %d\n", angle);
        current_voltage_vertical = get_voltage();
        if (current_voltage_vertical >= max_voltage_vertical)
        {
          max_voltage_vertical = current_voltage_vertical;
          max_position_vertical = angle;
        }
      }
      if (counter == 20)
      {
        index = 1;
        counter = 0;
      }
    }

    if (index == 1)
    {
      ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(max_position_vertical)));
      if (count_1 < 10)
      {
        angle_1 = angle_1 + 2;
      }
      count_1 = count_1 + 1;
      current_voltage_horizontal = get_voltage();
      if (current_voltage_horizontal >= max_voltage_horizontal)
      {
        max_voltage_horizontal = current_voltage_horizontal;
        max_position_horizontal = angle_1;
      }
      if (count_1 == 10)
      {
        angle_1 = angle_1 - 20;
        count_1 = 11;
      }
      if (count_1 > 10)
      {
        angle_1 = angle_1 - 2;
        current_voltage_horizontal = get_voltage();
        if (current_voltage_horizontal >= max_voltage_horizontal)
        {
          max_voltage_horizontal = current_voltage_horizontal;
          max_position_horizontal = angle_1;
        }
      }
      if (count_1 == 20)
      {
        index = 2;
        count_1 = 0;
      }

      int final_azimuth_and_altitude = 1;
    }

    if (index == 2)
    {
      printf("In state2 shifting to max position horizontal %d and max postion vertical %d\n", max_position_horizontal, max_position_vertical);

      printf("the max volts is %d\n", max_voltage_horizontal);
      printf("the max volts vertical is %d\n", max_voltage_vertical);

      ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(max_position_vertical)));
      ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(max_position_horizontal)));

      angle = max_position_vertical;
      angle_1 = max_position_horizontal;

      max_voltage_horizontal = 0;
      max_voltage_vertical = 0;

      //vTaskDelay(pdMS_TO_TICKS(5000));
      printf("PRE TIMER\n");
      seconds = -1;
      timerOn = 1;
      while (timerOn)
      {

        if (seconds <=5)
        { //(button_state != 1){
          if (xQueueReceive(queue, &ele, pdMS_TO_TICKS(2000)))
          {
            printf("TIMER ENABLED\n");
            ESP_LOGI(TAG, "Timer reloaded, count=%d", seconds);
            seconds++;
        } else {
            ESP_LOGW(TAG, "Missed one count event");
        }
      } else { 
            ESP_ERROR_CHECK(gptimer_stop(gptimer));
            seconds = -1;
            timerOn = 0;
            printf("TIEMR @ 5s\n");
            ESP_ERROR_CHECK(gptimer_start(gptimer));
        }
      }
      printf("Index = 0\n");
      index = 0;
    }

    uint32_t adc_reading = 0;
    // Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
      if (unit == ADC_UNIT_1)
      {
        adc_reading += adc1_get_raw((adc1_channel_t)channel);
      }
      else
      {
        int raw;
        adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
        adc_reading += raw;
      }
    }
    adc_reading /= NO_OF_SAMPLES;
    // Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    // printf("Raw: %lu\tVoltage: %lumV\n", adc_reading, voltage);
    vTaskDelay(pdMS_TO_TICKS(800));

    uint16_t displaybuffer[8];

    ESP_LOGI(TAG, "ALTIDUDE: %d", abs(angle_1));
    ESP_LOGI(TAG, "HORIZONTAL ANGLE: %d", abs(angle));

    if (max_position_vertical < 10)
    {
      displaybuffer[0] = 0b0000000000000000;
      displaybuffer[1] = alphafonttable[abs(angle_1) + 16];
    }

    if (max_position_vertical < 10)
    {
      displaybuffer[2] = 0b0000000000000000;
      displaybuffer[3] = alphafonttable[abs(angle) + 16];
    }

    if (max_position_horizontal >= 10)
    {
      int angle_first_1 = abs(angle_1) % 10;
      int angle_second_1 = floor(abs(angle_1) / 10);
      displaybuffer[0] = alphafonttable[abs(angle_second_1) + 16];
      displaybuffer[1] = alphafonttable[abs(angle_first_1) + 16];
    }

    if (max_position_horizontal >= 10)
    {
      int angle_first = abs(angle) % 10;
      int angle_second = floor(abs(angle) / 10);
      displaybuffer[2] = alphafonttable[abs(angle_second) + 16];
      displaybuffer[3] = alphafonttable[abs(angle_first) + 16];
    }

    // Send commands characters to display over I2C
    i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
    i2c_master_start(cmd4);
    i2c_master_write_byte(cmd4, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
    for (uint8_t i = 0; i < 8; i++)
    {
      i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
      i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
    }
    i2c_master_stop(cmd4);
    int ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd4);

    // for (int i = 0; i < 8; i++) {
    //     printf("%04x\n", displaybuffer[i]);
    // }

    if (ret == ESP_OK)
    {
      //  vTaskDelay(1000/portTICK_PERIOD_MS);

      printf("Wrote to Display\n");
    }
  }
}