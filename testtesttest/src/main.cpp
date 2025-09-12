#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_cpu.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "mqtt_client.h"
#include "esp_crt_bundle.h"
#include "config.h"
#include "driver/i2c.h"

// ROM delay function (provided in ROM)
extern "C" void ets_delay_us(uint32_t us);

static const char *TAG = "led_mqtt";
static const char *TAG_IR = "ir_system";

// PL9823-F8 RGB LED Configuration
typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} rgb_color_t;

// PL9823-F8 LED strip encoder
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} rmt_led_strip_encoder_t;

// PL9823-F8 timing definitions (800kHz protocol)
#define PL9823_BIT0_HIGH_NS    300      // 0.3us high for bit 0
#define PL9823_BIT0_LOW_NS     900      // 0.9us low for bit 0  
#define PL9823_BIT1_HIGH_NS    900      // 0.9us high for bit 1
#define PL9823_BIT1_LOW_NS     300      // 0.3us low for bit 1
#define PL9823_RESET_US        50       // 50us reset period

// RMT clock resolution
#define RMT_RESOLUTION_HZ      1000000  // 1MHz resolution

// Convert nanoseconds to RMT ticks
#define NS_TO_TICKS(ns) ((ns) * RMT_RESOLUTION_HZ / 1000000000)
#define US_TO_TICKS(us) ((us) * RMT_RESOLUTION_HZ / 1000000)

// MQTT
static esp_mqtt_client_handle_t client = NULL;
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// RMT and LED configuration
static const gpio_num_t LED_PIN = GPIO_NUM_15;
static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;
static TaskHandle_t led_task_handle = NULL;
static bool led_is_on = false;

// NJL7302L-F3 Light Sensor Configuration (GPIO3に変更)
static const gpio_num_t LIGHT_SENSOR_PIN = GPIO_NUM_3;  // GPIO3 for light sensor (moved from GPIO4)
static const adc_channel_t LIGHT_SENSOR_CHANNEL = ADC_CHANNEL_3;  // GPIO3 = ADC1_CH3
static const adc_unit_t LIGHT_SENSOR_UNIT = ADC_UNIT_1;
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_chan3_handle = NULL;
static bool adc_calibration_enable = false;

// Light sensor thresholds and variables
static int light_raw_value = 0;
static int light_voltage_mv = 0;
static TaskHandle_t light_sensor_task_handle = NULL;

// Buzzer task handle
static TaskHandle_t buzzer_task_handle = NULL;

// IR LED Configuration for GPIO7 (Safe testing)
static const gpio_num_t IR_LED_PIN = GPIO_NUM_7;
#define IR_LED_PWM_TIMER    LEDC_TIMER_1
#define IR_LED_PWM_MODE     LEDC_LOW_SPEED_MODE

// Buzzer Configuration for GPIO1
static const gpio_num_t BUZZER_PIN = GPIO_NUM_1;
#define BUZZER_PWM_TIMER    LEDC_TIMER_2
#define BUZZER_PWM_MODE     LEDC_LOW_SPEED_MODE
#define BUZZER_PWM_CHANNEL  LEDC_CHANNEL_2
#define BUZZER_FREQUENCY    4000  // 4kHz for clearer buzzer tone (more audible)
#define BUZZER_RESOLUTION   LEDC_TIMER_8_BIT

// OLED Display Configuration for 0.91" SSD1306
#define OLED_WIDTH 128
#define OLED_HEIGHT 32
#define OLED_ADDRESS 0x3C // I2C address for 0.91" OLED
#define I2C_SDA_PIN GPIO_NUM_23  // GPIO23 = SDA
#define I2C_SCL_PIN GPIO_NUM_22  // GPIO22 = SCL
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000 // I2C frequency 400kHz

// SSD1306 Commands
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETMULTIPLEX 0xA8
#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETSTARTLINE 0x40
#define SSD1306_CHARGEPUMP 0x8D
#define SSD1306_MEMORYMODE 0x20
#define SSD1306_SEGREMAP 0xA0
#define SSD1306_COMSCANDEC 0xC8
#define SSD1306_SETCOMPINS 0xDA
#define SSD1306_SETCONTRAST 0x81
#define SSD1306_SETPRECHARGE 0xD9
#define SSD1306_SETVCOMDETECT 0xDB
#define SSD1306_NORMALDISPLAY 0xA6

// OLED task handle
static TaskHandle_t oled_display_task_handle = NULL;
// GP1UXC41QS IR Receiver Configuration
static const gpio_num_t IR_RECEIVER_PIN = GPIO_NUM_8;  // GP1UXC41QS connected to GPIO8
static const gpio_num_t IR_TRANSMITTER_PIN = GPIO_NUM_7;  // IR LED connected to GPIO7
static QueueHandle_t ir_receive_queue = NULL;

// Simple IR testing - no protocol parsing needed for basic functionality test

// LED encoder functions for PL9823-F8
static size_t rmt_encode_led_strip(rmt_encoder_t *encoder, rmt_channel_handle_t channel, 
                                   const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state) {
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = led_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    switch (led_encoder->state) {
    case 0: // send RGB data
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = 1; // switch to next state when current encoder finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state = (rmt_encode_state_t)(state | RMT_ENCODING_MEM_FULL);
            goto out;
        }
    // fall-through
    case 1: // send reset code
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset_code,
                                                sizeof(rmt_symbol_word_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = RMT_ENCODING_RESET; // back to the initial encoding session
            state = (rmt_encode_state_t)(state | RMT_ENCODING_COMPLETE);
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state = (rmt_encode_state_t)(state | RMT_ENCODING_MEM_FULL);
            goto out;
        }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder) {
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_del_encoder(led_encoder->bytes_encoder);
    rmt_del_encoder(led_encoder->copy_encoder);
    free(led_encoder);
    return ESP_OK;
}

static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder) {
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_reset(led_encoder->bytes_encoder);
    rmt_encoder_reset(led_encoder->copy_encoder);
    led_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

// Create LED strip encoder for PL9823-F8
static esp_err_t rmt_new_led_strip_encoder(rmt_encoder_handle_t *ret_encoder) {
    esp_err_t ret = ESP_OK;
    rmt_led_strip_encoder_t *led_encoder = NULL;
    
    led_encoder = (rmt_led_strip_encoder_t*)calloc(1, sizeof(rmt_led_strip_encoder_t));
    if (!led_encoder) {
        ret = ESP_ERR_NO_MEM;
        return ret;
    }
    
    led_encoder->base.encode = rmt_encode_led_strip;
    led_encoder->base.del = rmt_del_led_strip_encoder;
    led_encoder->base.reset = rmt_led_strip_encoder_reset;
    
    // Create bytes encoder for RGB data
    rmt_bytes_encoder_config_t bytes_encoder_config = {};
    bytes_encoder_config.bit0.level0 = 1;
    bytes_encoder_config.bit0.duration0 = NS_TO_TICKS(PL9823_BIT0_HIGH_NS);
    bytes_encoder_config.bit0.level1 = 0;
    bytes_encoder_config.bit0.duration1 = NS_TO_TICKS(PL9823_BIT0_LOW_NS);
    bytes_encoder_config.bit1.level0 = 1;
    bytes_encoder_config.bit1.duration0 = NS_TO_TICKS(PL9823_BIT1_HIGH_NS);
    bytes_encoder_config.bit1.level1 = 0;
    bytes_encoder_config.bit1.duration1 = NS_TO_TICKS(PL9823_BIT1_LOW_NS);
    bytes_encoder_config.flags.msb_first = 1; // PL9823-F8 expects MSB first
    
    ret = rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder->bytes_encoder);
    if (ret != ESP_OK) {
        free(led_encoder);
        return ret;
    }
    
    // Create copy encoder for reset code
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ret = rmt_new_copy_encoder(&copy_encoder_config, &led_encoder->copy_encoder);
    if (ret != ESP_OK) {
        rmt_del_encoder(led_encoder->bytes_encoder);
        free(led_encoder);
        return ret;
    }
    
    // Construct the reset code
    led_encoder->reset_code.level0 = 0;
    led_encoder->reset_code.duration0 = US_TO_TICKS(PL9823_RESET_US);
    led_encoder->reset_code.level1 = 0;
    led_encoder->reset_code.duration1 = 0;
    
    *ret_encoder = &led_encoder->base;
    return ESP_OK;
}

// Initialize RMT for LED control
static esp_err_t init_rmt_led_strip(void) {
    ESP_LOGI(TAG, "Creating RMT TX channel for PL9823-F8 LED strip on GPIO%d", LED_PIN);
    
    rmt_tx_channel_config_t tx_chan_config = {};
    tx_chan_config.gpio_num = LED_PIN;
    tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_chan_config.resolution_hz = RMT_RESOLUTION_HZ;
    tx_chan_config.mem_block_symbols = 64;
    tx_chan_config.trans_queue_depth = 4;
    tx_chan_config.flags.invert_out = 0;
    tx_chan_config.flags.with_dma = 0;
    
    esp_err_t ret = rmt_new_tx_channel(&tx_chan_config, &led_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT TX channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Creating LED strip encoder");
    ret = rmt_new_led_strip_encoder(&led_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LED strip encoder: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Enabling RMT TX channel");
    ret = rmt_enable(led_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "RMT LED strip initialized successfully");
    return ESP_OK;
}

// Set LED color
static esp_err_t set_led_color(rgb_color_t color) {
    // PL9823-F8 color order: G-R-B (Green, Red, Blue)
    uint8_t led_strip_pixels[3] = {color.green, color.red, color.blue};
    
    rmt_transmit_config_t tx_config = {};
    tx_config.loop_count = 0; // no looping
    
    ESP_LOGI(TAG, "Setting LED color: R=%d, G=%d, B=%d", color.red, color.green, color.blue);
    esp_err_t ret = rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to transmit LED data: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for transmission to complete
    ret = rmt_tx_wait_all_done(led_chan, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wait for transmission completion: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

// Initialize ADC for NJL7302L-F3 light sensor
static esp_err_t init_adc_light_sensor(void) {
    ESP_LOGI(TAG, "Initializing ADC for NJL7302L-F3 light sensor on GPIO%d", LIGHT_SENSOR_PIN);
    
    // ADC1 Init
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = LIGHT_SENSOR_UNIT,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,  // Use default clock source for ESP32-C6
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config1, &adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }

    // ADC1 Config
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,        // 0-3.3V range for light sensor
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_oneshot_config_channel(adc1_handle, LIGHT_SENSOR_CHANNEL, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // ADC1 Calibration Init
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = LIGHT_SENSOR_UNIT,
        .chan = LIGHT_SENSOR_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_chan3_handle);
    if (ret == ESP_OK) {
        adc_calibration_enable = true;
        ESP_LOGI(TAG, "ADC calibration enabled");
    } else {
        ESP_LOGW(TAG, "ADC calibration not supported: %s", esp_err_to_name(ret));
        adc_calibration_enable = false;
    }

    ESP_LOGI(TAG, "NJL7302L-F3 light sensor ADC initialized successfully");
    return ESP_OK;
}

// Read light sensor value
static esp_err_t read_light_sensor(int *raw_value, int *voltage_mv) {
    esp_err_t ret = adc_oneshot_read(adc1_handle, LIGHT_SENSOR_CHANNEL, raw_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC: %s", esp_err_to_name(ret));
        return ret;
    }

    if (adc_calibration_enable && adc1_cali_chan3_handle) {
        ret = adc_cali_raw_to_voltage(adc1_cali_chan3_handle, *raw_value, voltage_mv);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to convert raw to voltage: %s", esp_err_to_name(ret));
            return ret;
        }
    } else {
        // Approximate conversion without calibration (for ESP32-C6 with 12-bit ADC)
        *voltage_mv = (*raw_value * 3300) / 4095;
    }

    return ESP_OK;
}

// Light sensor monitoring task
void light_sensor_task(void* arg) {
    ESP_LOGI(TAG, "Light sensor monitoring task started");
    
    while (1) {
        esp_err_t ret = read_light_sensor(&light_raw_value, &light_voltage_mv);
        if (ret == ESP_OK) {
            // Calculate approximate light intensity percentage (0-100%)
            // NJL7302L-F3 output increases with light intensity
            int light_percentage = (light_voltage_mv * 100) / 3300;
            
            ESP_LOGI(TAG, "Light Sensor - Raw: %d, Voltage: %dmV, Light: %d%%", 
                     light_raw_value, light_voltage_mv, light_percentage);
            
            // You can add logic here to control LED based on light levels
            // For example, turn on LED when it's dark (low light percentage)
            if (light_percentage < 20 && !led_is_on) {  // Dark environment
                ESP_LOGI(TAG, "Dark environment detected, could trigger LED");
                // Uncomment to auto-trigger LED in dark conditions:
                // if (!led_task_handle) {
                //     xTaskCreate(led_task, "led_task", 2048, NULL, 5, &led_task_handle);
                // }
            }
        }
        
        // Read light sensor every 2 seconds
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

// Simple GPIO-based IR receiver for GP1UXC41QS (ESP32-C6 compatible)
static void IRAM_ATTR ir_gpio_isr_handler(void* arg) {
    // Simple IR signal detection - trigger LED response
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static uint32_t last_trigger_time = 0;
    uint32_t current_time = esp_timer_get_time() / 1000;  // Convert to ms
    
    // Debounce: ignore signals within 100ms of each other
    if (current_time - last_trigger_time > 100) {
        last_trigger_time = current_time;
        // Notify from ISR - IR signal detected
        if (ir_receive_queue) {
            uint32_t signal_detected = 1;
            xQueueSendFromISR(ir_receive_queue, &signal_detected, &xHigherPriorityTaskWoken);
        }
    }
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// Initialize GP1UXC41QS IR receiver using GPIO interrupts
static esp_err_t init_ir_receiver(void) {
    ESP_LOGI(TAG_IR, "Initializing GP1UXC41QS IR receiver on GPIO%d", IR_RECEIVER_PIN);
    
    // Configure GPIO for IR receiver
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IR_RECEIVER_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,   // GP1UXC41QS needs pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE      // Trigger on falling edge
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_IR, "Failed to configure IR receiver GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Install GPIO ISR service
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG_IR, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Hook ISR handler for the specific GPIO pin
    ret = gpio_isr_handler_add(IR_RECEIVER_PIN, ir_gpio_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_IR, "Failed to add ISR handler: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG_IR, "GP1UXC41QS IR receiver initialized successfully");
    return ESP_OK;
}

// Initialize IR transmitter (LED on GPIO7) - Simple GPIO version
static esp_err_t init_ir_transmitter(void) {
    ESP_LOGI(TAG_IR, "Initializing IR transmitter on GPIO%d", IR_TRANSMITTER_PIN);
    
    // For simple GPIO-based transmission, just configure the pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IR_TRANSMITTER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_IR, "Failed to configure IR transmitter GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set initial state to low (off)
    gpio_set_level(IR_TRANSMITTER_PIN, 0);
    
    ESP_LOGI(TAG_IR, "IR transmitter initialized successfully");
    return ESP_OK;
}

// Initialize IR system (both receiver and transmitter)
static esp_err_t init_ir_system(void) {
    ESP_LOGI(TAG_IR, "Initializing IR system...");
    
    // Initialize transmitter first
    esp_err_t ret = init_ir_transmitter();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_IR, "Failed to initialize IR transmitter: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create receive queue
    ir_receive_queue = xQueueCreate(1, sizeof(uint32_t));  // Simple signal detection
    if (!ir_receive_queue) {
        ESP_LOGE(TAG_IR, "Failed to create IR receive queue");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize IR receiver using GPIO interrupts (ESP32-C6 compatible)
    ret = init_ir_receiver();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_IR, "Failed to initialize IR receiver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG_IR, "IR system initialized successfully");
    return ESP_OK;
}

// Initialize buzzer PWM
static esp_err_t init_buzzer(void) {
    ESP_LOGI(TAG, "Initializing buzzer on GPIO%d...", BUZZER_PIN);
    
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = BUZZER_PWM_MODE,
        .duty_resolution = BUZZER_RESOLUTION,
        .timer_num = BUZZER_PWM_TIMER,
        .freq_hz = BUZZER_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false
    };
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer for buzzer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure LEDC channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num = BUZZER_PIN,
        .speed_mode = BUZZER_PWM_MODE,
        .channel = BUZZER_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BUZZER_PWM_TIMER,
        .duty = 0, // Start with buzzer off
        .hpoint = 0,
        .flags = {
            .output_invert = 0
        }
    };
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel for buzzer: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Buzzer initialized successfully on GPIO%d", BUZZER_PIN);
    return ESP_OK;
}

// Turn buzzer on/off
static void set_buzzer(bool on) {
    uint32_t duty = on ? 128 : 0; // 50% duty cycle (128 out of 255 for 8-bit resolution)
    ledc_set_duty(BUZZER_PWM_MODE, BUZZER_PWM_CHANNEL, duty);
    ledc_update_duty(BUZZER_PWM_MODE, BUZZER_PWM_CHANNEL);
    ESP_LOGI(TAG, "Buzzer set to %s (duty: %lu)", on ? "ON" : "OFF", duty);
}

// Play startup buzzer sequence
// Play reminder buzzer for 3 seconds
static void play_reminder_buzzer(void) {
    ESP_LOGI(TAG, "Playing reminder buzzer (3 seconds)...");
    set_buzzer(true);
    vTaskDelay(pdMS_TO_TICKS(3000)); // 3 seconds
    set_buzzer(false);
    ESP_LOGI(TAG, "Reminder buzzer completed");
}

// Initialize OLED display
static esp_err_t init_oled(void) {
    ESP_LOGI(TAG, "Initializing OLED display (0.91 inch SSD1306)...");
    
    // I2C configuration
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize SSD1306 OLED
    uint8_t init_cmds[] = {
        SSD1306_DISPLAYOFF,
        SSD1306_SETDISPLAYCLOCKDIV, 0x80,
        SSD1306_SETMULTIPLEX, OLED_HEIGHT - 1,
        SSD1306_SETDISPLAYOFFSET, 0x00,
        SSD1306_SETSTARTLINE | 0x00,
        SSD1306_CHARGEPUMP, 0x14,
        SSD1306_MEMORYMODE, 0x00,
        SSD1306_SEGREMAP | 0x01,
        SSD1306_COMSCANDEC,
        SSD1306_SETCOMPINS, 0x02,
        SSD1306_SETCONTRAST, 0x8F,
        SSD1306_SETPRECHARGE, 0xF1,
        SSD1306_SETVCOMDETECT, 0x40,
        SSD1306_NORMALDISPLAY,
        SSD1306_DISPLAYON
    };
    
    // Send initialization commands
    for (int i = 0; i < sizeof(init_cmds); i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (OLED_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x00, true);  // Command mode
        i2c_master_write_byte(cmd, init_cmds[i], true);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SSD1306 init command %d failed: %s", i, esp_err_to_name(ret));
            return ret;
        }
    }
    
    // Clear display
    uint8_t clear_buf[OLED_WIDTH * OLED_HEIGHT / 8] = {0};
    
    // Set column address
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);  // Command mode
    i2c_master_write_byte(cmd, 0x21, true);  // Column address
    i2c_master_write_byte(cmd, 0x00, true);  // Start column
    i2c_master_write_byte(cmd, OLED_WIDTH - 1, true);  // End column
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    // Set page address
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);  // Command mode
    i2c_master_write_byte(cmd, 0x22, true);  // Page address
    i2c_master_write_byte(cmd, 0x00, true);  // Start page
    i2c_master_write_byte(cmd, (OLED_HEIGHT / 8) - 1, true);  // End page
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    // Send clear data
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x40, true);  // Data mode
    i2c_master_write(cmd, clear_buf, sizeof(clear_buf), true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    ESP_LOGI(TAG, "OLED display initialized successfully");
    return ESP_OK;
}

// Simple function to display text on OLED
static esp_err_t oled_display_text(const char* text) {
    // This is a simplified version - just shows that OLED is working
    // For now, we'll just turn on all pixels to show it's working
    uint8_t test_pattern[OLED_WIDTH * OLED_HEIGHT / 8];
    
    // Create a simple pattern to verify display is working
    for (int i = 0; i < sizeof(test_pattern); i++) {
        if (i < 64) {
            test_pattern[i] = 0xFF; // Top half bright
        } else {
            test_pattern[i] = 0x00; // Bottom half dark
        }
    }
    
    // Set addressing
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);  // Command mode
    i2c_master_write_byte(cmd, 0x21, true);  // Column address
    i2c_master_write_byte(cmd, 0x00, true);  // Start column
    i2c_master_write_byte(cmd, OLED_WIDTH - 1, true);  // End column
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);  // Command mode
    i2c_master_write_byte(cmd, 0x22, true);  // Page address
    i2c_master_write_byte(cmd, 0x00, true);  // Start page
    i2c_master_write_byte(cmd, (OLED_HEIGHT / 8) - 1, true);  // End page
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    // Send test pattern
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x40, true);  // Data mode
    i2c_master_write(cmd, test_pattern, sizeof(test_pattern), true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

// OLED display task - continuously display test text
static void oled_display_task(void *arg) {
    ESP_LOGI(TAG, "OLED display task started");
    
    while (1) {
        // Display test pattern
        esp_err_t ret = oled_display_text("TEST");
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to update OLED display: %s", esp_err_to_name(ret));
        }
        
        // Update every 2 seconds
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// IR receive task (simplified for ESP32-C6 using GPIO interrupts)
static void ir_receive_task(void *arg) {
    QueueHandle_t receive_queue = (QueueHandle_t)arg;
    uint32_t signal_detected;
    
    ESP_LOGI(TAG_IR, "IR receive task started - listening for GP1UXC41QS signals");
    
    while (1) {
        if (xQueueReceive(receive_queue, &signal_detected, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG_IR, "IR signal detected from GP1UXC41QS!");
            
            // Flash LED blue when IR signal detected
            uint32_t led_data = 0x0000FF00;  // Blue color for PL9823-F8
            
            rmt_transmit_config_t tx_config = {
                .loop_count = 0,
                .flags = {}
            };
            
            ESP_ERROR_CHECK(rmt_enable(led_chan));
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, &led_data, sizeof(led_data), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            ESP_ERROR_CHECK(rmt_disable(led_chan));
            
            // Turn off LED after 1 second
            vTaskDelay(pdMS_TO_TICKS(1000));
            led_data = 0x00000000;  // Off
            ESP_ERROR_CHECK(rmt_enable(led_chan));
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, &led_data, sizeof(led_data), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            ESP_ERROR_CHECK(rmt_disable(led_chan));
        }
    }
}

// Simple test IR signal transmission (safe for eyes)
static esp_err_t send_test_ir_signal(void) {
    ESP_LOGI(TAG_IR, "Sending simple test IR pulse (safe mode)");
    
    // Create simple 38kHz PWM signal for safe testing
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IR_TRANSMITTER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_IR, "Failed to configure IR transmitter GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Send a few safe pulses (100ms total duration)
    for (int i = 0; i < 10; i++) {
        gpio_set_level(IR_TRANSMITTER_PIN, 1);
        esp_rom_delay_us(13);  // ~38kHz half period
        gpio_set_level(IR_TRANSMITTER_PIN, 0);
        esp_rom_delay_us(13);  // ~38kHz half period
        
        // Pause between bursts for safety
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG_IR, "Test IR signal transmission completed safely");
    return ESP_OK;
}

// Power management initialization for thermal optimization
static void init_power_management(void) {
    ESP_LOGI(TAG, "Initializing power management for thermal optimization...");
    
    // Configure CPU frequency scaling for heat reduction
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 80,     // 最大80MHz（デフォルト160MHzから削減）
        .min_freq_mhz = 10,     // 最小10MHz（アイドル時）
        .light_sleep_enable = true  // ライトスリープ有効
    };
    
    esp_err_t ret = esp_pm_configure(&pm_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Power management configured: Max=%dMHz, Min=%dMHz, Light sleep enabled", 
                 pm_config.max_freq_mhz, pm_config.min_freq_mhz);
    } else {
        ESP_LOGW(TAG, "Power management configuration failed: %s", esp_err_to_name(ret));
    }
}

// LED task: turns on blue LED for 3 seconds then turns off
void led_task(void* arg) {
    ESP_LOGI(TAG, "LED task started - turning on blue LED for 3 seconds");
    
    // Turn on LED with blue color
    rgb_color_t blue_color = {.red = 0, .green = 0, .blue = 255};
    esp_err_t ret = set_led_color(blue_color);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LED color: %s", esp_err_to_name(ret));
    } else {
        led_is_on = true;
    }
    
    // Wait for 3 seconds
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    // Turn off LED (set to black)
    rgb_color_t off_color = {.red = 0, .green = 0, .blue = 0};
    ret = set_led_color(off_color);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to turn off LED: %s", esp_err_to_name(ret));
    } else {
        led_is_on = false;
    }
    
    ESP_LOGI(TAG, "LED task completed - LED turned off");
    
    // Clear the task handle when done
    led_task_handle = NULL;
    
    // Delete this task
    vTaskDelete(NULL);
}

// MQTT event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        // Subscribe to LED control topic
        msg_id = esp_mqtt_client_subscribe(client, "led/control", 0);
        ESP_LOGI(TAG, "sent subscribe to led/control, msg_id=%d", msg_id);
        
        // Subscribe to IR test topic
        msg_id = esp_mqtt_client_subscribe(client, "ir/test", 0);
        ESP_LOGI(TAG, "sent subscribe to ir/test, msg_id=%d", msg_id);
        
        // Subscribe to calendar reminders topic
        msg_id = esp_mqtt_client_subscribe(client, "calendar/test/reminders", 0);
        ESP_LOGI(TAG, "sent subscribe to calendar/test/reminders, msg_id=%d", msg_id);
        
        // Optimize power after successful connection (thermal management)
        ESP_LOGI(TAG, "Optimizing power settings after MQTT connection...");
        esp_wifi_set_ps(WIFI_PS_MAX_MODEM);  // より強力な電力節約モード
        esp_wifi_set_max_tx_power(44);       // さらに送信電力を下げる (11dBm)
        
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        
        // Handle LED control commands
        if (strncmp(event->topic, "led/control", event->topic_len) == 0) {
            if (strncmp(event->data, "on", event->data_len) == 0) {
                ESP_LOGI(TAG, "MQTT: LED ON command received");
                // Only start LED task if it's not already running
                if (!led_task_handle && !led_is_on) {
                    xTaskCreate(led_task, "led_task", 2048, NULL, 5, &led_task_handle);
                } else {
                    ESP_LOGI(TAG, "LED task already running or LED is on");
                }
            } else if (strncmp(event->data, "off", event->data_len) == 0) {
                ESP_LOGI(TAG, "MQTT: LED OFF command received");
                // Force turn off LED
                if (led_task_handle) {
                    vTaskDelete(led_task_handle);
                    led_task_handle = NULL;
                }
                rgb_color_t off_color = {.red = 0, .green = 0, .blue = 0};
                set_led_color(off_color);
                led_is_on = false;
                ESP_LOGI(TAG, "LED turned off immediately");
            }
        }
        // Handle IR test commands
        else if (strncmp(event->topic, "ir/test", event->topic_len) == 0) {
            if (strncmp(event->data, "send", event->data_len) == 0) {
                ESP_LOGI(TAG, "MQTT: IR send test command received");
                esp_err_t ret = send_test_ir_signal();
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "IR test signal sent successfully");
                } else {
                    ESP_LOGE(TAG, "Failed to send IR test signal: %s", esp_err_to_name(ret));
                }
            }
        }
        // Handle calendar reminder messages
        else if (strncmp(event->topic, "calendar/test/reminders", event->topic_len) == 0) {
            ESP_LOGI(TAG, "MQTT: Calendar reminder received: %.*s", event->data_len, event->data);
            // Play 3-second reminder buzzer
            play_reminder_buzzer();
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGE(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGE(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGE(TAG, "Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
                                strerror(event->error_handle->esp_transport_sock_errno));
        } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            ESP_LOGE(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
            // MQTT接続拒否の詳細コードを表示
            switch(event->error_handle->connect_return_code) {
                case 1:
                    ESP_LOGE(TAG, "Connection refused: unacceptable protocol version");
                    break;
                case 2:
                    ESP_LOGE(TAG, "Connection refused: identifier rejected");
                    break;
                case 3:
                    ESP_LOGE(TAG, "Connection refused: server unavailable");
                    break;
                case 4:
                    ESP_LOGE(TAG, "Connection refused: bad username or password");
                    break;
                case 5:
                    ESP_LOGE(TAG, "Connection refused: not authorized");
                    break;
                default:
                    ESP_LOGE(TAG, "Connection refused: unknown reason (%d)", event->error_handle->connect_return_code);
                    break;
            }
        }
        ESP_LOGI(TAG, "Error type: %d", event->error_handle->error_type);
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

// Initialize MQTT with TLS
static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {};
    
    // ブローカー設定
    mqtt_cfg.broker.address.hostname = MQTT_BROKER_HOST;
    mqtt_cfg.broker.address.port = MQTT_BROKER_PORT;
    mqtt_cfg.broker.address.transport = MQTT_TRANSPORT_OVER_SSL;
    
    // クライアントIDを明示的に設定
    mqtt_cfg.credentials.client_id = "esp32c6_led_device";
    
    // 認証情報が空文字列でない場合のみ設定
    if (strlen(MQTT_USERNAME) > 0) {
        mqtt_cfg.credentials.username = MQTT_USERNAME;
    }
    if (strlen(MQTT_PASSWORD) > 0) {
        mqtt_cfg.credentials.authentication.password = MQTT_PASSWORD;
    }
    
    // TLS設定 - EMQXクラウド用（SNI有効 + 証明書バンドル）
    mqtt_cfg.broker.verification.use_global_ca_store = false;  // global_cacertエラー回避
    mqtt_cfg.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
    mqtt_cfg.broker.verification.common_name = MQTT_BROKER_HOST;  // SNI設定
    mqtt_cfg.broker.verification.skip_cert_common_name_check = false;  // 証明書の検証を有効
    
    // 接続タイムアウト設定
    mqtt_cfg.network.timeout_ms = 30000;
    mqtt_cfg.network.refresh_connection_after_ms = 60000;
    
    // セッション設定
    mqtt_cfg.session.keepalive = 60;
    mqtt_cfg.session.disable_clean_session = false;
    
    ESP_LOGI(TAG, "=== MQTT Configuration ===");
    ESP_LOGI(TAG, "Broker: %s:%d", MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    ESP_LOGI(TAG, "Username: %s", MQTT_USERNAME);
    ESP_LOGI(TAG, "Client ID: esp32c6_led_device");
    ESP_LOGI(TAG, "TLS: Enabled with certificate bundle + SNI");
    ESP_LOGI(TAG, "SNI Hostname: %s", MQTT_BROKER_HOST);
    
    ESP_LOGI(TAG, "Initializing MQTT client for TLS connection...");
    client = esp_mqtt_client_init(&mqtt_cfg);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }
    
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    ESP_LOGI(TAG, "MQTT TLS client started");
}

// WiFi event handler for STA mode
static void wifi_event_handler_sta(void* arg, esp_event_base_t event_base,
                                   int32_t event_id, void* event_data) {
    static int retry_count = 0;
    const int MAX_RETRIES = 3;
    
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi station started, connecting...");
        retry_count = 0;
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGI(TAG, "WiFi disconnected, reason: %d", event->reason);
        
        if (retry_count < MAX_RETRIES) {
            retry_count++;
            ESP_LOGI(TAG, "Retrying WiFi connection (%d/%d) in 5 seconds...", retry_count, MAX_RETRIES);
            vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait 5 seconds before retry
            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "Max WiFi retries reached. Restarting...");
            esp_restart();
        }
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        retry_count = 0; // Reset retry count on successful connection
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "WiFi connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
        ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&event->ip_info.netmask));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        
        // Start MQTT after getting IP
        ESP_LOGI(TAG, "Starting MQTT connection to %s:%d", MQTT_BROKER_HOST, MQTT_BROKER_PORT);
        mqtt_app_start();
    }
}

// Initialize WiFi in STA mode for MQTT connection
static void wifi_init_sta_mqtt(void) {
    ESP_LOGI(TAG, "Initializing WiFi...");
    ESP_LOGI(TAG, "Target SSID: %s", WIFI_SSID);
    
    wifi_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler_sta,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler_sta,
                                                        NULL,
                                                        &instance_got_ip));
    
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASSWORD);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    // Add connection stability settings
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    wifi_config.sta.scan_method = WIFI_FAST_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifi_config.sta.threshold.rssi = -127;
    wifi_config.sta.failure_retry_cnt = 10;  // 再試行回数を増加
    wifi_config.sta.listen_interval = 1;     // ビーコン受信間隔を最短に
    
    ESP_LOGI(TAG, "Setting WiFi configuration...");
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    // Power management optimization for heat reduction
    ESP_LOGI(TAG, "Optimizing WiFi power settings for thermal management...");
    
    // Use minimum power save mode instead of completely disabling it
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));  // 最小電力管理
    
    // Reduce transmission power to lower heat generation (52 = 13dBm instead of 78 = 19.5dBm)
    esp_wifi_set_max_tx_power(52);  // 送信電力を下げて発熱抑制
    
    // Set moderate inactive timeout
    esp_wifi_set_inactive_time(WIFI_IF_STA, 30);  // 30秒でバランス調整
    
    // Set country configuration for better signal handling
    wifi_country_t country_config = {
        .cc = "JP",
        .schan = 1,
        .nchan = 14,
        .max_tx_power = 20,
        .policy = WIFI_COUNTRY_POLICY_MANUAL
    };
    ESP_ERROR_CHECK(esp_wifi_set_country(&country_config));
    
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi initialization completed. Starting connection...");
}

extern "C" void app_main(void) {
    // NVSストレージを初期化
    ESP_ERROR_CHECK(nvs_flash_init());

    // 電力管理初期化（発熱対策）
    init_power_management();

    // RMT LED strip初期化
    esp_err_t ret = init_rmt_led_strip();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize RMT LED strip: %s", esp_err_to_name(ret));
        return;
    }

    // ADC light sensor初期化 
    ret = init_adc_light_sensor();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC light sensor: %s", esp_err_to_name(ret));
        return;
    }

    // IR system初期化 (GPIO7送信、GPIO8受信)
    ret = init_ir_system();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize IR system: %s", esp_err_to_name(ret));
        return;
    }

    // IR receiver初期化 (GP1UXC41QS on GPIO8)
    ret = init_ir_receiver();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize IR receiver: %s", esp_err_to_name(ret));
        return;
    }

    // Buzzer初期化
    ret = init_buzzer();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize buzzer: %s", esp_err_to_name(ret));
        return;
    }

    // OLED Display初期化
    ret = init_oled();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize OLED display: %s", esp_err_to_name(ret));
        return;
    }

    // 初期状態でLEDを消灯
    rgb_color_t off_color = {.red = 0, .green = 0, .blue = 0};
    set_led_color(off_color);

    // 光センサー監視タスクを開始
    xTaskCreate(light_sensor_task, "light_sensor_task", 4096, NULL, 5, &light_sensor_task_handle);

    // IR受信監視タスクを開始
    xTaskCreate(ir_receive_task, "ir_receive_task", 4096, ir_receive_queue, 5, NULL);

    // OLED表示タスクを開始
    xTaskCreate(oled_display_task, "oled_display_task", 4096, NULL, 5, &oled_display_task_handle);

    ESP_LOGI(TAG, "ESP32-C6 IoT Controller started with:");
    ESP_LOGI(TAG, "- PL9823-F8 RGB LED on GPIO%d", LED_PIN);
    ESP_LOGI(TAG, "- NJL7302L-F3 Light Sensor on GPIO%d", LIGHT_SENSOR_PIN);
    ESP_LOGI(TAG, "- IR LED Transmitter on GPIO%d", IR_LED_PIN);
    ESP_LOGI(TAG, "- GP1UXC41QS IR Receiver on GPIO%d", IR_RECEIVER_PIN);
    ESP_LOGI(TAG, "- Buzzer on GPIO%d (MQTT reminder alerts)", BUZZER_PIN);
    ESP_LOGI(TAG, "- OLED Display 0.91\" SSD1306 - SDA:GPIO%d, SCL:GPIO%d", I2C_SDA_PIN, I2C_SCL_PIN);
    ESP_LOGI(TAG, "Connecting to WiFi: %s", WIFI_SSID);
    
    // WiFi初期化してMQTT接続
    wifi_init_sta_mqtt();
}
