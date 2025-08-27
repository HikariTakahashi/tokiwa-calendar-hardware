#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "driver/gpio.h"
#include "mqtt_client.h"
#include "esp_crt_bundle.h"
#include "config.h"

// ROM delay function (provided in ROM)
extern "C" void ets_delay_us(uint32_t us);

static const char *TAG = "buzzer_mqtt";

// MQTT
static esp_mqtt_client_handle_t client = NULL;
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// Buzzer on single pin
static const gpio_num_t BUZZER_PIN = GPIO_NUM_2;
static TaskHandle_t buzzer_task_handle = NULL;

// Tone generation task: generates ~1kHz square wave on buzzer pin
void buzzer_task(void* arg) {
    const uint32_t half_period_us = 500; // ~1 kHz tone
    while (1) {
        gpio_set_level(BUZZER_PIN, 1);
        ets_delay_us(half_period_us);
        gpio_set_level(BUZZER_PIN, 0);
        ets_delay_us(half_period_us);
    }
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
        // Subscribe to buzzer control topic
        msg_id = esp_mqtt_client_subscribe(client, "buzzer/control", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
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
        
        // Handle buzzer control commands
        if (strncmp(event->topic, "buzzer/control", event->topic_len) == 0) {
            if (strncmp(event->data, "on", event->data_len) == 0) {
                ESP_LOGI(TAG, "MQTT: Buzzer ON command received");
                if (!buzzer_task_handle) {
                    xTaskCreate(buzzer_task, "buzzer", 2048, NULL, 5, &buzzer_task_handle);
                }
            } else if (strncmp(event->data, "off", event->data_len) == 0) {
                ESP_LOGI(TAG, "MQTT: Buzzer OFF command received");
                if (buzzer_task_handle) {
                    vTaskDelete(buzzer_task_handle);
                    buzzer_task_handle = NULL;
                    gpio_set_level(BUZZER_PIN, 0);
                }
            }
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
    mqtt_cfg.credentials.client_id = "esp32c6_buzzer_device";
    
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
    ESP_LOGI(TAG, "Client ID: esp32c6_buzzer_device");
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
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi station started, connecting...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGI(TAG, "WiFi disconnected, reason: %d, retrying...", event->reason);
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
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
    
    ESP_LOGI(TAG, "Setting WiFi configuration...");
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi initialization completed. Starting connection...");
}

extern "C" void app_main(void) {
    // NVSストレージを初期化
    ESP_ERROR_CHECK(nvs_flash_init());

    // GPIOを初期化（ブザー）
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << BUZZER_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(BUZZER_PIN, 0);

    ESP_LOGI(TAG, "ESP32-C6 MQTT Buzzer Controller started");
    ESP_LOGI(TAG, "Connecting to WiFi: %s", WIFI_SSID);
    
    // WiFi初期化してMQTT接続
    wifi_init_sta_mqtt();
}
