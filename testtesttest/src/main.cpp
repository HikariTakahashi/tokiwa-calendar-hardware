#include <stdio.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include <stdint.h>  // for uint32_t
#include "esp_wifi_types.h"  // for MAC address formatting

// ROM delay function (provided in ROM)
extern "C" void ets_delay_us(uint32_t us);

static const char *TAG = "wifi_web";

// WiFi AP config
#define EXAMPLE_ESP_WIFI_SSID      "ESP32C6_Config"
#define EXAMPLE_ESP_WIFI_PASS      "configure123"
#define EXAMPLE_ESP_WIFI_CHANNEL   1
#define EXAMPLE_MAX_STA_CONN       4

// Buzzer on single pin
static const gpio_num_t BUZZER_PIN = GPIO_NUM_2;
static TaskHandle_t buzzer_task_handle = NULL;

// URL-decode percent-encoded strings
static void url_decode(const char *src, char *dst, size_t dst_len) {
    char a, b;
    while (*src && dst_len > 1) {
        if (*src == '%' && (a = src[1]) && (b = src[2]) && isxdigit(a) && isxdigit(b)) {
            if (a >= 'a') a -= 'a' - 'A';
            if (a >= 'A') a -= ('A' - 10); else a -= '0';
            if (b >= 'a') b -= 'a' - 'A';
            if (b >= 'A') b -= ('A' - 10); else b -= '0';
            *dst++ = 16 * a + b;
            src += 3;
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
        dst_len--;
    }
    *dst = '\0';
}

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station %02x:%02x:%02x:%02x:%02x:%02x join, AID=%d", \
                 event->mac[0], event->mac[1], event->mac[2], event->mac[3], event->mac[4], event->mac[5], event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station %02x:%02x:%02x:%02x:%02x:%02x leave, AID=%d", \
                 event->mac[0], event->mac[1], event->mac[2], event->mac[3], event->mac[4], event->mac[5], event->aid);
    }
}

// Handler for STA got IP event
static void sta_got_ip_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
    ESP_LOGI(TAG, "STA IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
}

// Initialize WiFi in STA mode with given credentials
static void wifi_init_sta(const char* ssid, const char* pwd) {
    // Register handler for STA IP event
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &sta_got_ip_handler, NULL, NULL));
    // Create default STA network interface
    esp_netif_create_default_wifi_sta();
    // Switch to AP+STA mode (keep AP active)
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, pwd, sizeof(wifi_config.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    // Connect in STA mode
    ESP_ERROR_CHECK(esp_wifi_connect());
    ESP_LOGI(TAG, "wifi_init_sta finished. SSID:%s PWD:%s", ssid, pwd);
}

// HTTP GET handler for root
esp_err_t root_get_handler(httpd_req_t *req) {
    const char resp[] = "<h1>ESP32-C6 WiFi設定サンプル</h1>"
                       "<form method='POST' action='/save'>"
                       "SSID: <input name='ssid'><br>"
                       "Password: <input name='pwd' type='password'><br>"
                       "<input type='submit' value='保存'>"
                       "</form>";
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// HTTP POST handler for /save (ダミー)
esp_err_t save_post_handler(httpd_req_t *req) {
    char buf[256];
    int ret, remaining = req->content_len;
    int idx = 0;
    while (remaining > 0 && idx < sizeof(buf)-1) {
        ret = httpd_req_recv(req, buf + idx, MIN(remaining, sizeof(buf)-1-idx));
        if (ret <= 0) return ESP_FAIL;
        remaining -= ret;
        idx += ret;
    }
    buf[idx] = '\0';

    // Parse parameters
    char ssid[32] = {0}, pwd[64] = {0};
    char* token = strtok(buf, "&");
    while (token) {
        if (strncmp(token, "ssid=", 5) == 0) {
            url_decode(token + 5, ssid, sizeof(ssid));
        } else if (strncmp(token, "pwd=", 4) == 0) {
            url_decode(token + 4, pwd, sizeof(pwd));
        }
        token = strtok(NULL, "&");
    }
    ESP_LOGI(TAG, "Received SSID:%s", ssid);

    // ステーションモードに切り替え
    wifi_init_sta(ssid, pwd);
    // 設定後すぐに操作画面へリダイレクト
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/control");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// HTTP GET handler for /control
esp_err_t control_get_handler(httpd_req_t *req) {
    // Use inline window.fetch to send commands without shadowing
    const char resp[] = "<h1>操作ページ</h1>"
                       "<button onclick=\"window.fetch('/buzzer/on').then(r=>r.text()).then(t=>alert(t))\">Buzzer ON</button>"
                       "<button onclick=\"window.fetch('/buzzer/off').then(r=>r.text()).then(t=>alert(t))\">Buzzer OFF</button>";
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

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

// Buzzer on handler
esp_err_t buzzer_on_handler(httpd_req_t* req) {
    if (!buzzer_task_handle) {
        xTaskCreate(buzzer_task, "buzzer", 2048, NULL, 5, &buzzer_task_handle);
    }
    httpd_resp_sendstr(req, "Buzzer ON");
    return ESP_OK;
}
// Buzzer off handler
esp_err_t buzzer_off_handler(httpd_req_t* req) {
    if (buzzer_task_handle) {
        vTaskDelete(buzzer_task_handle);
        buzzer_task_handle = NULL;
        gpio_set_level(BUZZER_PIN, 0);
    }
    httpd_resp_sendstr(req, "Buzzer OFF");
    return ESP_OK;
}

void start_webserver(void) {
    // HTTP server configuration
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    // Register URI handlers
    httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler, .user_ctx = NULL };
    httpd_uri_t save = { .uri = "/save", .method = HTTP_POST, .handler = save_post_handler, .user_ctx = NULL };
    httpd_uri_t control = { .uri = "/control", .method = HTTP_GET, .handler = control_get_handler, .user_ctx = NULL };
    httpd_uri_t buzzer_on = { .uri = "/buzzer/on", .method = HTTP_GET, .handler = buzzer_on_handler, .user_ctx = NULL };
    httpd_uri_t buzzer_off = { .uri = "/buzzer/off", .method = HTTP_GET, .handler = buzzer_off_handler, .user_ctx = NULL };
    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &save);
    httpd_register_uri_handler(server, &control);
    httpd_register_uri_handler(server, &buzzer_on);
    httpd_register_uri_handler(server, &buzzer_off);
}

void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_instance_t instance_any_id;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.ap.ssid, EXAMPLE_ESP_WIFI_SSID, sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID);
    strncpy((char*)wifi_config.ap.password, EXAMPLE_ESP_WIFI_PASS, sizeof(wifi_config.ap.password));
    wifi_config.ap.channel = EXAMPLE_ESP_WIFI_CHANNEL;
    wifi_config.ap.max_connection = EXAMPLE_MAX_STA_CONN;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
    // APモードのIPアドレスをシリアル出力
    {
        esp_netif_t* ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
        esp_netif_ip_info_t ip_info;
        esp_netif_get_ip_info(ap_netif, &ip_info);
        ESP_LOGI(TAG, "AP IP Address: " IPSTR, IP2STR(&ip_info.ip));
    }
}

extern "C" void app_main(void) {
    // reduce verbose Wi-Fi and HTTPD logs
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("httpd_txrx", ESP_LOG_WARN);
    ESP_ERROR_CHECK(nvs_flash_init());

    // Initialize buzzer pin
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << BUZZER_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    wifi_init_softap();
    start_webserver();
}
