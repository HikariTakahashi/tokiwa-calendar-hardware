#ifndef CONFIG_H
#define CONFIG_H

// 本番環境設定ファイル
// 機密情報は.envファイルから読み込み、このファイルには含めない

// MQTT Broker Configuration - 本番環境用
// 実際の値は.envファイルで設定
#ifndef MQTT_BROKER_HOST
#define MQTT_BROKER_HOST           "your-emqx-host.emqxsl.com"  // .envで上書き
#endif

#ifndef MQTT_BROKER_PORT
#define MQTT_BROKER_PORT           8883                         // TLS暗号化ポート
#endif

#ifndef MQTT_USERNAME
#define MQTT_USERNAME              "production_user"           // .envで上書き
#endif

#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD              "secure_password"           // .envで上書き
#endif

// Device Configuration
#define DEVICE_SERIAL_NUMBER       "ESP32C6_001"
#define FIRMWARE_VERSION           "1.0.0"
#define MANUFACTURER               "Tokiwa"

// Security Configuration
#define MAX_RECONNECT_ATTEMPTS     10
#define HEARTBEAT_INTERVAL_MS      60000                    // 1分間隔
#define CONNECTION_TIMEOUT_MS      30000                    // 30秒タイムアウト

// WiFi Configuration
#define MAX_WIFI_RETRY_ATTEMPTS    5
#define WIFI_CONNECT_TIMEOUT_MS    20000                    // 20秒タイムアウト

// WiFi Credentials - 本番環境用
// 実際の値は.envファイルで設定
#ifndef WIFI_SSID
#define WIFI_SSID                  "production_wifi"          // .envで上書き
#endif

#ifndef WIFI_PASSWORD  
#define WIFI_PASSWORD              "secure_wifi_password"     // .envで上書き
#endif

// 自動WiFi接続を有効にする場合は1、APモードを強制する場合は0
#define AUTO_WIFI_CONNECT          1

// Hardware Configuration
#define BUZZER_FREQUENCY_HZ        1000                     // ブザー周波数
#define BUZZER_MAX_DURATION_MS     30000                    // 最大30秒動作

#endif // CONFIG_H
