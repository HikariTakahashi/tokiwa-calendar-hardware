# Tokiwa Calendar Hardware - MQTT Buzzer Controller

本番環境対応のESP32-C6 MQTTブザーコントローラー

## 概要

このプロジェクトは、MQTT経由でブザーを制御できるESP32-C6ベースのIoTデバイスです。
Web UI とMQTT APIの両方でブザーを操作でき、本番環境でのセキュリティと安全性を考慮して設計されています。

## 主な機能

### セキュリティ機能
- TLS暗号化によるMQTT通信
- 認証機能（ユーザー名/パスワード）
- セキュリティヘッダー付きWeb UI
- 操作ログの記録
- Last Will Testament による切断検知

### 安全機能
- ブザーの最大動作時間制限（デフォルト30秒）
- 自動停止機能
- ハートビート監視
- エラーハンドリング

### 監視機能
- デバイス状態のリアルタイム監視
- 定期的なハートビート送信
- システム情報の API 提供

## 本番環境設定

### 1. MQTT ブローカー設定

`include/config.h` ファイルで以下を設定：

```c
#define MQTT_BROKER_HOST           "your-mqtt-broker.com"
#define MQTT_BROKER_PORT           8883  // TLS port
#define MQTT_USERNAME              "esp32c6_device"
#define MQTT_PASSWORD              "your_secure_password"
```

### 2. デバイス設定

```c
#define DEVICE_SERIAL_NUMBER       "ESP32C6_001"  // 一意のシリアル番号
#define FIRMWARE_VERSION           "1.0.0"
#define MANUFACTURER               "Tokiwa"
```

### 3. TLS証明書設定

本番環境では適切なCA証明書を設定してください：
- `CONFIG_MBEDTLS_CERTIFICATE_BUNDLE=y`
- `CONFIG_ESP_TLS_SKIP_SERVER_CERT_VERIFY=n`

## MQTT API

### トピック構成

```
tokiwa/devices/{device_id}/buzzer/control    # ブザー制御
tokiwa/devices/{device_id}/status           # ステータス通知
tokiwa/devices/{device_id}/heartbeat        # ハートビート
```

### ブザー制御

**トピック:** `tokiwa/devices/esp32c6_buzzer_001/buzzer/control`

**コマンド:**
- `on` - ブザーON
- `off` - ブザーOFF

**例:**
```bash
# mosquitto_pub を使用
mosquitto_pub -h your-mqtt-broker.com -p 8883 --cafile ca.crt \
  -u esp32c6_device -P your_secure_password \
  -t "tokiwa/devices/esp32c6_buzzer_001/buzzer/control" \
  -m "on"
```

### ステータス監視

**トピック:** `tokiwa/devices/esp32c6_buzzer_001/status`

**受信例:**
```json
{
  "buzzer": "on",
  "source": "mqtt",
  "timestamp": 1632345678
}
```

### ハートビート

**トピック:** `tokiwa/devices/esp32c6_buzzer_001/heartbeat`

**受信例:**
```json
{
  "uptime_min": 120,
  "buzzer_active": false,
  "free_heap": 245760
}
```

## Web API

### デバイス状態取得

**エンドポイント:** `GET /api/status`

**レスポンス例:**
```json
{
  "device_id": "esp32c6_buzzer_001",
  "firmware": "1.0.0",
  "manufacturer": "Tokiwa",
  "uptime_seconds": 7200,
  "free_heap": 245760,
  "buzzer_active": false,
  "mqtt_connected": true,
  "wifi_connected": true
}
```

## セットアップ手順

### 1. 開発環境
```bash
# PlatformIO でビルド
pio build

# フラッシュ書き込み
pio upload

# シリアルモニター
pio monitor
```

### 2. 初期設定
1. ESP32C6 を起動
2. WiFi AP `ESP32C6_Config` に接続（パスワード: `configure123`）
3. ブラウザで `192.168.4.1` にアクセス
4. 本番環境のWiFi設定を入力

### 3. 動作確認
1. 制御ページ (`/control`) でWeb経由の動作確認
2. MQTT クライアントで外部制御の確認
3. ステータス API (`/api/status`) でシステム状態確認

## セキュリティの考慮事項

### 本番環境での推奨設定
1. **強力なパスワード**: MQTT認証に強力なパスワードを使用
2. **証明書検証**: TLS証明書の検証を有効にする
3. **ファイアウォール**: 必要なポートのみ開放
4. **アクセス制御**: MQTTブローカーでACL設定
5. **監視**: ログ監視とアラート設定

### セキュリティ機能
- X-Content-Type-Options: nosniff
- X-Frame-Options: DENY
- 操作ログの記録
- 異常検知とアラート

## トラブルシューティング

### よくある問題

**MQTT接続エラー:**
- ブローカーホスト/ポートの確認
- 認証情報の確認
- TLS証明書の確認

**ブザーが動作しない:**
- GPIO設定の確認
- タスク作成エラーの確認
- 安全制限時間の確認

**WiFi接続エラー:**
- SSID/パスワードの確認
- 電波強度の確認
- APモードでの再設定

### ログ確認
```bash
# シリアルモニターでログ確認
pio monitor

# 重要なログレベル
[I] - 情報
[W] - 警告  
[E] - エラー
```

## ライセンス

MIT License

## サポート

問題や質問がある場合は、GitHubのIssueを作成してください。
