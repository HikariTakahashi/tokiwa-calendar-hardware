#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
環境変数を.envファイルから読み込んでビルドフラグとして設定するスクリプト
PlatformIOのextra_scriptsで使用
"""

import os
from pathlib import Path

def load_env_vars():
    """
    .envファイルから環境変数を読み込み
    """
    env_file = Path(".env")
    env_vars = {}
    
    if env_file.exists():
        with open(env_file, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#') and '=' in line:
                    key, value = line.split('=', 1)
                    key = key.strip()
                    value = value.strip()
                    # 引用符を削除
                    if value.startswith('"') and value.endswith('"'):
                        value = value[1:-1]
                    elif value.startswith("'") and value.endswith("'"):
                        value = value[1:-1]
                    env_vars[key] = value
    else:
        print("Warning: .env file not found. Using default values.")
    
    return env_vars

def setup_build_flags(env):
    """
    環境変数をビルドフラグとして設定
    """
    env_vars = load_env_vars()
    
    # WiFi設定
    if 'WIFI_SSID' in env_vars:
        env.Append(CPPDEFINES=[('WIFI_SSID', f'\\"{env_vars["WIFI_SSID"]}\\"')])
    if 'WIFI_PASSWORD' in env_vars:
        env.Append(CPPDEFINES=[('WIFI_PASSWORD', f'\\"{env_vars["WIFI_PASSWORD"]}\\"')])
    
    # MQTT設定
    if 'MQTT_BROKER_HOST' in env_vars:
        env.Append(CPPDEFINES=[('MQTT_BROKER_HOST', f'\\"{env_vars["MQTT_BROKER_HOST"]}\\"')])
    if 'MQTT_BROKER_PORT' in env_vars:
        env.Append(CPPDEFINES=[('MQTT_BROKER_PORT', env_vars['MQTT_BROKER_PORT'])])
    if 'MQTT_USERNAME' in env_vars:
        env.Append(CPPDEFINES=[('MQTT_USERNAME', f'\\"{env_vars["MQTT_USERNAME"]}\\"')])
    if 'MQTT_PASSWORD' in env_vars:
        env.Append(CPPDEFINES=[('MQTT_PASSWORD', f'\\"{env_vars["MQTT_PASSWORD"]}\\"')])
    
    # デバイス設定
    if 'DEVICE_SERIAL_NUMBER' in env_vars:
        env.Append(CPPDEFINES=[('DEVICE_SERIAL_NUMBER', f'\\"{env_vars["DEVICE_SERIAL_NUMBER"]}\\"')])
    if 'MANUFACTURER' in env_vars:
        env.Append(CPPDEFINES=[('MANUFACTURER', f'\\"{env_vars["MANUFACTURER"]}\\"')])
    
    print("Environment variables loaded from .env file")

# PlatformIOから呼び出される関数
Import("env")
setup_build_flags(env)
