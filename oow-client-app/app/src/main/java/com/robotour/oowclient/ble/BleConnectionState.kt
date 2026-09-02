package com.robotour.oowclient.ble

enum class BleConnectionState {
    DISCONNECTED,
    DISCONNECTING,
    CONNECTING,
    SEARCHING_SERVICES,
    READY,
    BLUETOOTH_OFF,
    ERROR
}
