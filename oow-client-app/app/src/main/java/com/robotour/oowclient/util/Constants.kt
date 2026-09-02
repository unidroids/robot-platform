package com.robotour.oowclient.util

import java.util.UUID

object Constants {
    val SERVICE_UUID: UUID = UUID.fromString("87654321-4321-4321-4321-abcdef987654")
    val CHAR_COMMAND_UUID: UUID = UUID.fromString("87654321-4321-4321-4321-abcdef987655")
    val CHAR_HEARTBEAT_UUID: UUID = UUID.fromString("87654321-4321-4321-4321-abcdef987656")
    val CHAR_TELEMETRY_UUID: UUID = UUID.fromString("87654321-4321-4321-4321-abcdef987657")
    val CCCD_UUID: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

    const val HEARTBEAT_INTERVAL_MS = 333L
    const val DISCOVERY_TIMEOUT_MS = 10000L
    const val SCAN_DURATION_MS = 10000L
    const val DISCOVERY_STABILIZE_DELAY_MS = 250L
    const val RETRY_DELAY_MS = 1000L
    const val DISCONNECT_RETRY_DELAY_MS = 1500L
    const val PREFS_NAME = "OowClientPrefs"
    const val PREF_CLIENT_NAME = "client_name"
    const val PERMISSION_REQUEST_CODE = 100
}
