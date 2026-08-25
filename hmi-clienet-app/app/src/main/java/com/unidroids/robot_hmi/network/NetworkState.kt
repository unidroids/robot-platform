package com.unidroids.robot_hmi.network

data class NetworkState(
    val isHmiConnected: Boolean = false,
    val isQrServerRunning: Boolean = false,
    val isClientConnected: Boolean = false,
    val isTerminalServerRunning: Boolean = false,
    val isTerminalClientConnected: Boolean = false,
    val lastCommand: String = "",
    val scannerTriggered: Boolean = false,
    val lastQrCode: String? = null,
    val blinkEvent: BlinkConfig? = null,
    val soundEvent: SoundConfig? = null,
    val messageEvent: MessageConfig? = null
)

data class MessageConfig(
    val header: String,
    val text: String,
    val buttons: List<MessageButton>
)

data class MessageButton(
    val id: String,
    val text: String
)

data class BlinkConfig(
    val color: String,
    val frequency: Float,
    val duration: Long,
    val timestamp: Long = System.currentTimeMillis()
)

data class SoundConfig(
    val name: String,
    val timestamp: Long = System.currentTimeMillis()
)
