package com.unidroids.robot_hmi.network

import android.content.Context
import java.io.File
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch

class NetworkEngine(private val context: Context) {
    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.IO)
    
    private val _state = MutableStateFlow(NetworkState())
    val state: StateFlow<NetworkState> = _state.asStateFlow()

    private val hmiManager = HmiManager(
        onConnectionStatusChanged = { connected ->
            _state.update { it.copy(isHmiConnected = connected) }
        }
    )

    private val qrCommandServer = QrCommandServer(
        onCommandReceived = { command ->
            _state.update { it.copy(lastCommand = command) }
            handleCommand(command)
        },
        onStatusChanged = { running ->
            _state.update { it.copy(isQrServerRunning = running) }
        },
        onClientStatusChanged = { connected ->
            _state.update { it.copy(isClientConnected = connected) }
        }
    )
    
    private val terminalZmqPublisher = TerminalZmqPublisher()

    private val terminalCommandServer = TerminalCommandServer(
        onCommandReceived = { command ->
            _state.update { it.copy(lastCommand = command) }
            handleTerminalCommand(command)
        },
        onStatusChanged = { running ->
            _state.update { it.copy(isTerminalServerRunning = running) }
        },
        onClientStatusChanged = { connected ->
            _state.update { it.copy(isTerminalClientConnected = connected) }
        }
    )

    private val qrZmqPublisher = QrZmqPublisher()

    fun start() {
        scope.launch { hmiManager.start() }
        scope.launch { qrCommandServer.start() }
        scope.launch { terminalCommandServer.start() }
        scope.launch { terminalZmqPublisher.start() }
        scope.launch { qrZmqPublisher.start() }
    }

    private fun handleCommand(command: String): String {
        val cmd = command.trim().uppercase()
        return when (cmd) {
            "PING" -> "PONG QRSCANER"
            "START" -> {
                _state.update { it.copy(scannerTriggered = true) }
                "OK STARTED"
            }
            "STOP" -> {
                _state.update { it.copy(scannerTriggered = false) }
                "OK STOPPED"
            }
            "STATUS" -> {
                val state = _state.value
                if (state.scannerTriggered) {
                    "RUNNING ${state.lastQrCode ?: ""}".trim()
                } else {
                    "WAITING"
                }
            }
            "QRCODE" -> {
                val last = _state.value.lastQrCode ?: ""
                _state.update { it.copy(lastQrCode = null) }
                last
            }
            else -> "ERROR: Unknown command $cmd"
        }
    }

    private fun handleTerminalCommand(command: String): String {
        val parts = command.trim().split(" ")
        if (parts.isEmpty()) return "ERROR: Empty command"
        
        val cmd = parts[0].uppercase()
        return when (cmd) {
            "PING" -> "PONG TERMINAL"
            "BLINK" -> {
                if (parts.size >= 4) {
                    val color = parts[1]
                    val frequency = parts[2].toFloatOrNull() ?: 1.0f
                    val duration = parts[3].toLongOrNull() ?: 1000L
                    _state.update { 
                        it.copy(blinkEvent = BlinkConfig(color, frequency, duration)) 
                    }
                    "OK BLINK"
                } else {
                    "ERROR: Invalid BLINK format. Use: BLINK <color> <frequency> <duration>"
                }
            }
            "SOUND" -> {
                if (parts.size >= 2) {
                    val name = parts[1]
                    val soundsDir = File(context.getExternalFilesDir(null), "Sounds")
                    if (!soundsDir.exists()) {
                        soundsDir.mkdirs()
                    }
                    
                    val file = File(soundsDir, "$name.wav")
                    if (file.exists()) {
                        _state.update {
                            it.copy(soundEvent = SoundConfig(name))
                        }
                        "OK SOUND"
                    } else {
                        "ERROR: file $name not found"
                    }
                } else {
                    "ERROR: Invalid SOUND format. Use: SOUND <name>"
                }
            }
            "MESSAGE" -> {
                if (parts.size >= 2) {
                    val jsonString = command.substringAfter("MESSAGE").trim()
                    try {
                        val jsonObject = org.json.JSONObject(jsonString)
                        val header = jsonObject.optString("header", "")
                        val text = jsonObject.optString("text", "")
                        
                        val buttonsList = mutableListOf<MessageButton>()
                        val buttonsArray = jsonObject.optJSONArray("buttons")
                        if (buttonsArray != null) {
                            for (i in 0 until buttonsArray.length()) {
                                val btnObj = buttonsArray.optJSONObject(i)
                                if (btnObj != null) {
                                    val id = btnObj.optString("id", "")
                                    val btnText = btnObj.optString("text", "")
                                    if (id.isNotEmpty() && btnText.isNotEmpty()) {
                                        buttonsList.add(MessageButton(id, btnText))
                                    }
                                }
                            }
                        }
                        
                        _state.update {
                            it.copy(messageEvent = MessageConfig(header, text, buttonsList))
                        }
                        "OK MESSAGE"
                    } catch (e: Exception) {
                        "ERROR: Invalid JSON"
                    }
                } else {
                    "ERROR: Empty MESSAGE"
                }
            }
            else -> "ERROR: Unknown command $cmd"
        }
    }

    fun publishTerminalButton(buttonId: String) {
        scope.launch {
            terminalZmqPublisher.sendButtonMessage(buttonId)
        }
    }

    fun onScannerHandled() {
        _state.update { it.copy(scannerTriggered = false) }
    }

    fun publishQrData(data: String) {
        _state.update { it.copy(lastQrCode = data) }
        var foundPrefix = "text:"
        val colonIndex = data.indexOf(':')
        
        if (colonIndex > 0) {
            val potentialPrefix = data.substring(0, colonIndex + 1)
            // valid URI prefix shouldn't contain spaces
            if (!potentialPrefix.contains(" ")) {
                foundPrefix = potentialPrefix.lowercase()
            }
        }

        val payload = data

        scope.launch {
            qrZmqPublisher.sendQrMessage("robot-qrscaner", foundPrefix, payload)
        }
    }

    fun stop() {
        hmiManager.stop()
        qrCommandServer.stop()
        terminalCommandServer.stop()
        terminalZmqPublisher.stop()
        qrZmqPublisher.stop()
    }
}
