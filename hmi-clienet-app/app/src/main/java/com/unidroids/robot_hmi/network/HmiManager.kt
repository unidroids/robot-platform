package com.unidroids.robot_hmi.network

import io.ktor.network.selector.SelectorManager
import io.ktor.network.sockets.aSocket
import io.ktor.network.sockets.openReadChannel
import io.ktor.network.sockets.openWriteChannel
import io.ktor.utils.io.readLine
import io.ktor.utils.io.writeStringUtf8
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.withContext
import java.util.concurrent.atomic.AtomicBoolean

class HmiManager(
    private val host: String = "127.0.0.1",
    private val port: Int = 9000,
    private val onConnectionStatusChanged: (Boolean) -> Unit
) {
    private val isRunning = AtomicBoolean(false)

    suspend fun start() = withContext(Dispatchers.IO) {
        isRunning.set(true)
        val selectorManager = SelectorManager(Dispatchers.IO)
        
        while (isRunning.get() && isActive) {
            try {
                aSocket(selectorManager).tcp().connect(host, port).use { socket ->
                    onConnectionStatusChanged(true)
                    val receiveChannel = socket.openReadChannel()
                    val sendChannel = socket.openWriteChannel(autoFlush = true)

                    sendChannel.writeStringUtf8("PING\n")
                    val response = receiveChannel.readLine()
                    // response should be PONG or similar
                    
                    while (isRunning.get() && isActive) {
                        // Keep connection alive, maybe wait for incoming data
                        val line = receiveChannel.readLine() ?: break
                    }
                }
            } catch (e: Exception) {
                onConnectionStatusChanged(false)
            }
            if (isRunning.get()) {
                delay(2000) // Reconnect every 2s on failure
            }
        }
        selectorManager.close()
    }

    fun stop() {
        isRunning.set(false)
    }
}
