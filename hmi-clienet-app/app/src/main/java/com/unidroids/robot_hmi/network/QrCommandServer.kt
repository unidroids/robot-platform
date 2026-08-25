package com.unidroids.robot_hmi.network

import io.ktor.network.selector.SelectorManager
import io.ktor.network.sockets.ServerSocket
import io.ktor.network.sockets.aSocket
import io.ktor.network.sockets.openReadChannel
import io.ktor.network.sockets.openWriteChannel
import io.ktor.utils.io.readLine
import io.ktor.utils.io.writeStringUtf8
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger

class QrCommandServer(
    private val port: Int = 9001,
    private val onCommandReceived: (String) -> String,
    private val onStatusChanged: (Boolean) -> Unit,
    private val onClientStatusChanged: (Boolean) -> Unit
) {
    private val isRunning = AtomicBoolean(false)
    private var serverSocket: ServerSocket? = null
    private val connectionCount = AtomicInteger(0)

    suspend fun start() = withContext(Dispatchers.IO) {
        isRunning.set(true)
        val selectorManager = SelectorManager(Dispatchers.IO)
        try {
            val socket = aSocket(selectorManager).tcp().bind(port = port)
            serverSocket = socket
            onStatusChanged(true)
            
            while (isRunning.get() && isActive) {
                val clientSocket = try {
                    socket.accept()
                } catch (e: Exception) {
                    if (isRunning.get()) throw e else break
                }
                
                launch {
                    val count = connectionCount.incrementAndGet()
                    if (count == 1) onClientStatusChanged(true)
                    
                    try {
                        val receiveChannel = clientSocket.openReadChannel()
                        val sendChannel = clientSocket.openWriteChannel(autoFlush = true)
                        while (isActive && isRunning.get()) {
                            val line = receiveChannel.readLine() ?: break
                            val response = onCommandReceived(line)
                            sendChannel.writeStringUtf8(response + "\n")
                        }
                    } catch (e: Exception) {
                        // Client disconnected or error
                    } finally {
                        clientSocket.close()
                        val remaining = connectionCount.decrementAndGet()
                        if (remaining == 0) onClientStatusChanged(false)
                    }
                }
            }
        } catch (e: Exception) {
            // Server stopped or error
        } finally {
            serverSocket?.close()
            serverSocket = null
            selectorManager.close()
            onStatusChanged(false)
            onClientStatusChanged(false)
            connectionCount.set(0)
        }
    }

    fun stop() {
        isRunning.set(false)
        serverSocket?.close()
    }
}
