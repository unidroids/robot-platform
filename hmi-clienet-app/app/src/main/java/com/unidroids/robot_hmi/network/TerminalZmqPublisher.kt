package com.unidroids.robot_hmi.network

import org.zeromq.SocketType
import org.zeromq.ZContext
import org.zeromq.ZMQ
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext

class TerminalZmqPublisher(
    private val address: String = "tcp://127.0.0.1:8002"
) {
    private var context: ZContext? = null
    private var publisher: ZMQ.Socket? = null

    suspend fun start() = withContext(Dispatchers.IO) {
        context = ZContext()
        publisher = context!!.createSocket(SocketType.PUB)
        publisher!!.connect(address)
    }

    fun sendButtonMessage(buttonId: String) {
        publisher?.let {
            it.sendMore("robot-terminal")
            it.sendMore("button")
            it.send(buttonId)
        }
    }

    fun stop() {
        context?.close()
        context = null
        publisher = null
    }
}
