package com.unidroids.robot_hmi.network

import org.zeromq.SocketType
import org.zeromq.ZContext
import org.zeromq.ZMQ
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext

class QrZmqPublisher(
    private val address: String = "tcp://127.0.0.1:8001"
) {
    private var context: ZContext? = null
    private var publisher: ZMQ.Socket? = null

    suspend fun start() = withContext(Dispatchers.IO) {
        context = ZContext()
        publisher = context!!.createSocket(SocketType.PUB)
        publisher!!.connect(address)
    }

    fun sendQrMessage(topic: String, prefix: String, data: String) {
        publisher?.let {
            it.sendMore(topic)
            it.sendMore(prefix)
            it.send(data)
        }
    }

    fun stop() {
        context?.close()
        context = null
        publisher = null
    }
}
