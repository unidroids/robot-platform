package com.unidroids.robot_hmi.viewmodel

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.unidroids.robot_hmi.network.NetworkEngine
import com.unidroids.robot_hmi.network.NetworkState
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch

class MainViewModel(application: Application) : AndroidViewModel(application) {
    private val networkEngine = NetworkEngine(application)
    val networkState: StateFlow<NetworkState> = networkEngine.state

    init {
        viewModelScope.launch {
            networkEngine.start()
        }
    }

    fun onScannerHandled() {
        networkEngine.onScannerHandled()
    }

    fun onMessageButtonClicked(buttonId: String) {
        networkEngine.publishTerminalButton(buttonId)
    }

    fun onQrCodeScanned(data: String) {
        networkEngine.publishQrData(data)
    }

    override fun onCleared() {
        super.onCleared()
        networkEngine.stop()
    }
}
