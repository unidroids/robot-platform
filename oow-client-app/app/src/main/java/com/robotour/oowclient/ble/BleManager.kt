package com.robotour.oowclient.ble

import android.annotation.SuppressLint
import android.bluetooth.*
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.os.Handler
import android.os.Looper
import android.os.ParcelUuid
import android.util.Log
import com.robotour.oowclient.model.TelemetryData
import com.robotour.oowclient.util.Constants

class BleManager(
    private val context: Context,
    private val callback: BleCallback
) {

    interface BleCallback {
        fun onStateChanged(state: BleConnectionState, deviceName: String?, errorInfo: String? = null)
        fun onTelemetryReceived(telemetry: TelemetryData)
        fun onDeviceDiscovered(device: BluetoothDevice, rssi: Int)
        fun onScanFinished(devicesFoundCount: Int)
        fun onGattRetry(status: Int, retryCount: Int)
    }

    private val bluetoothAdapter: BluetoothAdapter? by lazy {
        (context.getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager)?.adapter
    }

    private var bluetoothGatt: BluetoothGatt? = null
    private var lastDevice: BluetoothDevice? = null

    private var commandCharacteristic: BluetoothGattCharacteristic? = null
    private var heartbeatCharacteristic: BluetoothGattCharacteristic? = null
    private var telemetryCharacteristic: BluetoothGattCharacteristic? = null

    private val handler = Handler(Looper.getMainLooper())
    private var isDiscoveringServices = false
    private var isReadingTelemetry = false
    private var isHeartbeatActive = false
    private var clientIdProvider: (() -> String)? = null
    private var retryCount = 0
    private var discoveryAttempts = 0
    private var isScanning = false
    private val discoveredDevices = mutableSetOf<String>()

    val isBluetoothEnabled: Boolean
        get() = bluetoothAdapter?.isEnabled == true

    private val bluetoothStateReceiver = object : BroadcastReceiver() {
        override fun onReceive(c: Context?, intent: Intent?) {
            if (intent?.action == BluetoothAdapter.ACTION_STATE_CHANGED) {
                val state = intent.getIntExtra(BluetoothAdapter.EXTRA_STATE, BluetoothAdapter.ERROR)
                if (state == BluetoothAdapter.STATE_OFF || state == BluetoothAdapter.STATE_TURNING_OFF) {
                    resetState()
                    callback.onStateChanged(BleConnectionState.BLUETOOTH_OFF, null)
                }
            }
        }
    }

    fun registerStateReceiver() {
        try {
            context.registerReceiver(
                bluetoothStateReceiver,
                IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED),
                Context.RECEIVER_EXPORTED
            )
        } catch (e: Exception) {
            Log.e("BleManager", "Chyba pri registraci bluetoothStateReceiver: ${e.message}")
        }
    }

    fun unregisterStateReceiver() {
        try {
            context.unregisterReceiver(bluetoothStateReceiver)
        } catch (_: Exception) { }
    }

    private val discoverServicesRunnable = Runnable {
        val gatt = bluetoothGatt
        if (gatt != null && isDiscoveringServices) {
            discoveryAttempts++
            Log.d("BleManager", "Spoustim gatt.discoverServices() (pokus $discoveryAttempts)")
            val started = gatt.discoverServices()
            if (!started) {
                Log.e("BleManager", "discoverServices() selhalo ihned!")
                disconnect()
            }
        }
    }

    private val discoverTimeoutRunnable = Runnable {
        if (isDiscoveringServices && bluetoothGatt != null) {
            Log.e("BleManager", "Timeout hledání služeb (10s)!")
            disconnect()
        }
    }

    private val retryRunnable = Runnable {
        lastDevice?.let { connect(it) }
    }

    private val heartbeatRunnable = object : Runnable {
        @SuppressLint("MissingPermission")
        override fun run() {
            if (!isHeartbeatActive) return
            val gatt = bluetoothGatt
            val char = heartbeatCharacteristic
            if (gatt != null && char != null && !isReadingTelemetry) {
                val clientId = clientIdProvider?.invoke() ?: "Unknown"
                val payload = clientId.toByteArray()
                gatt.writeCharacteristic(char, payload, BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE)
            }
            handler.postDelayed(this, Constants.HEARTBEAT_INTERVAL_MS)
        }
    }

    private val scanCallback = object : ScanCallback() {
        @SuppressLint("MissingPermission")
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val device = result.device
            if (device.address !in discoveredDevices) {
                discoveredDevices.add(device.address)
                handler.post { callback.onDeviceDiscovered(device, result.rssi) }
            }
        }
    }

    private val scanTimeoutRunnable = Runnable { stopScan() }
    private val disconnectTimeoutRunnable = Runnable {
        Log.w("BleManager", "Disconnect timeout - vynucuji close a resetState")
        bluetoothGatt?.close()
        bluetoothGatt = null
        resetState()
    }

    @SuppressLint("MissingPermission")
    fun startScan() {
        if (!isBluetoothEnabled) {
            handler.post { callback.onStateChanged(BleConnectionState.BLUETOOTH_OFF, null) }
            return
        }

        discoveredDevices.clear()
        isScanning = true
        val scanner = bluetoothAdapter?.bluetoothLeScanner
        Log.d("BleManager", "Spoustim filtrovane skenovani pro UUID: ${Constants.SERVICE_UUID}")

        val filter = ScanFilter.Builder().setServiceUuid(ParcelUuid(Constants.SERVICE_UUID)).build()
        val settings = ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build()

        scanner?.startScan(listOf(filter), settings, scanCallback)
        handler.removeCallbacks(scanTimeoutRunnable)
        handler.postDelayed(scanTimeoutRunnable, Constants.SCAN_DURATION_MS)
    }

    @SuppressLint("MissingPermission")
    fun stopScan() {
        if (!isScanning) return
        isScanning = false
        handler.removeCallbacks(scanTimeoutRunnable)
        bluetoothAdapter?.bluetoothLeScanner?.stopScan(scanCallback)
        handler.post { callback.onScanFinished(discoveredDevices.size) }
    }

    @SuppressLint("MissingPermission")
    fun connect(device: BluetoothDevice) {
        lastDevice = device
        cancelAllPendingTasks()
        isDiscoveringServices = false
        discoveryAttempts = 0
        bluetoothGatt?.close()
        bluetoothGatt = null

        val deviceName = device.name ?: device.address
        Log.d("BleManager", "Pripojuji k: ${device.address} ($deviceName)")
        handler.post { callback.onStateChanged(BleConnectionState.CONNECTING, deviceName) }

        bluetoothGatt = device.connectGatt(
            context,
            false,
            gattCallback,
            BluetoothDevice.TRANSPORT_LE,
            BluetoothDevice.PHY_LE_1M_MASK
        )
    }

    @SuppressLint("MissingPermission")
    fun disconnect() {
        Log.d("BleManager", "disconnect() vyvolano")
        cancelAllPendingTasks()
        val gatt = bluetoothGatt
        val deviceName = lastDevice?.name
        handler.post { callback.onStateChanged(BleConnectionState.DISCONNECTING, deviceName) }
        if (gatt != null) {
            gatt.disconnect()
            handler.postDelayed(disconnectTimeoutRunnable, 1500)
        } else {
            resetState()
        }
    }

    private fun cancelAllPendingTasks() {
        handler.removeCallbacks(discoverServicesRunnable)
        handler.removeCallbacks(discoverTimeoutRunnable)
        handler.removeCallbacks(retryRunnable)
        handler.removeCallbacks(disconnectTimeoutRunnable)
    }

    private fun resetState() {
        stopHeartbeat()
        cancelAllPendingTasks()
        isDiscoveringServices = false
        isReadingTelemetry = false
        retryCount = 0
        discoveryAttempts = 0
        commandCharacteristic = null
        heartbeatCharacteristic = null
        telemetryCharacteristic = null
        handler.post {
            callback.onStateChanged(BleConnectionState.DISCONNECTED, null)
        }
    }

    private val gattCallback = object : BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            Log.d("BleManager", "onConnectionStateChange: status=$status, newState=$newState")

            if (status != BluetoothGatt.GATT_SUCCESS) {
                isDiscoveringServices = false
                cancelAllPendingTasks()
                if ((status == 133 || status == 19 || status == 62) && retryCount < 3 && lastDevice != null) {
                    retryCount++
                    handler.post { callback.onGattRetry(status, retryCount) }
                    gatt.close()
                    if (bluetoothGatt == gatt) bluetoothGatt = null
                    handler.postDelayed(retryRunnable, Constants.RETRY_DELAY_MS)
                } else {
                    handler.post { callback.onStateChanged(BleConnectionState.ERROR, gatt.device.name, "Status: $status") }
                    gatt.close()
                    if (bluetoothGatt == gatt) bluetoothGatt = null
                    resetState()
                }
                return
            }

            if (newState == BluetoothProfile.STATE_CONNECTED) {
                isDiscoveringServices = true
                discoveryAttempts = 0
                retryCount = 0
                val deviceName = gatt.device.name ?: gatt.device.address
                handler.post { callback.onStateChanged(BleConnectionState.SEARCHING_SERVICES, deviceName) }

                cancelAllPendingTasks()
                handler.postDelayed(discoverServicesRunnable, Constants.DISCOVERY_STABILIZE_DELAY_MS)
                handler.postDelayed(discoverTimeoutRunnable, Constants.DISCOVERY_TIMEOUT_MS)

            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                val wasDiscovering = isDiscoveringServices
                isDiscoveringServices = false
                cancelAllPendingTasks()
                gatt.close()
                if (bluetoothGatt == gatt) bluetoothGatt = null
                if (wasDiscovering && retryCount < 3 && lastDevice != null) {
                    retryCount++
                    handler.postDelayed(retryRunnable, Constants.DISCONNECT_RETRY_DELAY_MS)
                } else {
                    resetState()
                }
            }
        }

        @SuppressLint("MissingPermission")
        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            handler.removeCallbacks(discoverTimeoutRunnable)
            if (!isDiscoveringServices) return

            isDiscoveringServices = false
            discoveryAttempts = 0
            if (status == BluetoothGatt.GATT_SUCCESS) {
                Log.d("BleManager", "Sluzby nalezeny.")
                val service = gatt.getService(Constants.SERVICE_UUID)
                if (service != null) {
                    commandCharacteristic = service.getCharacteristic(Constants.CHAR_COMMAND_UUID)
                    heartbeatCharacteristic = service.getCharacteristic(Constants.CHAR_HEARTBEAT_UUID)
                    telemetryCharacteristic = service.getCharacteristic(Constants.CHAR_TELEMETRY_UUID)
                    enableTelemetryNotifications(gatt, telemetryCharacteristic)

                    val deviceName = gatt.device.name ?: gatt.device.address
                    handler.post { callback.onStateChanged(BleConnectionState.READY, deviceName) }
                } else {
                    handler.post { callback.onStateChanged(BleConnectionState.ERROR, gatt.device.name, "Chybí požadovaná BLE služba") }
                    gatt.disconnect()
                }
            } else {
                Log.e("BleManager", "onServicesDiscovered ERROR: $status")
                handler.post { callback.onStateChanged(BleConnectionState.ERROR, gatt.device.name, "Chyba vyhledání služeb: $status") }
                gatt.disconnect()
            }
        }

        override fun onCharacteristicRead(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic, value: ByteArray, status: Int) {
            isReadingTelemetry = false
            if (status == BluetoothGatt.GATT_SUCCESS && characteristic.uuid == Constants.CHAR_TELEMETRY_UUID) {
                val data = String(value, Charsets.UTF_8)
                val telemetry = TelemetryData.fromJson(data)
                handler.post { callback.onTelemetryReceived(telemetry) }
            }
        }

        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic, value: ByteArray) {
            if (characteristic.uuid == Constants.CHAR_TELEMETRY_UUID) {
                val data = String(value, Charsets.UTF_8)
                val telemetry = TelemetryData.fromJson(data)
                handler.post { callback.onTelemetryReceived(telemetry) }
            }
        }

        override fun onCharacteristicWrite(gatt: BluetoothGatt?, characteristic: BluetoothGattCharacteristic?, status: Int) {
            Log.d("BleManager", "onCharacteristicWrite status=$status, uuid=${characteristic?.uuid}")
        }
    }

    @SuppressLint("MissingPermission")
    private fun enableTelemetryNotifications(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic?) {
        if (characteristic != null && (characteristic.properties and BluetoothGattCharacteristic.PROPERTY_NOTIFY != 0)) {
            gatt.setCharacteristicNotification(characteristic, true)
            val desc = characteristic.getDescriptor(Constants.CCCD_UUID)
            if (desc != null) {
                gatt.writeDescriptor(desc, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE)
            }
        }
    }

    @SuppressLint("MissingPermission")
    fun sendCommand(clientId: String, command: String) {
        val gatt = bluetoothGatt
        val char = commandCharacteristic
        if (gatt != null && char != null) {
            val payload = "$clientId:$command".toByteArray()
            gatt.writeCharacteristic(char, payload, BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE)
        }
    }

    fun startHeartbeat(provider: () -> String) {
        if (isHeartbeatActive) return
        Log.d("BleManager", "Start Heartbeat (333ms)")
        clientIdProvider = provider
        isHeartbeatActive = true
        handler.post(heartbeatRunnable)
    }

    fun stopHeartbeat() {
        Log.d("BleManager", "Stop Heartbeat")
        isHeartbeatActive = false
        handler.removeCallbacks(heartbeatRunnable)
    }

    @SuppressLint("MissingPermission")
    fun readTelemetry() {
        val gatt = bluetoothGatt
        val char = telemetryCharacteristic
        if (gatt != null && char != null && !isReadingTelemetry) {
            isReadingTelemetry = true
            if (!gatt.readCharacteristic(char)) {
                isReadingTelemetry = false
            }
        }
    }

    fun cleanup() {
        unregisterStateReceiver()
        stopHeartbeat()
        cancelAllPendingTasks()
        bluetoothGatt?.close()
        bluetoothGatt = null
    }
}
