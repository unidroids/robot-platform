package com.robotour.oowclient

import android.Manifest
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
import android.content.SharedPreferences
import android.content.pm.PackageManager
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.ParcelUuid
import android.text.Editable
import android.text.TextWatcher
import android.util.Log
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.EditText
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.SwitchCompat
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.core.content.edit
import org.json.JSONObject
import java.util.UUID

class MainActivity : AppCompatActivity() {

    private lateinit var tvStatus: TextView
    private lateinit var tvTelemetry: TextView
    private lateinit var btnConnect: Button
    private lateinit var etClientName: EditText
    private lateinit var switchWatch: SwitchCompat
    private lateinit var switchCamera: SwitchCompat
    private lateinit var switchLidar: SwitchCompat
    private lateinit var switchRtk: SwitchCompat
    private lateinit var switchFusion: SwitchCompat
    private lateinit var switchDrive: SwitchCompat
    private lateinit var btnPause: Button
    private lateinit var btnResume: Button
    private lateinit var btnStop: Button
    private lateinit var btnPowerOff: Button
    private lateinit var btnCameraStatus: Button
    private lateinit var btnLidarStatus: Button
    private lateinit var btnRtkStatus: Button
    private lateinit var btnFusionStatus: Button
    private lateinit var btnDriveStatus: Button
    private lateinit var btnWaypointsStart: Button
    private lateinit var btnWaypointsStatus: Button
    private lateinit var btnTelemetry: Button

    private var bluetoothAdapter: BluetoothAdapter? = null
    private var bluetoothGatt: BluetoothGatt? = null

    private var commandCharacteristic: BluetoothGattCharacteristic? = null
    private var heartbeatCharacteristic: BluetoothGattCharacteristic? = null
    private var telemetryCharacteristic: BluetoothGattCharacteristic? = null

    private val SERVICE_UUID: UUID = UUID.fromString("87654321-4321-4321-4321-abcdef987654")
    private val CHAR_COMMAND_UUID: UUID = UUID.fromString("87654321-4321-4321-4321-abcdef987655")
    private val CHAR_HEARTBEAT_UUID: UUID = UUID.fromString("87654321-4321-4321-4321-abcdef987656")
    private val CHAR_TELEMETRY_UUID: UUID = UUID.fromString("87654321-4321-4321-4321-abcdef987657")

    private val PERMISSION_REQUEST_CODE = 100
    private lateinit var sharedPref: SharedPreferences
    private val handler = Handler(Looper.getMainLooper())
    private var isHeartbeatActive = false

    private val bluetoothStateReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            if (intent?.action == BluetoothAdapter.ACTION_STATE_CHANGED) {
                val state = intent.getIntExtra(BluetoothAdapter.EXTRA_STATE, BluetoothAdapter.ERROR)
                if (state == BluetoothAdapter.STATE_OFF || state == BluetoothAdapter.STATE_TURNING_OFF) {
                    runOnUiThread {
                        tvStatus.text = getString(R.string.status_bluetooth_off)
                        resetBluetoothState()
                    }
                }
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        tvStatus = findViewById(R.id.tvStatus)
        tvTelemetry = findViewById(R.id.tvTelemetry)
        btnConnect = findViewById(R.id.btnConnect)
        etClientName = findViewById(R.id.etClientName)
        switchWatch = findViewById(R.id.switchWatch)
        switchCamera = findViewById(R.id.switchCamera)
        switchLidar = findViewById(R.id.switchLidar)
        switchRtk = findViewById(R.id.switchRtk)
        switchFusion = findViewById(R.id.switchFusion)
        switchDrive = findViewById(R.id.switchDrive)
        btnPause = findViewById(R.id.btnPause)
        btnResume = findViewById(R.id.btnResume)
        btnStop = findViewById(R.id.btnStop)
        btnPowerOff = findViewById(R.id.btnPowerOff)
        btnCameraStatus = findViewById(R.id.btnCameraStatus)
        btnLidarStatus = findViewById(R.id.btnLidarStatus)
        btnRtkStatus = findViewById(R.id.btnRtkStatus)
        btnFusionStatus = findViewById(R.id.btnFusionStatus)
        btnDriveStatus = findViewById(R.id.btnDriveStatus)
        btnWaypointsStart = findViewById(R.id.btnWaypointsStart)
        btnWaypointsStatus = findViewById(R.id.btnWaypointsStatus)
        btnTelemetry = findViewById(R.id.btnTelemetry)

        sharedPref = getSharedPreferences("OowClientPrefs", Context.MODE_PRIVATE)
        etClientName.setText(sharedPref.getString("client_name", getString(R.string.default_operator_name)))

        etClientName.addTextChangedListener(object : TextWatcher {
            override fun beforeTextChanged(s: CharSequence?, start: Int, count: Int, after: Int) {}
            override fun onTextChanged(s: CharSequence?, start: Int, before: Int, count: Int) {}
            override fun afterTextChanged(s: Editable?) {
                sharedPref.edit { putString("client_name", s.toString()) }
            }
        })

        bluetoothAdapter = (getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager).adapter
        setupConnectButton()
        registerReceiver(bluetoothStateReceiver, IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED), RECEIVER_EXPORTED)

        switchWatch.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked) startHeartbeat() else stopHeartbeat()
        }

        switchCamera.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked) sendCommand("CAMERA_ON") else sendCommand("CAMERA_OFF")
        }

        switchLidar.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked) sendCommand("LIDAR_ON") else sendCommand("LIDAR_OFF")
        }

        switchRtk.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked) sendCommand("RTK_ON") else sendCommand("RTK_OFF")
        }

        switchFusion.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked) sendCommand("FUSION_ON") else sendCommand("FUSION_OFF")
        }

        switchDrive.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked) sendCommand("DRIVE_ON") else sendCommand("DRIVE_OFF")
        }

        btnPause.setOnClickListener { sendCommand("PAUSE") }
        btnResume.setOnClickListener { sendCommand("RESUME") }
        btnStop.setOnClickListener { sendCommand("STOP") }
        btnPowerOff.setOnClickListener { 
            AlertDialog.Builder(this)
                .setTitle(R.string.dialog_power_off_title)
                .setMessage(R.string.dialog_power_off_message)
                .setPositiveButton(R.string.yes) { _, _ -> sendCommand("POWEROFF") }
                .setNegativeButton(R.string.no, null)
                .show()
        }
        
        btnCameraStatus.setOnClickListener { sendCommand("CAMERA_STATUS") }
        btnLidarStatus.setOnClickListener { sendCommand("LIDAR_STATUS") }
        btnRtkStatus.setOnClickListener { sendCommand("RTK_STATUS") }
        btnFusionStatus.setOnClickListener { sendCommand("FUSION_STATUS") }
        btnDriveStatus.setOnClickListener { sendCommand("DRIVE_STATUS") }
        btnWaypointsStart.setOnClickListener { sendCommand("PILOT_WAYPOINTS_START") }
        btnWaypointsStatus.setOnClickListener { sendCommand("PILOT_WAYPOINTS_STATUS") }
        btnTelemetry.setOnClickListener { readTelemetry() }
    }

    private fun getClientId(): String = etClientName.text.toString().trim().ifEmpty { "Unknown" }

    private fun checkBluetoothPermissions(): Boolean {
        val permissions = arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
        return permissions.all { ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED }
    }

    private fun requestBluetoothPermissions() {
        val permissions = arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
        ActivityCompat.requestPermissions(this, permissions, PERMISSION_REQUEST_CODE)
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == PERMISSION_REQUEST_CODE && grantResults.isNotEmpty() && grantResults.all { it == PackageManager.PERMISSION_GRANTED }) {
            startDeviceScan()
        }
    }

    private fun setupConnectButton() {
        runOnUiThread {
            btnConnect.setText(R.string.btn_search_robot)
            btnConnect.isEnabled = true
            btnConnect.setOnClickListener {
                if (checkBluetoothPermissions()) startDeviceScan() else requestBluetoothPermissions()
            }
        }
    }

    private fun setupDisconnectButton() {
        runOnUiThread {
            btnConnect.setText(R.string.btn_disconnect)
            btnConnect.isEnabled = true
            btnConnect.setOnClickListener { disconnectGatt() }
        }
    }

    @SuppressLint("MissingPermission")
    private fun disconnectGatt() {
        Log.d("OowBLE", "disconnectGatt() vyvoláno")
        tvStatus.text = getString(R.string.status_disconnecting)
        val gatt = bluetoothGatt
        bluetoothGatt = null
        gatt?.disconnect()
        gatt?.close()
        resetBluetoothState()
    }

    private fun resetBluetoothState() {
        stopHeartbeat()
        isDiscoveringServices = false
        isReadingTelemetry = false
        retryCount = 0
        runOnUiThread {
            tvStatus.text = getString(R.string.status_disconnected)
            switchWatch.isChecked = false
            btnConnect.isEnabled = true
            commandCharacteristic = null
            heartbeatCharacteristic = null
            telemetryCharacteristic = null
            setupConnectButton()
        }
    }

    private val discoveredDevices = mutableListOf<BluetoothDevice>()
    private var scanDialog: AlertDialog? = null
    private var deviceListAdapter: ArrayAdapter<String>? = null

    @SuppressLint("MissingPermission")
    private fun startDeviceScan() {
        if (bluetoothAdapter?.isEnabled != true) {
            Toast.makeText(this, R.string.toast_enable_bluetooth, Toast.LENGTH_SHORT).show()
            return
        }

        discoveredDevices.clear()
        val deviceNames = mutableListOf<String>()
        deviceListAdapter = ArrayAdapter(this, android.R.layout.simple_list_item_1, deviceNames)

        scanDialog = AlertDialog.Builder(this)
            .setTitle(R.string.scan_dialog_title)
            .setAdapter(deviceListAdapter) { _, which ->
                stopScan()
                connectToGatt(discoveredDevices[which])
            }
            .setNegativeButton(R.string.btn_cancel) { _, _ -> stopScan() }
            .setOnDismissListener { stopScan() }
            .show()

        val scanner = bluetoothAdapter?.bluetoothLeScanner
        Log.d("OowBLE", "Spouštím filtrované skenování pro UUID: $SERVICE_UUID")
        
        val filter = ScanFilter.Builder().setServiceUuid(ParcelUuid(SERVICE_UUID)).build()
        val settings = ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build()

        scanner?.startScan(listOf(filter), settings, scanCallback)
        handler.postDelayed({ stopScan() }, 10000)
    }

    @SuppressLint("MissingPermission")
    private fun stopScan() {
        bluetoothAdapter?.bluetoothLeScanner?.stopScan(scanCallback)
        if (scanDialog?.isShowing == true && discoveredDevices.isEmpty()) {
            scanDialog?.setTitle(R.string.scan_no_robot_found)
        }
    }

    private val scanCallback = object : ScanCallback() {
        @SuppressLint("MissingPermission")
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val device = result.device
            if (device !in discoveredDevices) {
                discoveredDevices.add(device)
                val name = device.name ?: getString(R.string.unknown_device)
                handler.post {
                    deviceListAdapter?.add(getString(R.string.scan_device_info, name, device.address, result.rssi))
                    deviceListAdapter?.notifyDataSetChanged()
                }
            }
        }
    }

    private var lastDevice: BluetoothDevice? = null
    private var isDiscoveringServices = false
    private var retryCount = 0

    @SuppressLint("MissingPermission")
    private fun connectToGatt(device: BluetoothDevice) {
        lastDevice = device
        isDiscoveringServices = false
        bluetoothGatt?.close()
        bluetoothGatt = null
        
        Log.d("OowBLE", "Připojuji k: ${device.address}")
        tvStatus.text = getString(R.string.status_connecting, device.name ?: getString(R.string.unknown_device))
        btnConnect.isEnabled = false

        @Suppress("DEPRECATION")
        bluetoothGatt = device.connectGatt(this, false, gattCallback, BluetoothDevice.TRANSPORT_LE, BluetoothDevice.PHY_LE_1M_MASK, handler)
    }

    private val gattCallback = object : BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            Log.d("OowBLE", "onConnectionStateChange: status=$status, newState=$newState")
            
            if (status != BluetoothGatt.GATT_SUCCESS) {
                isDiscoveringServices = false
                if ((status == 133 || status == 19 || status == 62) && retryCount < 3 && lastDevice != null) {
                    retryCount++
                    handler.post { tvStatus.text = getString(R.string.status_error_gatt_retry, status, retryCount) }
                    if (status == 133) refreshDeviceCache(gatt)
                    gatt.close()
                    handler.postDelayed({ lastDevice?.let { connectToGatt(it) } }, 1000)
                } else {
                    handler.post { tvStatus.text = getString(R.string.status_error_connection, status) }
                    gatt.close()
                    resetBluetoothState()
                }
                return
            }

            if (newState == BluetoothProfile.STATE_CONNECTED) {
                isDiscoveringServices = true
                retryCount = 0
                val deviceName = gatt.device.name ?: getString(R.string.unknown_device)
                handler.post { 
                    tvStatus.text = getString(R.string.status_searching_services, deviceName)
                    setupDisconnectButton()
                }
                
                handler.postDelayed({
                    Log.d("OowBLE", "Spouštím discoverServices()")
                    if (bluetoothGatt == null || !gatt.discoverServices()) {
                        Log.e("OowBLE", "discoverServices() selhalo!")
                        gatt.disconnect()
                    }
                }, 250)

                handler.postDelayed({
                    if (isDiscoveringServices && bluetoothGatt != null) {
                        Log.e("OowBLE", "Timeout hledání služeb (20s)!")
                        gatt.disconnect()
                    }
                }, 20000)

            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                val wasDiscovering = isDiscoveringServices
                isDiscoveringServices = false
                gatt.close()
                if (wasDiscovering && retryCount < 3 && lastDevice != null) {
                    retryCount++
                    handler.postDelayed({ lastDevice?.let { connectToGatt(it) } }, 1500)
                } else {
                    handler.post { tvStatus.text = getString(R.string.status_disconnected) }
                    resetBluetoothState()
                }
            }
        }

        @SuppressLint("MissingPermission")
        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (!isDiscoveringServices) return 
            
            isDiscoveringServices = false
            if (status == BluetoothGatt.GATT_SUCCESS) {
                Log.d("OowBLE", "Služby nalezeny.")
                val service = gatt.getService(SERVICE_UUID)
                if (service != null) {
                    commandCharacteristic = service.getCharacteristic(CHAR_COMMAND_UUID)
                    heartbeatCharacteristic = service.getCharacteristic(CHAR_HEARTBEAT_UUID)
                    telemetryCharacteristic = service.getCharacteristic(CHAR_TELEMETRY_UUID)
                    enableTelemetryNotifications(gatt, telemetryCharacteristic)
                    
                    val deviceName = gatt.device.name ?: getString(R.string.unknown_device)
                    handler.post { tvStatus.text = getString(R.string.status_ready, deviceName) }
                } else {
                    handler.post { tvStatus.text = getString(R.string.status_error_no_service) }
                    gatt.disconnect()
                }
            } else {
                Log.e("OowBLE", "onServicesDiscovered ERROR: $status")
                gatt.disconnect()
            }
        }

        override fun onCharacteristicRead(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic, value: ByteArray, status: Int) {
            isReadingTelemetry = false
            if (status == BluetoothGatt.GATT_SUCCESS && characteristic.uuid == CHAR_TELEMETRY_UUID) {
                updateTelemetryUI(value)
            }
        }

        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic, value: ByteArray) {
            if (characteristic.uuid == CHAR_TELEMETRY_UUID) updateTelemetryUI(value)
        }

        override fun onCharacteristicWrite(gatt: BluetoothGatt?, characteristic: BluetoothGattCharacteristic?, status: Int) {
            Log.d("OowBLE", "onCharacteristicWrite status=$status, uuid=${characteristic?.uuid}")
        }
    }

    private fun refreshDeviceCache(gatt: BluetoothGatt) {
        try {
            val method = gatt.javaClass.getMethod("refresh")
            method.invoke(gatt)
            Log.d("OowBLE", "GATT cache refresh vyvolán.")
        } catch (_: Exception) { Log.e("OowBLE", "Refresh cache selhal") }
    }

    private fun updateTelemetryUI(value: ByteArray) {
        val data = String(value, Charsets.UTF_8)
        handler.post { 
            tvTelemetry.text = data 
            try {
                val json = JSONObject(data)
                
                // Update Camera Switch
                    if (json.has("camera_status")) {
                        val camStatus = json.getString("camera_status")
                        switchCamera.setOnCheckedChangeListener(null)
                        switchCamera.isChecked = camStatus.equals("ON", ignoreCase = true)
                        switchCamera.setOnCheckedChangeListener { _, isChecked ->
                            if (isChecked) sendCommand("CAMERA_ON") else sendCommand("CAMERA_OFF")
                        }
                    }

                    // Update Lidar Switch
                    if (json.has("lidar_status")) {
                        val lidarStatus = json.getString("lidar_status")
                        switchLidar.setOnCheckedChangeListener(null)
                        switchLidar.isChecked = lidarStatus.equals("ON", ignoreCase = true)
                        switchLidar.setOnCheckedChangeListener { _, isChecked ->
                            if (isChecked) sendCommand("LIDAR_ON") else sendCommand("LIDAR_OFF")
                        }
                    }

                    // Update RTK Switch
                    if (json.has("rtk_status")) {
                        val rtkStatus = json.getString("rtk_status")
                        switchRtk.setOnCheckedChangeListener(null)
                        switchRtk.isChecked = rtkStatus.equals("ON", ignoreCase = true)
                        switchRtk.setOnCheckedChangeListener { _, isChecked ->
                            if (isChecked) sendCommand("RTK_ON") else sendCommand("RTK_OFF")
                        }
                    }

                    // Update Drive Switch
                    if (json.has("drive_status")) {
                        val driveStatus = json.getString("drive_status")
                        switchDrive.setOnCheckedChangeListener(null)
                        switchDrive.isChecked = driveStatus.equals("ON", ignoreCase = true)
                        switchDrive.setOnCheckedChangeListener { _, isChecked ->
                            if (isChecked) sendCommand("DRIVE_ON") else sendCommand("DRIVE_OFF")
                        }
                    }
                    
                    // Update Fusion Switch
                if (json.has("fusion_status")) {
                    val fusionStatus = json.getString("fusion_status")
                    switchFusion.setOnCheckedChangeListener(null)
                    switchFusion.isChecked = fusionStatus.equals("ON", ignoreCase = true)
                    switchFusion.setOnCheckedChangeListener { _, isChecked ->
                        if (isChecked) sendCommand("FUSION_ON") else sendCommand("FUSION_OFF")
                    }
                }
            } catch (_: Exception) {
                // Tichá chyba, pokud data nejsou JSON
            }
        }
    }

    @SuppressLint("MissingPermission")
    private fun sendCommand(command: String) {
        val gatt = bluetoothGatt
        val char = commandCharacteristic
        if (gatt != null && char != null) {
            val payload = "${getClientId()}:$command".toByteArray()
            gatt.writeCharacteristic(char, payload, BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE)
        }
    }

    private val heartbeatRunnable = object : Runnable {
        @SuppressLint("MissingPermission")
        override fun run() {
            if (!isHeartbeatActive) return
            val gatt = bluetoothGatt
            val char = heartbeatCharacteristic
            if (gatt != null && char != null && !isReadingTelemetry) {
                val payload = getClientId().toByteArray()
                gatt.writeCharacteristic(char, payload, BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE)
            }
            handler.postDelayed(this, 333)
        }
    }

    private fun startHeartbeat() {
        if (isHeartbeatActive) return
        Log.d("OowBLE", "Start Heartbeat (333ms)")
        isHeartbeatActive = true
        handler.post(heartbeatRunnable)
    }

    private fun stopHeartbeat() {
        Log.d("OowBLE", "Stop Heartbeat")
        isHeartbeatActive = false
        handler.removeCallbacks(heartbeatRunnable)
    }

    @SuppressLint("MissingPermission")
    private fun enableTelemetryNotifications(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic?) {
        if (characteristic != null && (characteristic.properties and BluetoothGattCharacteristic.PROPERTY_NOTIFY != 0)) {
            gatt.setCharacteristicNotification(characteristic, true)
            val desc = characteristic.getDescriptor(UUID.fromString("00002902-0000-1000-8000-00805f9b34fb"))
            if (desc != null) {
                gatt.writeDescriptor(desc, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE)
            }
        }
    }

    private var isReadingTelemetry = false
    @SuppressLint("MissingPermission")
    private fun readTelemetry() {
        val gatt = bluetoothGatt
        val char = telemetryCharacteristic
        if (gatt != null && char != null && !isReadingTelemetry) {
            isReadingTelemetry = true
            tvTelemetry.setText(R.string.telemetry_loading)
            if (!gatt.readCharacteristic(char)) isReadingTelemetry = false
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        try { unregisterReceiver(bluetoothStateReceiver) } catch (_: Exception) { }
        disconnectGatt()
    }
}
