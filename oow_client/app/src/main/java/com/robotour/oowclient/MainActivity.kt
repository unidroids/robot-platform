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
import android.util.Log
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.ParcelUuid
import android.text.Editable
import android.text.TextWatcher
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.EditText
import android.widget.Switch
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import java.util.UUID

class MainActivity : AppCompatActivity() {

    private lateinit var tvStatus: TextView
    private lateinit var tvTelemetry: TextView
    private lateinit var btnConnect: Button
    private lateinit var etClientName: EditText
    private lateinit var switchWatch: Switch
    private lateinit var btnPause: Button
    private lateinit var btnResume: Button
    private lateinit var btnStop: Button
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
                        tvStatus.text = "Stav: Bluetooth vypnuto"
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
        btnPause = findViewById(R.id.btnPause)
        btnResume = findViewById(R.id.btnResume)
        btnStop = findViewById(R.id.btnStop)
        btnTelemetry = findViewById(R.id.btnTelemetry)

        sharedPref = getSharedPreferences("OowClientPrefs", Context.MODE_PRIVATE)
        etClientName.setText(sharedPref.getString("client_name", "Operator1"))

        etClientName.addTextChangedListener(object : TextWatcher {
            override fun beforeTextChanged(s: CharSequence?, start: Int, count: Int, after: Int) {}
            override fun onTextChanged(s: CharSequence?, start: Int, before: Int, count: Int) {}
            override fun afterTextChanged(s: Editable?) {
                sharedPref.edit().putString("client_name", s.toString()).apply()
            }
        })

        bluetoothAdapter = (getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager).adapter
        setupConnectButton()
        registerReceiver(bluetoothStateReceiver, IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED))

        switchWatch.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked) startHeartbeat() else stopHeartbeat()
        }

        btnPause.setOnClickListener { sendCommand("PAUSE") }
        btnResume.setOnClickListener { sendCommand("RESUME") }
        btnStop.setOnClickListener { sendCommand("STOP") }
        btnTelemetry.setOnClickListener { readTelemetry() }
    }

    private fun getClientId(): String = etClientName.text.toString().trim().ifEmpty { "Unknown" }

    private fun checkBluetoothPermissions(): Boolean {
        val permissions = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)
        }
        return permissions.all { ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED }
    }

    private fun requestBluetoothPermissions() {
        val permissions = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            arrayOf(Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION)
        }
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
            btnConnect.text = "Vyhledat robota (OOW)"
            btnConnect.isEnabled = true
            btnConnect.setOnClickListener {
                if (checkBluetoothPermissions()) startDeviceScan() else requestBluetoothPermissions()
            }
        }
    }

    private fun setupDisconnectButton() {
        runOnUiThread {
            btnConnect.text = "ODPOJIT"
            btnConnect.isEnabled = true
            btnConnect.setOnClickListener { disconnectGatt() }
        }
    }

    @SuppressLint("MissingPermission")
    private fun disconnectGatt() {
        Log.d("OowBLE", "disconnectGatt() vyvoláno")
        tvStatus.text = "Stav: Odpojuji..."
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
            tvStatus.text = "Stav: Odpojeno"
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
            Toast.makeText(this, "Zapněte Bluetooth!", Toast.LENGTH_SHORT).show()
            return
        }

        discoveredDevices.clear()
        val deviceNames = mutableListOf<String>()
        deviceListAdapter = ArrayAdapter(this, android.R.layout.simple_list_item_1, deviceNames)

        scanDialog = AlertDialog.Builder(this)
            .setTitle("Vyhledávám roboty...")
            .setAdapter(deviceListAdapter) { _, which ->
                stopScan()
                connectToGatt(discoveredDevices[which])
            }
            .setNegativeButton("Zrušit") { _, _ -> stopScan() }
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
            scanDialog?.setTitle("Žádný robot nenalezen")
        }
    }

    private val scanCallback = object : ScanCallback() {
        @SuppressLint("MissingPermission")
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val device = result.device
            if (device !in discoveredDevices) {
                discoveredDevices.add(device)
                val name = device.name ?: "Neznámé"
                handler.post {
                    deviceListAdapter?.add("$name\n(${device.address}) [${result.rssi} dBm]")
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
        tvStatus.text = "Připojuji k: ${device.address}..."
        btnConnect.isEnabled = false

        bluetoothGatt = device.connectGatt(this, false, gattCallback, BluetoothDevice.TRANSPORT_LE)
    }

    private val gattCallback = object : BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            Log.d("OowBLE", "onConnectionStateChange: status=$status, newState=$newState")
            
            if (status != BluetoothGatt.GATT_SUCCESS) {
                isDiscoveringServices = false
                if ((status == 133 || status == 19 || status == 62) && retryCount < 3 && lastDevice != null) {
                    retryCount++
                    handler.post { tvStatus.text = "GATT Chyba $status. Retry $retryCount/3..." }
                    if (status == 133) refreshDeviceCache(gatt)
                    gatt.close()
                    handler.postDelayed({ lastDevice?.let { connectToGatt(it) } }, 1000)
                } else {
                    handler.post { tvStatus.text = "Chyba spojení ($status)" }
                    gatt.close()
                    resetBluetoothState()
                }
                return
            }

            if (newState == BluetoothProfile.STATE_CONNECTED) {
                isDiscoveringServices = true
                retryCount = 0
                val deviceName = gatt.device.name ?: "Neznámé"
                val deviceAddress = gatt.device.address
                handler.post { 
                    tvStatus.text = "Spojeno s $deviceName ($deviceAddress)\nHledám služby..."
                    setupDisconnectButton()
                }
                
                handler.postDelayed({
                    Log.d("OowBLE", "Spouštím discoverServices()")
                    if (bluetoothGatt == null || !gatt.discoverServices()) {
                        Log.e("OowBLE", "discoverServices() selhalo!")
                        gatt.disconnect()
                    }
                }, 250) // Zkráceno z 800ms pro rychlejší odezvu

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
                    handler.post { tvStatus.text = "Odpojeno" }
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
                    
                    val deviceName = gatt.device.name ?: "Neznámé"
                    val deviceAddress = gatt.device.address
                    handler.post { tvStatus.text = "Připraveno: $deviceName\n($deviceAddress)" }
                } else {
                    handler.post { tvStatus.text = "Služba OOW nenalezena!" }
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
        } catch (e: Exception) { Log.e("OowBLE", "Refresh cache selhal: ${e.message}") }
    }

    private fun updateTelemetryUI(value: ByteArray) {
        val data = String(value, Charsets.UTF_8)
        handler.post { tvTelemetry.text = data }
    }

    @SuppressLint("MissingPermission")
    private fun sendCommand(command: String) {
        val gatt = bluetoothGatt
        val char = commandCharacteristic
        if (gatt != null && char != null) {
            val payload = "${getClientId()}:$command".toByteArray()
            if (Build.VERSION.SDK_INT >= 33) {
                gatt.writeCharacteristic(char, payload, BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE)
            } else {
                char.writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
                char.value = payload
                gatt.writeCharacteristic(char)
            }
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
                if (Build.VERSION.SDK_INT >= 33) {
                    gatt.writeCharacteristic(char, payload, BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE)
                } else {
                    char.writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
                    char.value = payload
                    gatt.writeCharacteristic(char)
                }
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
                if (Build.VERSION.SDK_INT >= 33) gatt.writeDescriptor(desc, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE)
                else { desc.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE; gatt.writeDescriptor(desc) }
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
            tvTelemetry.text = "Načítám..."
            if (!gatt.readCharacteristic(char)) isReadingTelemetry = false
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        try { unregisterReceiver(bluetoothStateReceiver) } catch (e: Exception) { }
        disconnectGatt()
    }
}
