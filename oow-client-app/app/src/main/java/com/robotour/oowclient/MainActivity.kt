package com.robotour.oowclient

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.content.Context
import android.content.SharedPreferences
import android.content.pm.PackageManager
import android.os.Bundle
import android.text.Editable
import android.text.TextWatcher
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
import com.robotour.oowclient.ble.BleConnectionState
import com.robotour.oowclient.ble.BleManager
import com.robotour.oowclient.model.TelemetryData
import com.robotour.oowclient.util.Constants

class MainActivity : AppCompatActivity(), BleManager.BleCallback {

    private lateinit var tvStatus: TextView
    private lateinit var tvTelemetry: TextView
    private lateinit var btnConnect: Button
    private lateinit var etClientName: EditText
    private lateinit var switchWatch: SwitchCompat
    private lateinit var switchCamera: SwitchCompat
    private lateinit var switchLidar: SwitchCompat
    private lateinit var switchGamepad: SwitchCompat
    private lateinit var switchRtk: SwitchCompat
    private lateinit var switchFusion: SwitchCompat
    private lateinit var switchDrive: SwitchCompat
    private lateinit var btnPause: Button
    private lateinit var btnResume: Button
    private lateinit var btnStop: Button
    private lateinit var btnPowerOff: Button
    private lateinit var btnCameraStatus: Button
    private lateinit var btnLidarStatus: Button
    private lateinit var btnGamepadStatus: Button
    private lateinit var btnRtkStatus: Button
    private lateinit var btnFusionStatus: Button
    private lateinit var btnDriveStatus: Button
    private lateinit var btnWaypointsStart: Button
    private lateinit var btnWaypointsStatus: Button
    private lateinit var btnTelemetry: Button

    private lateinit var bleManager: BleManager
    private lateinit var sharedPref: SharedPreferences

    private val discoveredDevices = mutableListOf<BluetoothDevice>()
    private var scanDialog: AlertDialog? = null
    private var deviceListAdapter: ArrayAdapter<String>? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        initViews()
        initPreferences()
        initBle()
        initListeners()
    }

    private fun initViews() {
        tvStatus = findViewById(R.id.tvStatus)
        tvTelemetry = findViewById(R.id.tvTelemetry)
        btnConnect = findViewById(R.id.btnConnect)
        etClientName = findViewById(R.id.etClientName)
        switchWatch = findViewById(R.id.switchWatch)
        switchCamera = findViewById(R.id.switchCamera)
        switchLidar = findViewById(R.id.switchLidar)
        switchGamepad = findViewById(R.id.switchGamepad)
        switchRtk = findViewById(R.id.switchRtk)
        switchFusion = findViewById(R.id.switchFusion)
        switchDrive = findViewById(R.id.switchDrive)
        btnPause = findViewById(R.id.btnPause)
        btnResume = findViewById(R.id.btnResume)
        btnStop = findViewById(R.id.btnStop)
        btnPowerOff = findViewById(R.id.btnPowerOff)
        btnCameraStatus = findViewById(R.id.btnCameraStatus)
        btnLidarStatus = findViewById(R.id.btnLidarStatus)
        btnGamepadStatus = findViewById(R.id.btnGamepadStatus)
        btnRtkStatus = findViewById(R.id.btnRtkStatus)
        btnFusionStatus = findViewById(R.id.btnFusionStatus)
        btnDriveStatus = findViewById(R.id.btnDriveStatus)
        btnWaypointsStart = findViewById(R.id.btnWaypointsStart)
        btnWaypointsStatus = findViewById(R.id.btnWaypointsStatus)
        btnTelemetry = findViewById(R.id.btnTelemetry)
    }

    private fun initPreferences() {
        sharedPref = getSharedPreferences(Constants.PREFS_NAME, Context.MODE_PRIVATE)
        etClientName.setText(sharedPref.getString(Constants.PREF_CLIENT_NAME, getString(R.string.default_operator_name)))

        etClientName.addTextChangedListener(object : TextWatcher {
            override fun beforeTextChanged(s: CharSequence?, start: Int, count: Int, after: Int) {}
            override fun onTextChanged(s: CharSequence?, start: Int, before: Int, count: Int) {}
            override fun afterTextChanged(s: Editable?) {
                sharedPref.edit { putString(Constants.PREF_CLIENT_NAME, s.toString()) }
            }
        })
    }

    private fun initBle() {
        bleManager = BleManager(this, this)
        bleManager.registerStateReceiver()
        setupConnectButton()
    }

    private fun initListeners() {
        switchWatch.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked) {
                bleManager.startHeartbeat { getClientId() }
            } else {
                bleManager.stopHeartbeat()
            }
        }

        switchCamera.setOnCheckedChangeListener { _, isChecked ->
            bleManager.sendCommand(getClientId(), if (isChecked) "CAMERA_ON" else "CAMERA_OFF")
        }

        switchLidar.setOnCheckedChangeListener { _, isChecked ->
            bleManager.sendCommand(getClientId(), if (isChecked) "LIDAR_ON" else "LIDAR_OFF")
        }

        switchGamepad.setOnCheckedChangeListener { _, isChecked ->
            bleManager.sendCommand(getClientId(), if (isChecked) "GAMEPAD_ON" else "GAMEPAD_OFF")
        }

        switchRtk.setOnCheckedChangeListener { _, isChecked ->
            bleManager.sendCommand(getClientId(), if (isChecked) "RTK_ON" else "RTK_OFF")
        }

        switchFusion.setOnCheckedChangeListener { _, isChecked ->
            bleManager.sendCommand(getClientId(), if (isChecked) "FUSION_ON" else "FUSION_OFF")
        }

        switchDrive.setOnCheckedChangeListener { _, isChecked ->
            bleManager.sendCommand(getClientId(), if (isChecked) "DRIVE_ON" else "DRIVE_OFF")
        }

        btnPause.setOnClickListener { bleManager.sendCommand(getClientId(), "PAUSE") }
        btnResume.setOnClickListener { bleManager.sendCommand(getClientId(), "RESUME") }
        btnStop.setOnClickListener { bleManager.sendCommand(getClientId(), "STOP") }

        btnPowerOff.setOnClickListener {
            AlertDialog.Builder(this)
                .setTitle(R.string.dialog_power_off_title)
                .setMessage(R.string.dialog_power_off_message)
                .setPositiveButton(R.string.yes) { _, _ -> bleManager.sendCommand(getClientId(), "POWEROFF") }
                .setNegativeButton(R.string.no, null)
                .show()
        }

        btnCameraStatus.setOnClickListener { bleManager.sendCommand(getClientId(), "CAMERA_STATUS") }
        btnLidarStatus.setOnClickListener { bleManager.sendCommand(getClientId(), "LIDAR_STATUS") }
        btnGamepadStatus.setOnClickListener { bleManager.sendCommand(getClientId(), "GAMEPAD_STATUS") }
        btnRtkStatus.setOnClickListener { bleManager.sendCommand(getClientId(), "RTK_STATUS") }
        btnFusionStatus.setOnClickListener { bleManager.sendCommand(getClientId(), "FUSION_STATUS") }
        btnDriveStatus.setOnClickListener { bleManager.sendCommand(getClientId(), "DRIVE_STATUS") }
        btnWaypointsStart.setOnClickListener { bleManager.sendCommand(getClientId(), "PILOT_WAYPOINTS_START") }
        btnWaypointsStatus.setOnClickListener { bleManager.sendCommand(getClientId(), "PILOT_WAYPOINTS_STATUS") }
        btnTelemetry.setOnClickListener {
            tvTelemetry.setText(R.string.telemetry_loading)
            bleManager.readTelemetry()
        }
    }

    private fun getClientId(): String = etClientName.text.toString().trim().ifEmpty { "Unknown" }

    // --- BLE Scan Dialog & Permissions ---

    private fun checkBluetoothPermissions(): Boolean {
        val permissions = arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
        return permissions.all { ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED }
    }

    private fun requestBluetoothPermissions() {
        val permissions = arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
        ActivityCompat.requestPermissions(this, permissions, Constants.PERMISSION_REQUEST_CODE)
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == Constants.PERMISSION_REQUEST_CODE && grantResults.isNotEmpty() && grantResults.all { it == PackageManager.PERMISSION_GRANTED }) {
            startScanWithDialog()
        }
    }

    private fun startScanWithDialog() {
        if (!bleManager.isBluetoothEnabled) {
            Toast.makeText(this, R.string.toast_enable_bluetooth, Toast.LENGTH_SHORT).show()
            return
        }

        discoveredDevices.clear()
        val deviceLabels = mutableListOf<String>()
        deviceListAdapter = ArrayAdapter(this, android.R.layout.simple_list_item_1, deviceLabels)

        scanDialog = AlertDialog.Builder(this)
            .setTitle(R.string.scan_dialog_title)
            .setAdapter(deviceListAdapter) { _, which ->
                if (which in discoveredDevices.indices) {
                    val selected = discoveredDevices[which]
                    bleManager.stopScan()
                    bleManager.connect(selected)
                }
            }
            .setNegativeButton(R.string.btn_cancel) { _, _ -> bleManager.stopScan() }
            .setOnDismissListener { bleManager.stopScan() }
            .show()

        bleManager.startScan()
    }

    private fun setupConnectButton() {
        btnConnect.setText(R.string.btn_search_robot)
        btnConnect.isEnabled = true
        btnConnect.setOnClickListener {
            if (checkBluetoothPermissions()) startScanWithDialog() else requestBluetoothPermissions()
        }
    }

    private fun setupDisconnectButton() {
        btnConnect.setText(R.string.btn_disconnect)
        btnConnect.isEnabled = true
        btnConnect.setOnClickListener { bleManager.disconnect() }
    }

    // --- BleManager.BleCallback Implementation ---

    override fun onStateChanged(state: BleConnectionState, deviceName: String?, errorInfo: String?) {
        runOnUiThread {
            @SuppressLint("MissingPermission")
            val name = deviceName ?: getString(R.string.unknown_device)
            when (state) {
                BleConnectionState.DISCONNECTED -> {
                    tvStatus.text = getString(R.string.status_disconnected)
                    switchWatch.isChecked = false
                    setupConnectButton()
                }
                BleConnectionState.DISCONNECTING -> {
                    tvStatus.text = getString(R.string.status_disconnecting)
                    btnConnect.isEnabled = false
                }
                BleConnectionState.CONNECTING -> {
                    tvStatus.text = getString(R.string.status_connecting, name)
                    btnConnect.isEnabled = false
                }
                BleConnectionState.SEARCHING_SERVICES -> {
                    tvStatus.text = getString(R.string.status_searching_services, name)
                    setupDisconnectButton()
                }
                BleConnectionState.READY -> {
                    tvStatus.text = getString(R.string.status_ready, name)
                    setupDisconnectButton()
                }
                BleConnectionState.BLUETOOTH_OFF -> {
                    tvStatus.text = getString(R.string.status_bluetooth_off)
                    switchWatch.isChecked = false
                    setupConnectButton()
                }
                BleConnectionState.ERROR -> {
                    tvStatus.text = errorInfo ?: getString(R.string.status_error_connection, 0)
                    setupConnectButton()
                }
            }
        }
    }

    override fun onTelemetryReceived(telemetry: TelemetryData) {
        runOnUiThread {
            tvTelemetry.text = telemetry.rawJson
            updateSwitchQuietly(switchCamera, telemetry.cameraOn, "CAMERA_ON", "CAMERA_OFF")
            updateSwitchQuietly(switchLidar, telemetry.lidarOn, "LIDAR_ON", "LIDAR_OFF")
            updateSwitchQuietly(switchGamepad, telemetry.gamepadOn, "GAMEPAD_ON", "GAMEPAD_OFF")
            updateSwitchQuietly(switchRtk, telemetry.rtkOn, "RTK_ON", "RTK_OFF")
            updateSwitchQuietly(switchDrive, telemetry.driveOn, "DRIVE_ON", "DRIVE_OFF")
            updateSwitchQuietly(switchFusion, telemetry.fusionOn, "FUSION_ON", "FUSION_OFF")
        }
    }

    private fun updateSwitchQuietly(switch: SwitchCompat, isChecked: Boolean?, onCommand: String, offCommand: String) {
        if (isChecked == null) return
        switch.setOnCheckedChangeListener(null)
        switch.isChecked = isChecked
        switch.setOnCheckedChangeListener { _, checked ->
            bleManager.sendCommand(getClientId(), if (checked) onCommand else offCommand)
        }
    }

    @SuppressLint("MissingPermission")
    override fun onDeviceDiscovered(device: BluetoothDevice, rssi: Int) {
        runOnUiThread {
            if (discoveredDevices.none { it.address == device.address }) {
                discoveredDevices.add(device)
                val name = device.name ?: getString(R.string.unknown_device)
                deviceListAdapter?.add(getString(R.string.scan_device_info, name, device.address, rssi))
                deviceListAdapter?.notifyDataSetChanged()
            }
        }
    }

    override fun onScanFinished(devicesFoundCount: Int) {
        runOnUiThread {
            if (scanDialog?.isShowing == true && devicesFoundCount == 0 && discoveredDevices.isEmpty()) {
                scanDialog?.setTitle(R.string.scan_no_robot_found)
            }
        }
    }

    override fun onGattRetry(status: Int, retryCount: Int) {
        runOnUiThread {
            tvStatus.text = getString(R.string.status_error_gatt_retry, status, retryCount)
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        bleManager.cleanup()
    }
}
