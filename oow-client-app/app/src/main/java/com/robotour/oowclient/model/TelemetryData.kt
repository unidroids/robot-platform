package com.robotour.oowclient.model

import org.json.JSONObject

data class TelemetryData(
    val rawJson: String,
    val cameraOn: Boolean? = null,
    val lidarOn: Boolean? = null,
    val gamepadOn: Boolean? = null,
    val rtkOn: Boolean? = null,
    val driveOn: Boolean? = null,
    val fusionOn: Boolean? = null
) {
    companion object {
        fun fromJson(raw: String): TelemetryData {
            var camera: Boolean? = null
            var lidar: Boolean? = null
            var gamepad: Boolean? = null
            var rtk: Boolean? = null
            var drive: Boolean? = null
            var fusion: Boolean? = null

            try {
                val json = JSONObject(raw)
                if (json.has("camera_status")) {
                    camera = json.optString("camera_status").equals("ON", ignoreCase = true)
                }
                if (json.has("lidar_status")) {
                    lidar = json.optString("lidar_status").equals("ON", ignoreCase = true)
                }
                if (json.has("gamepad_status")) {
                    gamepad = json.optString("gamepad_status").equals("ON", ignoreCase = true)
                }
                if (json.has("rtk_status")) {
                    rtk = json.optString("rtk_status").equals("ON", ignoreCase = true)
                }
                if (json.has("drive_status")) {
                    drive = json.optString("drive_status").equals("ON", ignoreCase = true)
                }
                if (json.has("fusion_status")) {
                    fusion = json.optString("fusion_status").equals("ON", ignoreCase = true)
                }
            } catch (_: Exception) {
                // Return data with raw string if parsing fails
            }

            return TelemetryData(
                rawJson = raw,
                cameraOn = camera,
                lidarOn = lidar,
                gamepadOn = gamepad,
                rtkOn = rtk,
                driveOn = drive,
                fusionOn = fusion
            )
        }
    }
}
