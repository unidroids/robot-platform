package com.unidroids.robot_hmi.network

import android.content.Context
import android.media.MediaPlayer
import android.util.Log
import java.io.File

class SoundManager(private val context: Context) {
    private var mediaPlayer: MediaPlayer? = null

    init {
        val soundsDir = File(context.getExternalFilesDir(null), "Sounds")
        if (!soundsDir.exists()) {
            soundsDir.mkdirs()
        }
    }

    fun playSound(name: String) {
        try {
            val file = File(context.getExternalFilesDir(null), "Sounds/$name.wav")
            if (!file.exists()) {
                Log.e("SoundManager", "Sound file not found: ${file.absolutePath}")
                return
            }

            mediaPlayer?.release()
            mediaPlayer = MediaPlayer().apply {
                setDataSource(file.absolutePath)
                prepare()
                start()
                setOnCompletionListener {
                    it.release()
                    mediaPlayer = null
                }
            }
        } catch (e: Exception) {
            Log.e("SoundManager", "Failed to play sound: $name", e)
            mediaPlayer?.release()
            mediaPlayer = null
        }
    }

    fun release() {
        mediaPlayer?.release()
        mediaPlayer = null
    }
}
