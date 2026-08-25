package com.unidroids.robot_hmi

import android.content.pm.ActivityInfo
import android.os.Bundle
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Modifier
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation3.runtime.NavEntry
import androidx.navigation3.runtime.rememberNavBackStack
import androidx.navigation3.runtime.entryProvider
import androidx.navigation3.runtime.NavKey as BaseNavKey
import androidx.navigation3.ui.NavDisplay
import androidx.compose.ui.graphics.Color
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.zIndex
import android.os.PowerManager
import android.content.Context
import kotlinx.coroutines.delay
import com.unidroids.robot_hmi.ui.navigation.NavKey
import com.unidroids.robot_hmi.ui.screens.DashboardScreen
import com.unidroids.robot_hmi.ui.screens.ScannerScreen
import com.unidroids.robot_hmi.ui.theme.RobotHMITheme
import com.unidroids.robot_hmi.viewmodel.MainViewModel
import com.unidroids.robot_hmi.network.SoundManager

class MainActivity : ComponentActivity() {
    private lateinit var soundManager: SoundManager

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        soundManager = SoundManager(this)
        
        setShowWhenLocked(true)
        setTurnScreenOn(true)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        // Lock to landscape
        requestedOrientation = ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE
        
        // Immersive mode
        WindowCompat.setDecorFitsSystemWindows(window, false)
        val windowInsetsController = WindowCompat.getInsetsController(window, window.decorView)
        windowInsetsController.systemBarsBehavior = WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        windowInsetsController.hide(WindowInsetsCompat.Type.systemBars())

        enableEdgeToEdge()
        
        setContent {
            RobotHMITheme {
                val viewModel: MainViewModel = viewModel()
                val backStack = rememberNavBackStack(NavKey.Dashboard)
                val networkState by viewModel.networkState.collectAsState()

                LaunchedEffect(networkState.scannerTriggered) {
                    if (networkState.scannerTriggered) {
                        if (backStack.lastOrNull() !is NavKey.Scanner) {
                            backStack.add(NavKey.Scanner)
                        }
                    } else {
                        if (backStack.lastOrNull() is NavKey.Scanner) {
                            backStack.removeAt(backStack.size - 1)
                        }
                    }
                }

                val myEntryProvider = entryProvider<BaseNavKey> {
                    addEntryProvider(NavKey.Dashboard) {
                        DashboardScreen(viewModel = viewModel)
                    }
                    addEntryProvider(NavKey.Scanner) {
                        ScannerScreen(
                            viewModel = viewModel,
                            onBack = {
                                if (backStack.lastOrNull() is NavKey.Scanner) {
                                    backStack.removeAt(backStack.size - 1)
                                }
                            }
                        )
                    }
                }

                Box(modifier = Modifier.fillMaxSize()) {
                    NavDisplay(
                        backStack = backStack,
                        modifier = Modifier.fillMaxSize(),
                        entryProvider = myEntryProvider
                    )

                    // Sound playback
                    LaunchedEffect(networkState.soundEvent) {
                        networkState.soundEvent?.let { event ->
                            soundManager.playSound(event.name)
                        }
                    }

                    // Blink Overlay
                    val blinkEvent = networkState.blinkEvent
                    if (blinkEvent != null) {
                        val timeSinceEvent = System.currentTimeMillis() - blinkEvent.timestamp
                        if (timeSinceEvent < blinkEvent.duration) {
                            var isVisible by remember(blinkEvent) { mutableStateOf(true) }
                            
                            LaunchedEffect(blinkEvent) {
                                // Wake up screen
                                val powerManager = getSystemService(Context.POWER_SERVICE) as PowerManager
                                val wakeLock = powerManager.newWakeLock(
                                    PowerManager.SCREEN_BRIGHT_WAKE_LOCK or PowerManager.ACQUIRE_CAUSES_WAKEUP,
                                    "RobotHMI:BlinkWakeLock"
                                )
                                wakeLock.acquire(blinkEvent.duration)
                                
                                val delayTime = (1000f / blinkEvent.frequency / 2f).toLong()
                                val endTime = blinkEvent.timestamp + blinkEvent.duration
                                while (System.currentTimeMillis() < endTime) {
                                    isVisible = !isVisible
                                    delay(delayTime)
                                }
                                isVisible = false
                            }

                            if (isVisible) {
                                val parseColor = try {
                                    Color(android.graphics.Color.parseColor(blinkEvent.color))
                                } catch (e: Exception) {
                                    Color.Red // Fallback
                                }
                                Box(
                                    modifier = Modifier
                                        .fillMaxSize()
                                        .background(parseColor)
                                        .zIndex(100f)
                                )
                            }
                        }
                    }
                }
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        soundManager.release()
    }
}
