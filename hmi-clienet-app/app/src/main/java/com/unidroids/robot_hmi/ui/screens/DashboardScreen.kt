package com.unidroids.robot_hmi.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Text
import androidx.compose.material3.adaptive.layout.ListDetailPaneScaffold
import androidx.compose.material3.adaptive.layout.PaneAdaptedValue
import androidx.compose.material3.adaptive.navigation.rememberListDetailPaneScaffoldNavigator
import androidx.compose.material3.adaptive.ExperimentalMaterial3AdaptiveApi
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.foundation.layout.Row
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.Box
import com.unidroids.robot_hmi.network.MessageConfig
import com.unidroids.robot_hmi.viewmodel.MainViewModel

@OptIn(ExperimentalMaterial3AdaptiveApi::class)
@Composable
fun DashboardScreen(viewModel: MainViewModel) {
    val networkState by viewModel.networkState.collectAsState()
    val navigator = rememberListDetailPaneScaffoldNavigator<Nothing>()

    Box(modifier = Modifier.fillMaxSize()) {
        ListDetailPaneScaffold(
            directive = navigator.scaffoldDirective,
            value = navigator.scaffoldValue,
            listPane = {
                TerminalStatusPane(
                    isHmiConnected = networkState.isHmiConnected,
                    isQrServerRunning = networkState.isQrServerRunning
                )
            },
            detailPane = {
                TerminalDiagnosticsPane(
                    lastCommand = networkState.lastCommand
                )
            },
            modifier = Modifier.fillMaxSize().background(Color.Black)
        )
        
        networkState.messageEvent?.let { config ->
            MessageOverlay(
                config = config,
                onButtonClick = { id -> viewModel.onMessageButtonClicked(id) }
            )
        }
    }
}

@Composable
fun TerminalStatusPane(isHmiConnected: Boolean, isQrServerRunning: Boolean) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(Color.Black)
            .padding(32.dp),
        verticalArrangement = Arrangement.Center,
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        val statusText = if (isHmiConnected) "ROBOT ONLINE" else "WAITING FOR ROBOT"
        val statusColor = if (isHmiConnected) Color.Green else Color.Red

        Text(
            text = "> $statusText",
            color = statusColor,
            fontSize = 42.sp,
            fontFamily = FontFamily.Monospace,
            fontWeight = FontWeight.Bold
        )

        Spacer(modifier = Modifier.height(32.dp))

        Text(
            text = "[ QR SERVER: ${if (isQrServerRunning) "RUNNING" else "STOPPED"} ]",
            color = Color.Green,
            fontSize = 18.sp,
            fontFamily = FontFamily.Monospace
        )
    }
}

@Composable
fun TerminalDiagnosticsPane(lastCommand: String) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(Color.Black)
            .padding(32.dp)
    ) {
        Text(
            text = "--- DIAGNOSTICS ---",
            color = Color.Green,
            fontSize = 20.sp,
            fontFamily = FontFamily.Monospace
        )
        Spacer(modifier = Modifier.height(16.dp))
        Text(
            text = "SYSTEM: READY",
            color = Color.Green,
            fontSize = 16.sp,
            fontFamily = FontFamily.Monospace
        )
        Text(
            text = "LAST CMD: ${lastCommand.ifEmpty { "NONE" }}",
            color = Color.Green,
            fontSize = 16.sp,
            fontFamily = FontFamily.Monospace
        )
        Spacer(modifier = Modifier.weight(1f))
        Text(
            text = "DUMB TERMINAL V1.0",
            color = Color.DarkGray,
            fontSize = 12.sp,
            fontFamily = FontFamily.Monospace
        )
    }
}

@Preview(showBackground = true, widthDp = 1280, heightDp = 800)
@Composable
fun DashboardScreenPreview() {
    TerminalStatusPane(isHmiConnected = true, isQrServerRunning = true)
}

@Composable
fun MessageOverlay(config: MessageConfig, onButtonClick: (String) -> Unit) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(Color.Black.copy(alpha = 0.9f))
            .padding(32.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(
            text = config.header,
            color = Color.Cyan,
            fontSize = 32.sp,
            fontWeight = FontWeight.Bold,
            fontFamily = FontFamily.Monospace,
            modifier = Modifier.padding(bottom = 24.dp)
        )
        
        Box(modifier = Modifier.weight(1f).fillMaxWidth()) {
            Text(
                text = config.text,
                color = Color.White,
                fontSize = 20.sp,
                fontFamily = FontFamily.Monospace,
                modifier = Modifier.verticalScroll(rememberScrollState())
            )
        }
        
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(top = 24.dp),
            horizontalArrangement = Arrangement.SpaceEvenly
        ) {
            config.buttons.forEach { button ->
                Button(
                    onClick = { onButtonClick(button.id) },
                    colors = ButtonDefaults.buttonColors(containerColor = Color.DarkGray)
                ) {
                    Text(
                        text = button.text,
                        color = Color.White,
                        fontSize = 18.sp,
                        fontFamily = FontFamily.Monospace,
                        modifier = Modifier.padding(horizontal = 16.dp, vertical = 8.dp)
                    )
                }
            }
        }
    }
}
