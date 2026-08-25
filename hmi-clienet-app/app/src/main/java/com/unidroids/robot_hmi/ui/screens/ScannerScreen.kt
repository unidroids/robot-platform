package com.unidroids.robot_hmi.ui.screens

import android.Manifest
import androidx.activity.compose.BackHandler
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.Preview
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.offset
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableLongStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Rect
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.PathFillType
import androidx.compose.ui.graphics.ClipOp
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.clipRect
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import androidx.lifecycle.compose.LocalLifecycleOwner
import com.google.accompanist.permissions.ExperimentalPermissionsApi
import com.google.accompanist.permissions.isGranted
import com.google.accompanist.permissions.rememberPermissionState
import com.google.mlkit.vision.barcode.BarcodeScannerOptions
import com.google.mlkit.vision.barcode.BarcodeScanning
import com.google.mlkit.vision.barcode.common.Barcode
import com.google.mlkit.vision.common.InputImage
import com.unidroids.robot_hmi.viewmodel.MainViewModel
import kotlin.math.min

@OptIn(ExperimentalPermissionsApi::class)
@Composable
fun ScannerScreen(
    viewModel: MainViewModel,
    onBack: () -> Unit
) {
    val cameraPermissionState = rememberPermissionState(Manifest.permission.CAMERA)

    DisposableEffect(Unit) {
        onDispose {
            viewModel.onScannerHandled()
        }
    }

    BackHandler {
        onBack()
    }

    if (cameraPermissionState.status.isGranted) {
        CameraPreviewWithOverlay(viewModel, onBack)
    } else {
        LaunchedEffect(Unit) {
            cameraPermissionState.launchPermissionRequest()
        }
        Box(
            modifier = Modifier
                .fillMaxSize()
                .background(Color.Black),
            contentAlignment = Alignment.Center
        ) {
            Text(
                text = "> REQUESTING CAMERA PERMISSION...",
                color = Color.Green,
                fontFamily = FontFamily.Monospace
            )
        }
    }
}

@Composable
fun CameraPreviewWithOverlay(viewModel: MainViewModel, onBack: () -> Unit) {
    val context = LocalContext.current
    val lifecycleOwner = LocalLifecycleOwner.current

    // Store the last scanned code and timestamp for cooldown
    var lastScannedCode by remember { mutableStateOf<String?>(null) }
    var lastScanTimestamp by remember { mutableLongStateOf(0L) }
    
    // UI state for showing the code
    var displayedCode by remember { mutableStateOf("---") }

    val analyzer = remember {
        val options = BarcodeScannerOptions.Builder()
            .setBarcodeFormats(Barcode.FORMAT_ALL_FORMATS)
            .build()
        val scanner = BarcodeScanning.getClient(options)
        
        ImageAnalysis.Analyzer { imageProxy ->
            val mediaImage = imageProxy.image
            if (mediaImage != null) {
                val image = InputImage.fromMediaImage(mediaImage, imageProxy.imageInfo.rotationDegrees)
                scanner.process(image)
                    .addOnSuccessListener { barcodes ->
                        val barcode = barcodes.firstOrNull()?.rawValue
                        if (barcode != null) {
                            val now = System.currentTimeMillis()
                            // Cooldown: 0s for new code, 2s for the same code
                            if (barcode != lastScannedCode || (now - lastScanTimestamp > 2000)) {
                                lastScannedCode = barcode
                                lastScanTimestamp = now
                                displayedCode = barcode
                                viewModel.onQrCodeScanned(barcode)
                            }
                        }
                    }
                    .addOnCompleteListener {
                        imageProxy.close()
                    }
            } else {
                imageProxy.close()
            }
        }
    }

    BoxWithConstraints(modifier = Modifier.fillMaxSize()) {
        val boxSizeDp = (min(maxWidth.value, maxHeight.value) * 0.6f).dp
        
        AndroidView(
            modifier = Modifier.fillMaxSize(),
            factory = { ctx ->
                val previewView = PreviewView(ctx)
                val cameraProviderFuture = ProcessCameraProvider.getInstance(ctx)
                
                cameraProviderFuture.addListener({
                    val cameraProvider = cameraProviderFuture.get()
                    val preview = Preview.Builder().build().also {
                        it.setSurfaceProvider(previewView.surfaceProvider)
                    }
                    
                    val imageAnalysis = ImageAnalysis.Builder()
                        .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                        .build()
                        .also {
                            it.setAnalyzer(ContextCompat.getMainExecutor(ctx), analyzer)
                        }

                    val cameraSelector = CameraSelector.DEFAULT_BACK_CAMERA
                    
                    try {
                        cameraProvider.unbindAll()
                        cameraProvider.bindToLifecycle(lifecycleOwner, cameraSelector, preview, imageAnalysis)
                    } catch (e: Exception) {
                        e.printStackTrace()
                    }
                }, ContextCompat.getMainExecutor(ctx))
                
                previewView
            }
        )
        
        // Target Overlay
        Canvas(modifier = Modifier.fillMaxSize()) {
            val canvasWidth = size.width
            val canvasHeight = size.height
            val boxSize = min(canvasWidth, canvasHeight) * 0.6f
            
            val left = (canvasWidth - boxSize) / 2
            val top = (canvasHeight - boxSize) / 2
            val right = (canvasWidth + boxSize) / 2
            val bottom = (canvasHeight + boxSize) / 2

            // Draw dark background with cutout
            clipRect(left, top, right, bottom, clipOp = ClipOp.Difference) {
                drawRect(color = Color.Black.copy(alpha = 0.6f))
            }
            
            // Draw crosshairs / borders
            val length = boxSize * 0.1f
            
            val stroke = Stroke(width = 4.dp.toPx())
            val cornerColor = Color.Green
            
            // Top Left
            drawLine(cornerColor, Offset(left, top), Offset(left + length, top), stroke.width)
            drawLine(cornerColor, Offset(left, top), Offset(left, top + length), stroke.width)
            // Top Right
            drawLine(cornerColor, Offset(right, top), Offset(right - length, top), stroke.width)
            drawLine(cornerColor, Offset(right, top), Offset(right, top + length), stroke.width)
            // Bottom Left
            drawLine(cornerColor, Offset(left, bottom), Offset(left + length, bottom), stroke.width)
            drawLine(cornerColor, Offset(left, bottom), Offset(left, bottom - length), stroke.width)
            // Bottom Right
            drawLine(cornerColor, Offset(right, bottom), Offset(right - length, bottom), stroke.width)
            drawLine(cornerColor, Offset(right, bottom), Offset(right, bottom - length), stroke.width)
        }
        
        Text(
            text = "Scan QR Code",
            color = Color.Green,
            fontFamily = FontFamily.Monospace,
            fontWeight = FontWeight.Bold,
            modifier = Modifier
                .align(Alignment.Center)
                .offset(y = -(boxSizeDp / 2) - 32.dp)
        )

        // Text showing last scanned code
        Column(
            modifier = Modifier
                .align(Alignment.BottomCenter)
                .padding(bottom = 32.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Text(
                text = "LAST SCAN: $displayedCode",
                color = Color.White,
                fontFamily = FontFamily.Monospace,
                modifier = Modifier
                    .background(Color.Black.copy(alpha = 0.7f))
                    .padding(horizontal = 16.dp, vertical = 8.dp)
            )
        }
    }
}
