@file:OptIn(ExperimentalUnsignedTypes::class)

package paulfaria.mechaferris

import android.annotation.SuppressLint
import android.content.ComponentName
import android.content.Intent
import android.content.ServiceConnection
import android.os.Bundle
import android.os.Handler
import android.os.IBinder
import android.os.Looper
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.viewModels
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.RowScope
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Close
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.MoreVert
import androidx.compose.material.icons.filled.PlayArrow
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.lifecycle.ViewModel
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.launch
import paulfaria.mechaferris.Constants.Companion.DEVICE_ADDRESS_EXTRA
import paulfaria.mechaferris.Constants.Companion.DEVICE_NAME_EXTRA
import paulfaria.mechaferris.ui.calibrating.CalibratingView
import paulfaria.mechaferris.ui.controls.ControlsView
import paulfaria.mechaferris.ui.home.HomeView
import paulfaria.mechaferris.ui.theme.MechaFerrisTheme

enum class ScaffoldState {
    Home, Calibrating, Controls,
}

class DialogViewModel : ViewModel() {
    var showDialog by mutableStateOf(false)
    fun onDisconnected() {
        showDialog = true
    }

    fun onDismissed() {
        showDialog = false
    }
}

class Connected : ComponentActivity() {

    private val dialogViewModel: DialogViewModel by viewModels()
    private val connectedViewModel: ConnectedViewModel by viewModels()
    private val defaultScope = CoroutineScope(Dispatchers.Default)
    private val mainHandler = Handler(Looper.getMainLooper())
    private var bleServiceConn: MFServiceConn? = null
    private var bleServiceData: BleService.BluetoothServiceBinder? by mutableStateOf(null)

    private val batteryChangeNotifications = Channel<UInt>()
    private val stateMachineChangeNotifications = Channel<StateMachine>()
    private val animationFactorChangeNotifications = Channel<Float>()
    private val angularVelocityChangeNotifications = Channel<Float>()
    private val motionVectorChangeNotifications = Channel<Vector3>()
    private val bodyTranslationChangeNotifications = Channel<Translation>()
    private val bodyRotationChangeNotifications = Channel<Quaternion>()
    private val legRadiusChangeNotifications = Channel<Float>()
    private val batteryUpdateIntervalMsChangeNotifications = Channel<UInt>()
    private val getCalibrationDatumChangeNotifications = Channel<CalibrationDatum>()
    private val setCalibrationResultChangeNotifications = Channel<SetResult>()

    private inner class MFServiceConn : ServiceConnection {
        override fun onServiceConnected(name: ComponentName?, service: IBinder?) {
            if (BuildConfig.DEBUG && BleService::class.java.name != name?.className) {
                error("Connected to unknown service")
            } else {
                bleServiceData = service as BleService.BluetoothServiceBinder
                bleServiceData?.let { bleServiceData ->
                    bleServiceData.setBatteryChangedChannel(batteryChangeNotifications)
                    bleServiceData.setStateMachineChangedChannel(stateMachineChangeNotifications)
                    bleServiceData.setAnimationFactorChangedChannel(animationFactorChangeNotifications)
                    bleServiceData.setAngularVelocityChangedChannel(
                        angularVelocityChangeNotifications
                    )
                    bleServiceData.setMotionVectorChangedChannel(motionVectorChangeNotifications)
                    bleServiceData.setBodyTranslationChangedChannel(
                        bodyTranslationChangeNotifications
                    )
                    bleServiceData.setBodyRotationChangedChannel(bodyRotationChangeNotifications)
                    bleServiceData.setLegRadiusChangedChannel(legRadiusChangeNotifications)
                    bleServiceData.setBatteryUpdateIntervalMsChangedChannel(
                        batteryUpdateIntervalMsChangeNotifications
                    )
                    bleServiceData.setGetCalibrationResultChangedChannel(
                        getCalibrationDatumChangeNotifications
                    )
                    bleServiceData.setSetCalibrationResultChangedChannel(
                        setCalibrationResultChangeNotifications
                    )
                    connectedViewModel.setBleServiceData(bleServiceData)
                    defaultScope.launch {
                        try {
                            connectedViewModel.deviceAddress?.let {
                                bleServiceData.sync(it, true)
                                connectedViewModel.batteryLevel = bleServiceData.getBatteryLevel(it)
                                connectedViewModel.stateMachine = bleServiceData.getStateMachine(it)
                                connectedViewModel.animationFactor = bleServiceData.getAnimationFactor(it)
                                connectedViewModel.angularVelocity =
                                    bleServiceData.getAngularVelocity(it)
                                connectedViewModel.motionVector = bleServiceData.getMotionVector(it)
                                connectedViewModel.bodyTranslation =
                                    bleServiceData.getBodyTranslation(it)
                                connectedViewModel.bodyRotation = bleServiceData.getBodyRotation(it)
                                connectedViewModel.legRadius = bleServiceData.getLegRadius(it)
                                connectedViewModel.batteryUpdateIntervalMs =
                                    bleServiceData.getBatteryUpdateIntervalMs(it)

                                // sync calibration data?
                                connectedViewModel.calibratingViewModel.let {
                                    cvm ->
                                    bleServiceData.requestCalibrationData(it,
                                        cvm.leg.intValue.toUByte(),
                                        cvm.joint.intValue.toUByte(),
                                        cvm.kind.intValue.toUByte(),
                                    )
                                    // TODO: show loading indicator to signify request in progress?
                                }
                            }
                        } catch (e: Exception) {
                            Log.e("Connected", "Failed to sync: $e")
                        }
                    }
                }
            }
        }

        override fun onServiceDisconnected(name: ComponentName?) {
            if (BuildConfig.DEBUG && BleService::class.java.name != name?.className) {
                error("Disconnected from unknown service")
            } else {
                bleServiceData = null
                dialogViewModel.onDisconnected()
                finish()
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val name = intent.getStringExtra(DEVICE_NAME_EXTRA)
        val address = intent.getStringExtra(DEVICE_ADDRESS_EXTRA)
        connectedViewModel.deviceName = name
        connectedViewModel.deviceAddress = address

        bleServiceData?.let {
            connectedViewModel.setBleServiceData(it)
        }

        defaultScope.launch {
            for (update in batteryChangeNotifications) {
                mainHandler.run {
                    connectedViewModel.batteryLevel = update
                }
            }
        }
        defaultScope.launch {
            for (update in stateMachineChangeNotifications) {
                mainHandler.run {
                    connectedViewModel.stateMachine = update
                }
            }
        }
        defaultScope.launch {
            for (update in animationFactorChangeNotifications) {
                mainHandler.run {
                    connectedViewModel.animationFactor = update
                }
            }
        }
        defaultScope.launch {
            for (update in angularVelocityChangeNotifications) {
                mainHandler.run {
                    connectedViewModel.angularVelocity = update
                }
            }
        }
        defaultScope.launch {
            for (update in motionVectorChangeNotifications) {
                mainHandler.run {
                    connectedViewModel.motionVector = update
                }
            }
        }
        defaultScope.launch {
            for (update in bodyTranslationChangeNotifications) {
                mainHandler.run {
                    connectedViewModel.bodyTranslation = update
                }
            }
        }
        defaultScope.launch {
            for (update in bodyRotationChangeNotifications) {
                mainHandler.run {
                    connectedViewModel.bodyRotation = update
                }
            }
        }
        defaultScope.launch {
            for (update in legRadiusChangeNotifications) {
                mainHandler.run {
                    connectedViewModel.legRadius = update
                }
            }
        }
        defaultScope.launch {
            for (update in batteryUpdateIntervalMsChangeNotifications) {
                mainHandler.run {
                    connectedViewModel.batteryUpdateIntervalMs = update
                }
            }
        }
        defaultScope.launch {
            for (update in getCalibrationDatumChangeNotifications) {
                mainHandler.run {
                    // TODO: Sync results to storage
                    // TODO: turn off any loading indicator
                    Log.i("Connected", "getCalibrationDatumChangeNotifications: $update")
                    connectedViewModel.calibratingViewModel.pulse.floatValue = update.pulse
                }
            }
        }
        defaultScope.launch {
            for (update in setCalibrationResultChangeNotifications) {
                mainHandler.run {
                    // TODO: Sync results to storage?
                    // TODO: turn off any loading indicator
                    // TODO: Show error?
                    Log.i("Connected", "setCalibrationResultChangeNotifications: $update")
                }
            }
        }

        setContent {
            val coroutineScope = rememberCoroutineScope()
            MechaFerrisTheme {
                ConnectedView(
                    coroutineScope, dialogViewModel.showDialog,
                    onDisconnectedConfirmed = {
                        dialogViewModel.onDismissed()
                        finish()
                    },
                    connectedViewModel
                )
            }
        }
    }

    override fun onStart() {
        super.onStart()
        val latestServiceConn = MFServiceConn()
        if (bindService(
                Intent(BleService.DATA_PLANE_ACTION, null, this, BleService::class.java),
                latestServiceConn,
                0
            )
        ) {
            bleServiceConn = latestServiceConn
        }
    }

    override fun onStop() {
        super.onStop()

        if (bleServiceConn != null) {
            unbindService(bleServiceConn!!)
            bleServiceConn = null
        }
    }
}

@SuppressLint("UnusedMaterial3ScaffoldPaddingParameter")
@Composable
fun ConnectedView(
    coroutineScope: CoroutineScope,
    showDisconnected: Boolean,
    onDisconnectedConfirmed: () -> Unit,
    viewModel: ConnectedViewModel
) {
    var expanded by rememberSaveable { mutableStateOf(false) }
    val context = LocalContext.current
    if (showDisconnected) {
        AlertDialog(
            confirmButton = {
                Button(onClick = onDisconnectedConfirmed) {
                    Text(text = "OK")
                }
            },
            text = {
                Text(text = "Disconnected from MechaFerris")
            },
            title = {
                Text(text = "Disconnected")
            },
            onDismissRequest = {}
        )
    }
    Scaffold(
        modifier = Modifier
            .fillMaxWidth()
            .fillMaxHeight(),
        topBar = {
            Row(
                horizontalArrangement = Arrangement.Center,
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(vertical = 12.dp)
            ) {
                when (viewModel.scaffoldState) {
                    ScaffoldState.Home -> {
                        Text(text = "Home")
                    }

                    ScaffoldState.Calibrating -> {
                        Text(text = "Calibrate")
                    }

                    ScaffoldState.Controls -> {
                        Text(text = "Controls")
                    }
                }
            }
        },
        bottomBar = {
            Row(
                Modifier
                    .fillMaxWidth()
                    .padding(vertical = 12.dp)
                    .background(color = MaterialTheme.colorScheme.primaryContainer),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Surface(
                    onClick = {
                        coroutineScope.launch {
                            viewModel.onClickAction(
                                context,
                                ScaffoldState.Home
                            )
                        }
                    },
                    modifier = Modifier
                        .padding(start = 12.dp)
                        .background(color = Color.Transparent)
                ) {
                    Text(text = "Home", color = MaterialTheme.colorScheme.onPrimaryContainer)
                }
                Spacer(Modifier.weight(1f))
                Surface(onClick = {
                    coroutineScope.launch {
                        viewModel.onClickAction(
                            context,
                            ScaffoldState.Calibrating
                        )
                    }
                }) {
                    Text(text = "Calibrate")
                }
                Spacer(Modifier.weight(1f))
                Surface(
                    onClick = {
                        coroutineScope.launch {
                            viewModel.onClickAction(
                                context,
                                ScaffoldState.Controls
                            )
                        }
                    },
                    modifier = Modifier.padding(end = 12.dp)
                ) {
                    Text(text = "Controls")
                }
            }
        },
        floatingActionButton = {
            // Floating action button
            Column {
                if (expanded) {
                    CircleButton(onClick = {
                        coroutineScope.launch {
                            viewModel.setStateMachine(
                                context,
                                StateMachine.EXPLORING
                            )
                            viewModel.sync(context)
                        }
                    }) {
                        Icon(
                            painterResource(id = R.drawable.baseline_videogame_asset_24),
                            contentDescription = "control robot"
                        )
                    }
                    Spacer(Modifier.size(12.dp))
                    CircleButton(onClick = {
                        coroutineScope.launch {
                            viewModel.setStateMachine(
                                context,
                                StateMachine.CALIBRATING
                            )
                            viewModel.sync(context)
                        }
                    }) {
                        Icon(
                            painterResource(id = R.drawable.baseline_architecture_24),
                            contentDescription = "calibrate robot"
                        )
                    }
                    Spacer(Modifier.size(12.dp))
                    CircleButton(onClick = {
                        coroutineScope.launch {
                            viewModel.onPlayPause(
                                context
                            )
                            viewModel.sync(context)
                        }
                    }) {
                        PlayPauseIcon(stateMachine = viewModel.stateMachine)
                    }
                    Spacer(Modifier.size(12.dp))
                    CircleButton(onClick = {
                        coroutineScope.launch {
                            viewModel.setStateMachine(
                                context,
                                StateMachine.HOMING
                            )
                            viewModel.sync(context)
                        }
                    }) {
                        Icon(Icons.Filled.Home, contentDescription = "home robot")
                    }
                    Spacer(Modifier.size(12.dp))
                }
                CircleButton(onClick = { expanded = !expanded }) {
                    if (expanded) {
                        Icon(Icons.Filled.Close, contentDescription = "close more actions")
                    } else {
                        Icon(Icons.Filled.MoreVert, contentDescription = "show more actions")
                    }
                }
            }
        },
    ) { paddingValues ->
        Surface(
            Modifier
                .padding(paddingValues)
                .padding(horizontal = 12.dp)
        ) {
            when (viewModel.scaffoldState) {
                ScaffoldState.Home -> {
                    HomeView(state = viewModel)
                }

                ScaffoldState.Calibrating -> {
                    CalibratingView(viewModel.calibratingViewModel)
                }

                ScaffoldState.Controls -> {
                    ControlsView(viewModel)
                }
            }
        }
    }
}

@Composable
fun CircleButton(onClick: () -> Unit, content: @Composable RowScope.() -> Unit) {
    Button(
        onClick = onClick,
        contentPadding = PaddingValues(0.dp),
        modifier = Modifier.size(50.dp),
        content = content
    )
}

@Composable
fun PlayPauseIcon(stateMachine: StateMachine?) {
    when (stateMachine) {
        StateMachine.PAUSED -> Icon(
            Icons.Filled.PlayArrow, contentDescription = "re-enable robot"
        )

        StateMachine.HOMING -> Icon(
            painterResource(id = R.drawable.baseline_pause_24), contentDescription = "pause robot"
        )

        StateMachine.LOOPING -> Icon(
            painterResource(id = R.drawable.baseline_pause_24), contentDescription = "pause robot"
        )

        StateMachine.EXPLORING -> Icon(
            painterResource(id = R.drawable.baseline_pause_24), contentDescription = "pause robot"
        )

        StateMachine.CALIBRATING -> Icon(
            painterResource(id = R.drawable.baseline_play_disabled_24),
            contentDescription = "pause robot"
        )

        null -> Icon(
            painterResource(id = R.drawable.baseline_play_disabled_24),
            contentDescription = "pause robot"
        )
    }
}

@Preview(showBackground = true)
@Composable
fun ConnectedViewHomePreview() {
    MechaFerrisTheme {
        val coroutineScope = rememberCoroutineScope()
        ConnectedView(coroutineScope,
            false,
            viewModel = ConnectedViewModel().apply { scaffoldState = ScaffoldState.Home },
            onDisconnectedConfirmed = {})
    }
}

@Preview(showBackground = true)
@Composable
fun ConnectedViewCalibratingPreview() {
    MechaFerrisTheme {
        val coroutineScope = rememberCoroutineScope()
        ConnectedView(coroutineScope,
            false,
            viewModel = ConnectedViewModel().apply {
                scaffoldState = ScaffoldState.Calibrating
            },
            onDisconnectedConfirmed = {})
    }
}

@Preview(showBackground = true)
@Composable
fun ConnectedViewControlsPreview() {
    MechaFerrisTheme {
        val coroutineScope = rememberCoroutineScope()
        ConnectedView(coroutineScope,
            false,
            viewModel = ConnectedViewModel().apply {
                scaffoldState = ScaffoldState.Controls
            },
            onDisconnectedConfirmed = {})
    }
}

@Preview(showBackground = true)
@Composable
fun DialogPreview() {
    MechaFerrisTheme {
        val coroutineScope = rememberCoroutineScope()
        ConnectedView(coroutineScope,
            true,
            viewModel = ConnectedViewModel().apply {
                scaffoldState = ScaffoldState.Home
            },
            onDisconnectedConfirmed = {})
    }
}