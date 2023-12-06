@file:OptIn(ExperimentalUnsignedTypes::class)

package paulfaria.mechaferris

import android.annotation.SuppressLint
import android.os.Bundle
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
import androidx.compose.ui.unit.dp
import androidx.lifecycle.ViewModel
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.initializer
import androidx.lifecycle.viewmodel.viewModelFactory
import com.juul.exercise.annotations.Exercise
import com.juul.exercise.annotations.Extra
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.launch
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

@Exercise(
    Extra("macAddress", String::class),
    Extra("deviceName", String::class, optional = true)
)
class Connected : ComponentActivity() {
    private val connectedViewModel by viewModels<ConnectedViewModel> {
        viewModelFactory {
            initializer {
                ConnectedViewModel(extras.deviceName, extras.macAddress)
            }
        }
    }

    private val dialogViewModel: DialogViewModel by viewModels()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        setContent {
            val coroutineScope = rememberCoroutineScope()
            val viewState by connectedViewModel.viewState.collectAsStateWithLifecycle(ViewState.Disconnected)
            MechaFerrisTheme {
                Text(viewState.label)
                Spacer(modifier = Modifier.size(12.dp))
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
}

@SuppressLint("UnusedMaterial3ScaffoldPaddingParameter")
@Composable
fun ConnectedView(
    coroutineScope: CoroutineScope,
    showDisconnected: Boolean,
    onDisconnectedConfirmed: () -> Unit,
    viewModel: ConnectedViewModel,
) {
    val deviceName = viewModel.deviceName
    val deviceAddress = viewModel.macAddress
    var expanded by rememberSaveable { mutableStateOf(false) }

    val batteryLevel by viewModel.batteryLevel.collectAsStateWithLifecycle(initialValue = null)
    val stateMachine by viewModel.stateMachine.collectAsStateWithLifecycle(initialValue = null)
    val animationFactor by viewModel.animationFactor.collectAsStateWithLifecycle(initialValue = null)
    val angularVelocity by viewModel.angularVelocity.collectAsStateWithLifecycle(initialValue = null)
    val motionVector by viewModel.motionVector.collectAsStateWithLifecycle(initialValue = null)
    val bodyTranslation by viewModel.bodyTranslation.collectAsStateWithLifecycle(initialValue = null)
    val bodyRotation by viewModel.bodyRotation.collectAsStateWithLifecycle(initialValue = null)
    val legRadius by viewModel.legRadius.collectAsStateWithLifecycle(initialValue = null)

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
                        PlayPauseIcon(stateMachine = stateMachine)
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
                    HomeView(
                        deviceName = deviceName,
                        deviceAddress = deviceAddress,
                        batteryLevel = batteryLevel,
                        stateMachine = stateMachine,
                        animationFactor = animationFactor,
                        angularVelocity = angularVelocity,
                        motionVector = motionVector,
                        bodyTranslation = bodyTranslation,
                        bodyRotation = bodyRotation,
                        legRadius = legRadius
                    )
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

//@Preview(showBackground = true)
//@Composable
//fun ConnectedViewHomePreview() {
//    MechaFerrisTheme {
//        val coroutineScope = rememberCoroutineScope()
//        ConnectedView(coroutineScope,
//            false,
//            viewModel = ConnectedViewModel("MechaFerris", "abc123").apply { scaffoldState = ScaffoldState.Home },
//            onDisconnectedConfirmed = {})
//    }
//}
//
//@Preview(showBackground = true)
//@Composable
//fun ConnectedViewCalibratingPreview() {
//    MechaFerrisTheme {
//        val coroutineScope = rememberCoroutineScope()
//        ConnectedView(coroutineScope,
//            false,
//            viewModel = ConnectedViewModel("MechaFerris", "abc123").apply {
//                scaffoldState = ScaffoldState.Calibrating
//            },
//            onDisconnectedConfirmed = {})
//    }
//}
//
//@Preview(showBackground = true)
//@Composable
//fun ConnectedViewControlsPreview() {
//    MechaFerrisTheme {
//        val coroutineScope = rememberCoroutineScope()
//        ConnectedView(coroutineScope,
//            false,
//            viewModel = ConnectedViewModel("MechaFerris", "abc123").apply {
//                scaffoldState = ScaffoldState.Controls
//            },
//            onDisconnectedConfirmed = {})
//    }
//}
//
//@Preview(showBackground = true)
//@Composable
//fun DialogPreview() {
//    MechaFerrisTheme {
//        val coroutineScope = rememberCoroutineScope()
//        ConnectedView(coroutineScope,
//            true,
//            viewModel = ConnectedViewModel("MechaFerris", "abc123").apply {
//                scaffoldState = ScaffoldState.Home
//            },
//            onDisconnectedConfirmed = {})
//    }
//}