@file:OptIn(ExperimentalUnsignedTypes::class)

package paulfaria.mechaferris

import android.content.Context
import android.util.Log
import android.widget.Toast
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.FlowPreview
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.emptyFlow
import kotlinx.coroutines.flow.flatMapConcat
import kotlinx.coroutines.flow.flowOf
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.merge
import kotlinx.coroutines.flow.receiveAsFlow
import kotlinx.coroutines.flow.stateIn
import paulfaria.mechaferris.ui.calibrating.CalibratingViewModel

@OptIn(FlowPreview::class)
fun <T, U> Flow<T?>.flatMapNullable(transform: (T) -> Flow<U>): Flow<U?> {
    return this.flatMapConcat { it?.let { transform(it) } ?: flowOf(null) }
}

@OptIn(ExperimentalCoroutinesApi::class)
class ConnectedViewModel(bleServiceData: Flow<BleService.BluetoothServiceBinder?> = emptyFlow()) :
    ViewModel() {
    var deviceName: String? = null
    var deviceAddress: String? = null

    private val bleServiceDataState =
        bleServiceData.stateIn(viewModelScope, SharingStarted.WhileSubscribed(), null)

    val batteryLevelFlow: StateFlow<UInt?> = bleServiceData.flatMapNullable { it.batteryFlow }
        .stateIn(viewModelScope, SharingStarted.WhileSubscribed(), null)

    private val _stateMachineChannel = Channel<StateMachine?>()
    var stateMachineFlow: StateFlow<StateMachine?> = merge(_stateMachineChannel.receiveAsFlow(),
        bleServiceData.flatMapNullable { it.stateMachineFlow }).stateIn(
        viewModelScope, SharingStarted.Lazily, null
    )
    private var previousStateMachine: StateMachine? = null

    private val _animationFactorChannel = Channel<Float?>()
    var animationFactorFlow: StateFlow<Float?> =
        merge(_animationFactorChannel.receiveAsFlow(), bleServiceData.flatMapNullable {
            it.animationFactorFlow
        }).stateIn(viewModelScope, SharingStarted.Lazily, null)

    private val _angularVelocityChannel = Channel<Float?>()
    var angularVelocityFlow: StateFlow<Float?> =
        merge(_angularVelocityChannel.receiveAsFlow(), bleServiceData.flatMapNullable {
            it.angularVelocityFlow
        }).stateIn(viewModelScope, SharingStarted.Lazily, null)

    private val _motionVectorChannel = Channel<Vector3?>()
    var motionVectorFlow: StateFlow<Vector3?> =
        merge(_motionVectorChannel.receiveAsFlow(), bleServiceData.flatMapNullable {
            it.motionVectorFlow
        }).stateIn(viewModelScope, SharingStarted.Lazily, null)

    private val _bodyTranslationChannel = Channel<Translation?>()
    var bodyTranslationFlow: StateFlow<Translation?> =
        merge(_bodyTranslationChannel.receiveAsFlow(), bleServiceData.flatMapNullable {
            it.bodyTranslationFlow
        }).stateIn(viewModelScope, SharingStarted.Lazily, null)

    private val _bodyRotationChannel = Channel<Quaternion?>()
    var bodyRotationFlow: StateFlow<Quaternion?> = merge(_bodyRotationChannel.receiveAsFlow(),
        bleServiceData.flatMapNullable { it.bodyRotationFlow }).stateIn(
        viewModelScope, SharingStarted.Lazily, null
    )

    private val _legRadiusChannel = Channel<Float?>()
    var legRadiusFlow: StateFlow<Float?> = merge(_legRadiusChannel.receiveAsFlow(),
        bleServiceData.flatMapNullable { it.legRadiusFlow }).stateIn(
        viewModelScope, SharingStarted.Lazily, null
    )

    private val _batteryUpdateIntervalMsChannel = Channel<UInt?>()
    var batteryUpdateIntervalMsFlow: StateFlow<UInt?> =
        merge(
            _batteryUpdateIntervalMsChannel.receiveAsFlow(),
            bleServiceData.flatMapNullable { it.batteryUpdateIntervalMsFlow }).stateIn(
            viewModelScope, SharingStarted.Lazily, null
        )

    var scaffoldState: ScaffoldState by mutableStateOf(ScaffoldState.Home)
    val calibratingViewModel = CalibratingViewModel()
    val height: StateFlow<Float> = bodyTranslationFlow.map {
        it?.y ?: 75.0f
    }.stateIn(viewModelScope, SharingStarted.WhileSubscribed(), 75.0f)

    // for (update in getCalibrationDatumChangeNotifications) {
    //     mainHandler.run {
    //         // TODO: Sync results to storage
    //         // TODO: turn off any loading indicator
    //         Log.i("Connected", "getCalibrationDatumChangeNotifications: $update")
    //         connectedViewModel.calibratingViewModel.pulse.floatValue = update.pulse
    //     }
    // }
    // for (update in setCalibrationResultChangeNotifications) {
    //     mainHandler.run {
    //         // TODO: Sync results to storage?
    //         // TODO: turn off any loading indicator
    //         // TODO: Show error?
    //         Log.i("Connected", "setCalibrationResultChangeNotifications: $update")
    //     }
    // }

    suspend fun onClickAction(context: Context, scaffoldState: ScaffoldState) {
        when (scaffoldState) {
            ScaffoldState.Home -> {
                this.scaffoldState = ScaffoldState.Home
                Log.i("ConnectedViewModel", "Home onClickAction: ${this.scaffoldState}")
            }

            ScaffoldState.Calibrating -> {
                setStateMachine(context, StateMachine.CALIBRATING) {
                    this.scaffoldState = ScaffoldState.Calibrating
                    Log.i("ConnectedViewModel", "Calibrating onClickAction: ${this.scaffoldState}")
                }
            }

            ScaffoldState.Controls -> {
                setStateMachine(context, StateMachine.EXPLORING) {
                    this.scaffoldState = ScaffoldState.Controls
                    Log.i("ConnectedViewModel", "Controls onClickAction: ${this.scaffoldState}")
                }
            }
        }
    }

    suspend fun setStateMachine(
        context: Context,
        stateMachine: StateMachine,
        onUpdate: () -> Unit = {},
    ) {
        try {
            if (deviceAddress?.let {
                    bleServiceDataState.value?.setStateMachine(
                        it, stateMachine
                    )
                } != null) {
                onUpdate()
                _stateMachineChannel.send(stateMachine)
            }
        } catch (e: Exception) {
            Toast.makeText(context, "Failed to set new state: ${e.message}", Toast.LENGTH_LONG)
                .show()
        }
    }

    suspend fun sync(context: Context, all: Boolean = false) {
        try {
            if (deviceAddress?.let { bleServiceDataState.value?.sync(it, all) } != null) {
                Toast.makeText(context, "Synced", Toast.LENGTH_LONG).show()
            }
        } catch (e: Exception) {
            Toast.makeText(context, "Failed to sync: ${e.message}", Toast.LENGTH_LONG).show()
        }
    }

    suspend fun setAnimationFactor(
        context: Context, animationFactor: Float
    ) {
        try {
            if (deviceAddress?.let {
                    bleServiceDataState.value?.setAnimationFactor(
                        it, animationFactor
                    )
                } != null) {
                _animationFactorChannel.send(animationFactor)
            }
        } catch (e: Exception) {
            Toast.makeText(
                context, "Failed to set new animation factor: ${e.message}", Toast.LENGTH_LONG
            ).show()
        }
    }

    suspend fun setAngularVelocity(
        context: Context, angularVelocity: Float
    ) {
        try {
            if (deviceAddress?.let {
                    bleServiceDataState.value?.setAngularVelocity(
                        it, angularVelocity
                    )
                } != null) {
                _angularVelocityChannel.send(angularVelocity)
            }
        } catch (e: Exception) {
            Toast.makeText(
                context, "Failed to set new angular velocity: ${e.message}", Toast.LENGTH_LONG
            ).show()
        }
    }

    suspend fun setMotionVector(
        context: Context, motionVector: Vector3
    ) {
        try {
            if (deviceAddress?.let {
                    bleServiceDataState.value?.setMotionVector(
                        it, motionVector
                    )
                } != null) {
                _motionVectorChannel.send(motionVector)
            }
        } catch (e: Exception) {
            Toast.makeText(
                context, "Failed to set new motion vector: ${e.message}", Toast.LENGTH_LONG
            ).show()
        }
    }

    suspend fun setBodyTranslation(
        context: Context, bodyTranslation: Translation
    ) {
        try {
            if (deviceAddress?.let {
                    bleServiceDataState.value?.setBodyTranslation(
                        it, bodyTranslation
                    )
                } != null) {
                _bodyTranslationChannel.send(bodyTranslation)
            }
        } catch (e: Exception) {
            Toast.makeText(
                context, "Failed to set new body translation: ${e.message}", Toast.LENGTH_LONG
            ).show()
        }
    }

    suspend fun setBodyRotation(
        context: Context, bodyRotation: Quaternion
    ) {
        try {
            if (deviceAddress?.let {
                    bleServiceDataState.value?.setBodyRotation(
                        it, bodyRotation
                    )
                } != null) {
                _bodyRotationChannel.send(bodyRotation)
            }
        } catch (e: Exception) {
            Toast.makeText(
                context, "Failed to set new body rotation: ${e.message}", Toast.LENGTH_LONG
            ).show()
        }
    }

    suspend fun setLegRadius(
        context: Context, legRadius: Float
    ) {
        try {
            if (deviceAddress?.let {
                    bleServiceDataState.value?.setLegRadius(
                        it, legRadius
                    )
                } != null) {
                _legRadiusChannel.send(legRadius)
            }
        } catch (e: Exception) {
            Toast.makeText(context, "Failed to set new leg radius: ${e.message}", Toast.LENGTH_LONG)
                .show()
        }
    }

    suspend fun setBatteryUpdateIntervalMs(
        context: Context, batteryUpdateIntervalMs: UInt
    ) {
        try {
            if (deviceAddress?.let {
                    bleServiceDataState.value?.setBatteryUpdateIntervalMs(
                        it, batteryUpdateIntervalMs
                    )
                } != null) {
                _batteryUpdateIntervalMsChannel.send(batteryUpdateIntervalMs)
                Toast.makeText(
                    context,
                    "Set new battery update interval: $batteryUpdateIntervalMs",
                    Toast.LENGTH_LONG
                ).show()
            }
        } catch (e: Exception) {
            Toast.makeText(
                context,
                "Failed to set new battery update interval: ${e.message}",
                Toast.LENGTH_LONG
            ).show()
        }
    }

    suspend fun onPlayPause(context: Context) {
        if (stateMachineFlow.value == StateMachine.PAUSED && previousStateMachine != null) {
            setStateMachine(context, previousStateMachine!!) {
                previousStateMachine = null
            }
        } else {
            setStateMachine(context, StateMachine.PAUSED) {
                previousStateMachine = stateMachineFlow.value
            }
        }
    }
}