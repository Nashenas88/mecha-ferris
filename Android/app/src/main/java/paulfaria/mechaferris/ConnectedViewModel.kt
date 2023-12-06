@file:OptIn(ExperimentalUnsignedTypes::class)

package paulfaria.mechaferris

import android.content.Context
import android.util.Log
import android.widget.Toast
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.lifecycle.LiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.asLiveData
import androidx.lifecycle.viewModelScope
import com.juul.kable.Bluetooth
import com.juul.kable.ConnectionLostException
import com.juul.kable.NotReadyException
import com.juul.kable.Peripheral
import com.juul.kable.State
import com.juul.kable.peripheral
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.ensureActive
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.catch
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.filter
import kotlinx.coroutines.flow.flatMapLatest
import kotlinx.coroutines.flow.flow
import kotlinx.coroutines.flow.flowOf
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.job
import kotlinx.coroutines.launch
import paulfaria.mechaferris.ui.calibrating.CalibratingViewModel
import kotlin.time.Duration.Companion.seconds

private val reconnectDelay = 1.seconds

sealed class ViewState {

    data object BluetoothUnavailable : ViewState()

    data object Connecting : ViewState()

    data class Connected(
        val rssi: Int,
        val batteryLevel: UInt,
        val stateMachine: StateMachine,
        val animationFactor: Float,
        val angularVelocity: Float,
        val motionVector: Vector3,
        val bodyTranslation: Translation,
        val bodyRotation: Quaternion,
        val legRadius: Float,
    ) : ViewState()

    data object Disconnecting : ViewState()

    data object Disconnected : ViewState()
}

val ViewState.label: String
    get() = when (this) {
        ViewState.BluetoothUnavailable -> "Bluetooth unavailable"
        ViewState.Connecting -> "Connecting"
        is ViewState.Connected -> "Connected"
        ViewState.Disconnecting -> "Disconnecting"
        ViewState.Disconnected -> "Disconnected"
    }

@OptIn(ExperimentalCoroutinesApi::class, ExperimentalUnsignedTypes::class)
class ConnectedViewModel(val deviceName: String?, val macAddress: String) :
    ViewModel() {
    private val autoConnect = MutableStateFlow(false)
    private val scope =
        CoroutineScope(peripheralScope.coroutineContext + Job(peripheralScope.coroutineContext.job))

    private val peripheral = scope.peripheral(macAddress) {
        autoConnectIf(autoConnect::value)
    }
    private val mechaFerris = MechaFerris(peripheral)
    private val state = combine(Bluetooth.availability, peripheral.state, ::Pair)
    private var previousState: StateMachine? = null

    val mutRequestedAnimationFactor = MutableStateFlow(1.0f)
    val requestedAnimationFactor: StateFlow<Float> = mutRequestedAnimationFactor

    init {
        viewModelScope.enableAutoReconnect()
    }

    private fun CoroutineScope.enableAutoReconnect() {
        state.filter { (bluetoothAvailability, connectionState) ->
            bluetoothAvailability == Bluetooth.Availability.Available && connectionState is State.Disconnected
        }.onEach {
            ensureActive()
            Log.i("ConnectedViewModel", "Waiting $reconnectDelay to reconnect...")
            delay(reconnectDelay)
            connect()
        }.launchIn(this)
    }

    private fun CoroutineScope.connect() {
        launch {
            Log.d("ConnectedViewModel", "Connecting")
            try {
                peripheral.connect()
                autoConnect.value = true
                Log.d("ConnectedViewModel DEBUG", "Services: ")
                if (peripheral.services != null) {
                    for (service in peripheral.services!!) {
                        Log.d("ConnectedViewModel DEBUG", "  ${service.serviceUuid}")
                        if (service.serviceUuid.toString() == controllerServiceUuid) {
                            Log.d("ConnectedViewModel DEBUG", "    Characteristics: ")
                            for (characteristic in service.characteristics) {
                                Log.d(
                                    "ConnectedViewModel DEBUG",
                                    "      ${characteristic.characteristicUuid}"
                                )
                            }
                        }
                    }
                }
                mechaFerris.notifyAll()
            } catch (e: ConnectionLostException) {
                autoConnect.value = false
                Log.w("ConnectedViewModel", "Connection attempt failed", e)
            }
        }
    }

    val viewState: Flow<ViewState> = state
        .flatMapLatest { (bluetoothAvailability, state) ->
            if (bluetoothAvailability is Bluetooth.Availability.Unavailable) {
                return@flatMapLatest flowOf(ViewState.BluetoothUnavailable)
            }
            when (state) {
                is State.Connecting -> flowOf(ViewState.Connecting)
                State.Connected -> combine(
                    peripheral.remoteRssi(),
                    mechaFerris.batteryLevel,
                    mechaFerris.stateMachine,
                    mechaFerris.animationFactor,
                    mechaFerris.angularVelocity,
                    mechaFerris.motionVector,
                    mechaFerris.bodyTranslation,
                    mechaFerris.bodyRotation,
                    mechaFerris.legRadius
                ) { args ->
                    val (rssi, batteryLevel, stateMachine, animationFactor, angularVelocity, motionVector, bodyTranslation, bodyRotation, legRadius) = args
                    ViewState.Connected(
                        rssi as Int,
                        batteryLevel as UInt,
                        stateMachine as StateMachine,
                        animationFactor as Float,
                        angularVelocity as Float,
                        motionVector as Vector3,
                        bodyTranslation as Translation,
                        bodyRotation as Quaternion,
                        legRadius as Float
                    )
                }

                State.Disconnecting -> flowOf(ViewState.Disconnecting)
                is State.Disconnected -> flowOf(ViewState.Disconnected)
            }
        }

    val batteryLevel = mechaFerris.batteryLevel
    val stateMachine = mechaFerris.stateMachine
    val animationFactor = mechaFerris.animationFactor
    val angularVelocity = mechaFerris.angularVelocity
    val motionVector = mechaFerris.motionVector
    val bodyTranslation = mechaFerris.bodyTranslation
    val bodyRotation = mechaFerris.bodyRotation
    val legRadius = mechaFerris.legRadius

    var scaffoldState: ScaffoldState by mutableStateOf(ScaffoldState.Home)
    val calibratingViewModel =
        CalibratingViewModel(mechaFerris.getCalibrationResult, { context, leg, joint, kind ->
            this@ConnectedViewModel.getCalibration(context, leg, joint, kind)
        }, mechaFerris.setCalibrationResult, { context, leg, joint, kind, pulse ->
            this@ConnectedViewModel.setCalibration(context, leg, joint, kind, pulse)
        })
    val height: StateFlow<Float> = viewState.map {
        when (it) {
            is ViewState.Connected -> it.bodyTranslation.y
            else -> 0.0f
        }
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

    @OptIn(ExperimentalUnsignedTypes::class)
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

    @OptIn(ExperimentalUnsignedTypes::class)
    suspend fun setStateMachine(
        context: Context,
        stateMachine: StateMachine,
        onUpdate: () -> Unit = {},
    ) {
        try {
            mechaFerris.setStateMachine(stateMachine)
            onUpdate()
        } catch (e: Exception) {
            Toast.makeText(context, "Failed to set new state: ${e.message}", Toast.LENGTH_LONG)
                .show()
        }
    }

    suspend fun getCalibration(context: Context, leg: UByte, joint: UByte, kind: UByte) {
        try {
            mechaFerris.requestCalibrationData(leg, joint, kind)
            mechaFerris.sync(false)
        } catch (e: Exception) {
            Toast.makeText(
                context,
                "Failed to get calibration result: ${e.message}",
                Toast.LENGTH_LONG
            )
                .show()
        }
    }

    suspend fun setCalibration(
        context: Context,
        leg: UByte,
        joint: UByte,
        kind: UByte,
        pulse: Float
    ): Float? {
        return try {
            Log.i("ConnectedViewModel", "setCalibration: $leg, $joint, $kind, $pulse")
            mechaFerris.setCalibrationPulse(leg, joint, kind, pulse)
            mechaFerris.sync(false)
            pulse
        } catch (e: Exception) {
            Toast.makeText(
                context,
                "Failed to set calibration result: ${e.message}",
                Toast.LENGTH_LONG
            )
                .show()
            null
        }
    }

    suspend fun sync(context: Context, all: Boolean = false) {
        try {
            mechaFerris.sync(all)
        } catch (e: Exception) {
            Toast.makeText(context, "Failed to sync: ${e.message}", Toast.LENGTH_LONG).show()
        }
    }

    suspend fun setAnimationFactor(
        context: Context, animationFactor: Float
    ) {
        try {
            mechaFerris.setAnimationFactor(animationFactor)
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
            mechaFerris.setAngularVelocity(angularVelocity)
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
            mechaFerris.setMotionVector(motionVector)
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
            mechaFerris.setBodyTranslation(bodyTranslation)
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
            mechaFerris.setBodyRotation(bodyRotation)
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
            mechaFerris.setLegRadius(legRadius)
        } catch (e: Exception) {
            Toast.makeText(context, "Failed to set new leg radius: ${e.message}", Toast.LENGTH_LONG)
                .show()
        }
    }

    suspend fun setBatteryUpdateIntervalMs(
        context: Context, batteryUpdateIntervalMs: UInt
    ) {
        try {
            mechaFerris.setBatteryUpdateIntervalMs(batteryUpdateIntervalMs)
        } catch (e: Exception) {
            Toast.makeText(
                context,
                "Failed to set new battery update interval: ${e.message}",
                Toast.LENGTH_LONG
            ).show()
        }
    }

    private val stateMachineLiveData: LiveData<StateMachine> =
        mechaFerris.stateMachine.asLiveData(peripheralScope.coroutineContext)

    @OptIn(ExperimentalUnsignedTypes::class)
    suspend fun onPlayPause(context: Context) {
        val stateMachine = stateMachineLiveData.value
        try {
            when (stateMachine) {
                StateMachine.PAUSED -> {
                    if (previousState != null) {
                        mechaFerris.setStateMachine(previousState!!)
                    } else {
                        mechaFerris.setStateMachine(StateMachine.LOOPING)
                    }
                    previousState = null
                }

                else -> {
                    previousState = stateMachine
                    mechaFerris.setStateMachine(StateMachine.PAUSED)
                }
            }
        } catch (exception: Exception) {
            Toast.makeText(
                context,
                "Failed to set new state: ${exception.message}",
                Toast.LENGTH_LONG
            )
                .show()
        }
    }
}

private operator fun <T : Any> Array<T>.component6(): Any {
    return this[5]
}

private operator fun <T : Any> Array<T>.component7(): Any {
    return this[6]
}

private operator fun <T : Any> Array<T>.component8(): Any {
    return this[7]
}

private operator fun <T : Any> Array<T>.component9(): Any {
    return this[8]
}

private fun Peripheral.remoteRssi() = flow {
    while (true) {
        val rssi = rssi()
        Log.d("ConnectedViewModel Peripheral", "RSSI: $rssi")
        emit(rssi)
        delay(1_000L)
    }
}.catch { cause ->
    if (cause !is NotReadyException) throw cause
}
