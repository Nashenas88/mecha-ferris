package com.mechaferris.mechaferris

import android.content.Context
import android.util.Log
import android.widget.Toast
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.lifecycle.ViewModel
import com.mechaferris.mechaferris.ui.calibrating.CalibratingViewModel

class ConnectedViewModel : ViewModel() {
    private var bleServiceData: BleService.BluetoothServiceBinder? = null
    var deviceName: String? = null
    var deviceAddress: String? = null
    var batteryLevel: Int? by mutableStateOf(null)
    var awaitingBatteryLevel: Boolean = false
    var stateMachine: StateMachine? by mutableStateOf(null)
    var awaitingStateMachine: Boolean = false
    private var previousStateMachine: StateMachine? = null
    var animationFactor: Float? by mutableStateOf(null)
    var awaitingAnimationFactor: Boolean = false
    var angularVelocity: Float? by mutableStateOf(null)
    var awaitingAngularVelocity: Boolean = false
    var motionVector: Vector3? by mutableStateOf(null)
    var awaitingMotionVector: Boolean = false
    var bodyTranslation: Vector3? by mutableStateOf(null)
    var awaitingBodyTranslation: Boolean = false
    var bodyRotation: Quaternion? by mutableStateOf(null)
    var awaitingBodyRotation: Boolean = false
    var legRadius: Float? by mutableStateOf(null)
    var awaitingLegRadius: Boolean = false
    var batteryUpdateIntervalMs: Int? by mutableStateOf(null)
    var scaffoldState: ScaffoldState by mutableStateOf(ScaffoldState.Home)
    val calibratingViewModel = CalibratingViewModel()

    fun setBleServiceData(bleServiceData: BleService.BluetoothServiceBinder) {
        this.bleServiceData = bleServiceData
    }

    suspend fun onClickAction(context: Context, scaffold_state: ScaffoldState) {
        when (scaffold_state) {
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
            if (deviceAddress?.let { bleServiceData?.setStateMachine(it, stateMachine) } != null) {
                onUpdate()
                this.stateMachine = stateMachine
            }
        } catch (e: Exception) {
            Toast.makeText(context, "Failed to set new state: ${e.message}", Toast.LENGTH_LONG)
                .show()
        }
    }

    suspend fun sync(context: Context, all: Boolean = false) {
        try {
            if (deviceAddress?.let { bleServiceData?.sync(it, all) } != null) {
                Toast.makeText(context, "Synced", Toast.LENGTH_LONG)
                    .show()
            }
        } catch (e: Exception) {
            Toast.makeText(context, "Failed to sync: ${e.message}", Toast.LENGTH_LONG)
                .show()
        }
    }

    suspend fun setAnimationFactor(
        context: Context,
        animationFactor: Float
    ) {
        try {
            if (deviceAddress?.let { bleServiceData?.setAnimationFactor(it, animationFactor) } != null) {
                this.animationFactor = animationFactor
            }
        } catch (e: Exception) {
            Toast.makeText(context, "Failed to set new animation factor: ${e.message}", Toast.LENGTH_LONG)
                .show()
        }
    }

    suspend fun setAngularVelocity(
        context: Context,
        angularVelocity: Float
    ) {
        try {
            if (deviceAddress?.let {
                    bleServiceData?.setAngularVelocity(
                        it,
                        angularVelocity
                    )
                } != null) {
                this.angularVelocity = angularVelocity
            }
        } catch (e: Exception) {
            Toast.makeText(
                context,
                "Failed to set new angular velocity: ${e.message}",
                Toast.LENGTH_LONG
            )
                .show()
        }
    }

    suspend fun setMotionVector(
        context: Context,
        motionVector: Vector3
    ) {
        try {
            if (deviceAddress?.let { bleServiceData?.setMotionVector(it, motionVector) } != null) {
                this.motionVector = motionVector
            }
        } catch (e: Exception) {
            Toast.makeText(
                context,
                "Failed to set new motion vector: ${e.message}",
                Toast.LENGTH_LONG
            )
                .show()
        }
    }

    suspend fun setBodyTranslation(
        context: Context,
        bodyTranslation: Vector3
    ) {
        try {
            if (deviceAddress?.let {
                    bleServiceData?.setBodyTranslation(
                        it,
                        bodyTranslation
                    )
                } != null) {
                Log.i("CVM", "Body translation set to ${bodyTranslation.y}")
                this.bodyTranslation = bodyTranslation
                Log.i("CVM", "This Body translation set to ${this.bodyTranslation?.y}")

            }
        } catch (e: Exception) {
            Toast.makeText(
                context,
                "Failed to set new body translation: ${e.message}",
                Toast.LENGTH_LONG
            )
                .show()
        }
    }

    suspend fun setBodyRotation(
        context: Context,
        bodyRotation: Quaternion
    ) {
        try {
            if (deviceAddress?.let { bleServiceData?.setBodyRotation(it, bodyRotation) } != null) {
                this.bodyRotation = bodyRotation
            }
        } catch (e: Exception) {
            Toast.makeText(
                context,
                "Failed to set new body rotation: ${e.message}",
                Toast.LENGTH_LONG
            )
                .show()
        }
    }

    suspend fun setLegRadius(
        context: Context,
        legRadius: Float
    ) {
        try {
            if (deviceAddress?.let { bleServiceData?.setLegRadius(it, legRadius) } != null) {
                this.legRadius = legRadius
            }
        } catch (e: Exception) {
            Toast.makeText(context, "Failed to set new leg radius: ${e.message}", Toast.LENGTH_LONG)
                .show()
        }
    }

    suspend fun setBatteryUpdateIntervalMs(
        context: Context,
        batteryUpdateIntervalMs: Int
    ) {
        try {
            if (deviceAddress?.let {
                    bleServiceData?.setBatteryUpdateIntervalMs(
                        it,
                        batteryUpdateIntervalMs
                    )
                } != null) {
                this.batteryUpdateIntervalMs = batteryUpdateIntervalMs
            }
        } catch (e: Exception) {
            Toast.makeText(
                context,
                "Failed to set new battery update interval: ${e.message}",
                Toast.LENGTH_LONG
            )
                .show()
        }
    }

    suspend fun onPlayPause(context: Context) {
        if (stateMachine == StateMachine.PAUSED && previousStateMachine != null) {
            setStateMachine(context, previousStateMachine!!) {
                previousStateMachine = null
            }
        } else {
            setStateMachine(context, StateMachine.PAUSED) {
                previousStateMachine = stateMachine
            }
        }
    }
}