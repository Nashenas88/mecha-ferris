package paulfaria.mechaferris.ui.calibrating

import android.content.Context
import android.content.res.Configuration
import android.util.Log
import android.widget.Toast
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Checkbox
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.ExposedDropdownMenuBox
import androidx.compose.material3.ExposedDropdownMenuDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Slider
import androidx.compose.material3.Text
import androidx.compose.material3.TextField
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.derivedStateOf
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.lifecycle.ViewModel
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.emptyFlow
import kotlinx.coroutines.launch
import paulfaria.mechaferris.CalibrationDatum
import paulfaria.mechaferris.SetRes
import paulfaria.mechaferris.SetResult
import paulfaria.mechaferris.calibration.Joint
import paulfaria.mechaferris.calibration.ServoCalibration
import paulfaria.mechaferris.data.calibrationDataStore
import paulfaria.mechaferris.ui.theme.MechaFerrisTheme

@OptIn(ExperimentalUnsignedTypes::class)
class CalibratingViewModel @OptIn(ExperimentalUnsignedTypes::class) constructor(
    private val getCalibrationFlow: Flow<CalibrationDatum>,
    private val onRequestCalibration: suspend (Context, UByte, UByte, UByte) -> Unit,
    private val setCalibrationFlow: Flow<SetResult>,
    val onSetCalibration: suspend (Context, UByte, UByte, UByte, Float) -> Float?,
) : ViewModel() {
    companion object {
        val LEG_OPTIONS = listOf("1", "2", "3", "4", "5", "6")
        val JOINT_OPTIONS = listOf("Coxa", "Femur", "Tibia")
        val SHORT_KIND_OPTIONS = listOf("Min", "Mid", "Max")
        val LONG_KIND_OPTIONS = listOf("Home", "Min", "Mid", "Max")
    }

    val leg = mutableIntStateOf(0)
    val joint = mutableIntStateOf(0)
    val kind = mutableIntStateOf(0)
    val enabled = mutableStateOf(false)
    val pulse = mutableFloatStateOf(1500.0f)

    val calibrations = mutableStateListOf<Float?>()

    init {
        calibrations.addAll(List(60) { null })
    }

    fun onLaunch(context: Context) {
        Log.i("CalibratingViewModel", "Launching")
        viewModelScope.launch {
            getCalibrationFlow.collect {
                Log.i(
                    "CalibratingViewModel",
                    "Received calibration for (${it.index.leg}, ${it.index.joint}, ${it.index.kind}): ${it.pulse}"
                )
                val index =
                    it.index.leg.toByte() * 6 + it.index.joint.toByte() * 3 + it.index.kind.toByte()
                calibrations[index] = it.pulse
            }
        }.invokeOnCompletion { cause ->
            Log.i("CalibratingViewModel", "Completed get watch with $cause") }
        viewModelScope.launch {
            setCalibrationFlow.collect {
                if (it.result != SetRes.Ok) {
                    Log.e(
                        "CalibratingViewModel",
                        "Failed to set calibration for (${it.index.leg}, ${it.index.joint}, ${it.index.kind})"
                    )
                    Toast.makeText(
                        context,
                        "Failed to set calibration for (${it.index.leg}, ${it.index.joint}, ${it.index.kind})",
                        Toast.LENGTH_SHORT
                    ).show()
                }
            }
        }.invokeOnCompletion { cause ->
            Log.i("CalibratingViewModel", "Completed set watch with $cause") }
        viewModelScope.launch {
            Log.i("CalibratingViewModel", "Requesting all calibrations")
            for (leg in (0..<6)) {
                for (joint in (0..<3)) {
                    val maxKind = if (joint == 2) 4 else 3
                    for (kind in (0..<maxKind)) {
                        onRequestCalibration(
                            context,
                            leg.toUByte(),
                            joint.toUByte(),
                            kind.toUByte()
                        )
                    }
                }
            }
        }.invokeOnCompletion { cause ->
            Log.i("CalibratingViewModel", "Completed all gets with $cause") }
    }

    fun kindOptions(): List<String> {
        return when (joint.intValue) {
            // Coxa or Femur
            0, 1 -> SHORT_KIND_OPTIONS
            // Tibia
            2 -> LONG_KIND_OPTIONS
            else -> throw Exception("Invalid joint ${joint.intValue}")
        }
    }

    fun next() {
        if (kind.intValue == kindOptions().size - 1) {
            if (joint.intValue == JOINT_OPTIONS.size - 1) {
                joint.intValue = 0
                if (leg.intValue == LEG_OPTIONS.size - 1) {
                    leg.intValue = 0
                } else {
                    leg.intValue += 1
                }
            } else {
                joint.intValue += 1
            }
            kind.intValue = 0
        } else {
            kind.intValue += 1
        }
    }

    fun prev() {
        if (kind.intValue == 0) {
            if (joint.intValue == 0) {
                joint.intValue = JOINT_OPTIONS.size - 1
                if (leg.intValue == 0) {
                    leg.intValue = LEG_OPTIONS.size - 1
                } else {
                    leg.intValue -= 1
                }
            } else {
                joint.intValue -= 1
            }
            kind.intValue = kindOptions().size - 1
        } else {
            kind.intValue -= 1
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun Spinner(
    label: String,
    options: List<String>,
    selected: Int,
    onSelected: (Int) -> Unit,
) {
    var expanded by remember { mutableStateOf(false) }
    ExposedDropdownMenuBox(expanded = expanded, onExpandedChange = { expanded = !expanded }) {
        TextField(
            readOnly = true,
            value = options[selected],
            onValueChange = {},
            label = { Text(text = label) },
            trailingIcon = {
                ExposedDropdownMenuDefaults.TrailingIcon(expanded = expanded)
            },
            colors = ExposedDropdownMenuDefaults.textFieldColors(),
            modifier = Modifier
                .fillMaxWidth()
                .menuAnchor()
        )
        ExposedDropdownMenu(expanded = expanded, onDismissRequest = { expanded = false }) {
            options.forEachIndexed { index, _ ->
                DropdownMenuItem(text = { Text(text = options[index]) }, onClick = {
                    expanded = false
                    onSelected(index)
                })
            }
        }
    }
}

fun Float.format(digits: Int) = "%07.${digits}f".format(this)

@Composable
fun CalibratingView(viewModel: CalibratingViewModel) {
    val coroutineScope = rememberCoroutineScope()
    val localContext = LocalContext.current

    LaunchedEffect(true) {
        viewModel.onLaunch(localContext)
    }

    val calibrationData = localContext.calibrationDataStore.data.collectAsStateWithLifecycle(null)
    val currentCalibrationData by rememberSaveable {
        // We want to snapshot the stored data and use it as the initial value. It should NOT
        // be derived!
        mutableStateOf(calibrationData.value)
    }
    val pulse = rememberSaveable {
        viewModel.pulse
    }
    var enabled by rememberSaveable {
        viewModel.enabled
    }
    val leg = rememberSaveable {
        viewModel.leg
    }
    val joint = rememberSaveable {
        viewModel.joint
    }
    val kind = rememberSaveable {
        viewModel.kind
    }
    val modelPulse by remember {
        derivedStateOf {
            val index =
                kind.intValue.toByte() + joint.intValue.toByte() * 3 + leg.intValue.toByte() * 10
            viewModel.calibrations[index]
        }
    }

    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(12.dp)
    ) {

        Spinner(options = CalibratingViewModel.LEG_OPTIONS, label = "Leg", selected = leg.intValue,
            onSelected = {
                leg.intValue = it
            })
        Spacer(modifier = Modifier.height(12.dp))
        Spinner(options = CalibratingViewModel.JOINT_OPTIONS,
            label = "Joint",
            selected = joint.intValue,
            onSelected = {
                // Need to correct for Tibia having a HOME position
                if (joint.intValue != Joint.TIBIA_VALUE && it == Joint.TIBIA_VALUE) {
                    kind.intValue = kind.intValue + 1
                } else if (joint.intValue == Joint.TIBIA_VALUE && it != Joint.TIBIA_VALUE) {
                    kind.intValue = maxOf(0, kind.intValue - 1)
                }
                joint.intValue = it
            })
        Spacer(modifier = Modifier.height(12.dp))
        Spinner(
            options = viewModel.kindOptions(), label = "Calibration Kind",
            selected = kind.intValue,
            onSelected = {
                kind.intValue = it
            }
        )
        Row(verticalAlignment = Alignment.CenterVertically) {
            Text(
                text = "Pulse: ${pulse.floatValue.format(2)}",
                textAlign = TextAlign.Center,
                color = MaterialTheme.colorScheme.primary
            )
            Spacer(modifier = Modifier.width(12.dp))
            Slider(
                value = pulse.floatValue, onValueChange = { pulse.floatValue = it },
                onValueChangeFinished = {
                    if (enabled) {
                        coroutineScope.launch {
                            viewModel.onSetCalibration(
                                localContext,
                                leg.intValue.toUByte(),
                                joint.intValue.toUByte(),
                                kind.intValue.toUByte(),
                                pulse.floatValue
                            )?.let {
                                viewModel.calibrations[kind.intValue + joint.intValue * 3 + leg.intValue * 10] =
                                    it
                            }
                        }
                    }
                },
                valueRange = 400.0f..2700.0f
            )
        }
        Row(verticalAlignment = Alignment.CenterVertically) {
            Text(
                text = "Live Pulse: ${(modelPulse ?: 1000.0f).format(2)}",
                textAlign = TextAlign.Center,
                color = MaterialTheme.colorScheme.primary
            )
            Spacer(modifier = Modifier.width(12.dp))
            Slider(
                value = modelPulse ?: 1000.0f,
                onValueChange = {},
                enabled = false,
                valueRange = 400.0f..2700.0f
            )
        }
        Row(
            horizontalArrangement = Arrangement.Start,
            verticalAlignment = Alignment.CenterVertically,
            modifier = Modifier.fillMaxWidth()
        ) {
            Checkbox(checked = enabled, onCheckedChange = { new ->
                enabled = new
                if (enabled && modelPulse != pulse.floatValue) {
                    coroutineScope.launch {
                        viewModel.onSetCalibration(
                            localContext,
                            leg.intValue.toUByte(),
                            joint.intValue.toUByte(),
                            kind.intValue.toUByte(),
                            pulse.floatValue
                        )?.let {
                            viewModel.calibrations[kind.intValue + joint.intValue * 3 + leg.intValue * 10] =
                                it
                        }
                    }
                }
            }, enabled = true)
            Spacer(modifier = Modifier.width(12.dp))
            Text(
                text = "Enabled", color = MaterialTheme.colorScheme.primary
            )
        }
        Row(horizontalArrangement = Arrangement.Center, modifier = Modifier.fillMaxWidth()) {
            Button(modifier = Modifier.weight(0.75f), onClick = {
                viewModel.prev()
            }) {
                Text(text = "Previous")
            }
            Spacer(modifier = Modifier.weight(1f))
            Button(modifier = Modifier.weight(0.75f), onClick = {
                viewModel.next()
            }) {
                Text(text = "Next")
            }
        }
        Spacer(modifier = Modifier.height(12.dp))
        Row(horizontalArrangement = Arrangement.Center, modifier = Modifier.fillMaxWidth()) {
            val context: Context = LocalContext.current
            Button(
                modifier = Modifier.weight(0.75f),
                onClick = {
                    coroutineScope.launch {
                        context.calibrationDataStore.updateData { calibrations ->
                            // 3 or 4 kinds per joint, can be calculated with joint*3 + kind because
                            // the first 2 joints have 3 elements. The last joint has 4, but that's
                            // not needed to find the offset for the kind. We do take it into account
                            // when looking up the offset based on the leg though: 3 + 3 + 4 = 10
                            // kinds per leg.
                            val index = kind.intValue + joint.intValue * 3 + leg.intValue * 10
                            Log.i(
                                "CalibratingView",
                                "Storing index $index from ${kind.intValue}, ${joint.intValue}, ${leg.intValue}"
                            )
                            calibrations.toBuilder().setCalibrations(
                                index,
                                ServoCalibration.newBuilder().setLeg(leg.intValue)
                                    .setJointValue(joint.intValue).setKindValue(kind.intValue)
                                    .setPulse(pulse.floatValue)
                                    .build()
                            ).build()
                        }
                        Log.i("CalibratingView", "Stored calibration?")
                    }
                },
                colors = ButtonDefaults.buttonColors(containerColor = MaterialTheme.colorScheme.primary)
            ) {
                Text(text = "Save", color = MaterialTheme.colorScheme.onPrimary)
            }
            Spacer(modifier = Modifier.weight(1f))
            Button(
                modifier = Modifier.weight(0.75f),
                onClick = {
                    val index = kind.intValue + joint.intValue * 3 + leg.intValue * 10
                    Log.i(
                        "CalibratingView",
                        "Loading index $index from ${kind.intValue}, ${joint.intValue}, ${leg.intValue}"
                    )
                    val calibration = calibrationData.value?.getCalibrations(index)
                    pulse.floatValue = calibration?.pulse ?: 1500.0f
                },
                colors = ButtonDefaults.buttonColors(containerColor = MaterialTheme.colorScheme.secondary)
            ) {
                Text(text = "Load", color = MaterialTheme.colorScheme.onSecondary)
            }
        }
    }
}

@OptIn(ExperimentalUnsignedTypes::class)
@Preview(showBackground = true)
@Preview(showBackground = true, uiMode = Configuration.UI_MODE_NIGHT_YES)
@Composable
fun CalibratingViewPreview() {
    MechaFerrisTheme {
        CalibratingView(
            CalibratingViewModel(
                emptyFlow(),
                { _, _, _, _ -> },
                emptyFlow(),
                { _, _, _, _, _ -> null })
        )
    }
}

@Preview(showBackground = true)
@Composable
fun DropdownViewPreview() {
    val selected = remember { mutableIntStateOf(1) }
    Box(modifier = Modifier.padding(12.dp)) {
        Spinner(
            options = listOf("Coxa", "Femur", "Tibia"),
            label = "Joint",
            selected = selected.intValue,
            onSelected = {
                selected.intValue = it
            }
        )
    }
}
