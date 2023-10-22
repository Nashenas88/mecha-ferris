package paulfaria.mechaferris.ui.calibrating

import android.content.Context
import android.content.res.Configuration
import android.os.Parcelable
import android.util.Log
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
import androidx.compose.runtime.MutableIntState
import androidx.compose.runtime.MutableState
import androidx.compose.runtime.derivedStateOf
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableIntStateOf
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
import kotlinx.coroutines.launch
import kotlinx.parcelize.Parcelize
import kotlinx.parcelize.RawValue
import paulfaria.mechaferris.data.calibrationDataStore
import paulfaria.mechaferris.ui.theme.MechaFerrisTheme

class CalibratingViewModel {
    companion object {
        val SHORT_KINDS = listOf("Min", "Mid", "Max")
        val LONG_KINDS = listOf("Home", "Min", "Mid", "Max")
    }

    val legListModel = SpinnerViewModel(
        options = listOf("1", "2", "3", "4", "5", "6"), label = "Leg"
    )
    val jointListModel = SpinnerViewModel(
        options = listOf("Coxa", "Femur", "Tibia"), label = "Joint"
    )
    val enabled = mutableStateOf(false)
    val pulse = mutableFloatStateOf(1500.0f)

    private val kindOptions by derivedStateOf {
        Log.i(
            "CalibratingViewModel",
            "Derived state of kindOptions: ${jointListModel.selected.intValue}"
        )
        when (jointListModel.selected.intValue) {
            // Coxa or Femur
            0, 1 -> SHORT_KINDS
            // Tibia
            2 -> LONG_KINDS
            else -> throw Exception("Invalid joint ${jointListModel.selected.intValue}")
        }
    }

    val kindListModel = SpinnerViewModel(
        label = "Calibration Kind", options = kindOptions
    )

    fun next(kind: SpinnerViewModel) {
        Log.i(
            "CalibratingViewModel",
            "Next ${kindListModel.options}, vs $kindOptions, vs ${kind.options}"
        )
        if (kindListModel.selected.intValue == kindListModel.options.size - 1) {
            if (jointListModel.selected.intValue == jointListModel.options.size - 1) {
                jointListModel.selected.intValue = 0
                if (legListModel.selected.intValue == legListModel.options.size - 1) {
                    legListModel.selected.intValue = 0
                } else {
                    legListModel.selected.intValue += 1
                }
            } else {
                jointListModel.selected.intValue += 1
            }
            kindListModel.selected.intValue = 0
        } else {
            kindListModel.selected.intValue += 1
        }
        Log.i(
            "CalibratingViewModel",
            "Indices: kind ${kindListModel.selected.intValue}, joint ${jointListModel.selected.intValue}, leg ${legListModel.selected.intValue}"
        )
    }

    fun prev() {
        if (kindListModel.selected.intValue == 0) {
            if (jointListModel.selected.intValue == 0) {
                jointListModel.selected.intValue = jointListModel.options.size - 1
                if (legListModel.selected.intValue == 0) {
                    legListModel.selected.intValue = legListModel.options.size - 1
                } else {
                    legListModel.selected.intValue -= 1
                }
            } else {
                jointListModel.selected.intValue -= 1
            }
            kindListModel.selected.intValue = kindListModel.options.size - 1
        } else {
            kindListModel.selected.intValue -= 1
        }
    }
}

@Parcelize
class SpinnerViewModel(
    val options: List<String>,
    val label: String,
    val selected: @RawValue MutableIntState = mutableIntStateOf(0),
    val expanded: @RawValue MutableState<Boolean> = mutableStateOf(false)
) : Parcelable

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun Spinner(
    viewModel: SpinnerViewModel,
    onSelected: () -> Unit = {},
) {
    var expanded by remember { viewModel.expanded }
    var selected by remember { viewModel.selected }
    ExposedDropdownMenuBox(expanded = expanded, onExpandedChange = { expanded = !expanded }) {
        TextField(
            readOnly = true,
            value = viewModel.options[selected],
            onValueChange = {},
            label = { Text(text = viewModel.label) },
            trailingIcon = {
                ExposedDropdownMenuDefaults.TrailingIcon(expanded = expanded)
            },
            colors = ExposedDropdownMenuDefaults.textFieldColors(),
            modifier = Modifier
                .fillMaxWidth()
                .menuAnchor()
        )
        ExposedDropdownMenu(expanded = expanded, onDismissRequest = { expanded = false }) {
            viewModel.options.forEachIndexed { index, _ ->
                DropdownMenuItem(text = { Text(text = viewModel.options[index]) }, onClick = {
                    selected = index
                    expanded = false
                    onSelected()
                })
            }
        }
    }
}

fun Float.format(digits: Int) = "%07.${digits}f".format(this)

@Composable
fun CalibratingView(context: Context, viewModel: CalibratingViewModel) {
    val coroutineScope = rememberCoroutineScope()
    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(12.dp)
    ) {
        Log.i("CalibratingView", "CalibratingView recomposing")
        var pulse by rememberSaveable {
            viewModel.pulse
        }
        var enabled by rememberSaveable {
            viewModel.enabled
        }
        val kinds = remember {
            viewModel.kindListModel
        }
        Log.i(
            "CalibratingView",
            "CalibratingView recomposing: [${viewModel.jointListModel.selected.intValue}] ${kinds.options}"
        )
        Spinner(viewModel.legListModel)
        Spacer(modifier = Modifier.height(12.dp))
        Spinner(viewModel.jointListModel)
        Spacer(modifier = Modifier.height(12.dp))
        Spinner(kinds)
        Row(verticalAlignment = Alignment.CenterVertically) {
            Text(
                text = "Pulse: ${pulse.format(2)}",
                textAlign = TextAlign.Center,
                color = MaterialTheme.colorScheme.primary
            )
            Spacer(modifier = Modifier.width(12.dp))
            Slider(value = pulse, onValueChange = { pulse = it }, valueRange = 400.0f..2700.0f)
        }
        Row(
            horizontalArrangement = Arrangement.Start,
            verticalAlignment = Alignment.CenterVertically,
            modifier = Modifier.fillMaxWidth()
        ) {
            Checkbox(checked = enabled, onCheckedChange = { new ->
                Log.i("CalibratingView", "Enabled: $new")
                // TODO this doesn't work
                enabled = new
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
                viewModel.next(kinds)
            }) {
                Text(text = "Next")
            }
        }
        Spacer(modifier = Modifier.weight(1f))
        Row(horizontalArrangement = Arrangement.Center, modifier = Modifier.fillMaxWidth()) {
            Button(
                modifier = Modifier.weight(0.75f),
                onClick = {

                },
                colors = ButtonDefaults.buttonColors(containerColor = MaterialTheme.colorScheme.primary)
            ) {
                Text(text = "Save", color = MaterialTheme.colorScheme.onPrimary)
            }
            Spacer(modifier = Modifier.width(12.dp))
            Button(
                modifier = Modifier.weight(0.75f),
                onClick = {
                    coroutineScope.launch {
                        context.calibrationDataStore.data.collect {
                            Log.i("CalibratingView", "Calibration data: $it")
                        }
                    }
                },
                colors = ButtonDefaults.buttonColors(containerColor = MaterialTheme.colorScheme.secondary)
            ) {
                Text(text = "Load", color = MaterialTheme.colorScheme.onSecondary)
            }
        }
    }
}

@Preview(showBackground = true)
@Preview(showBackground = true, uiMode = Configuration.UI_MODE_NIGHT_YES)
@Composable
fun CalibratingViewPreview() {
    MechaFerrisTheme {
        val context = LocalContext.current
        CalibratingView(context, CalibratingViewModel())
    }
}

@Preview(showBackground = true)
@Composable
fun DropdownViewPreview() {
    val selected = remember { mutableIntStateOf(1) }
    val expanded = remember { mutableStateOf(true) }
    val viewModel = SpinnerViewModel(
        options = listOf("Coxa", "Femur", "Tibia"),
        label = "Joint",
        selected = selected,
        expanded = expanded
    )

    Box(modifier = Modifier.padding(12.dp)) {
        Spinner(
            viewModel
        )
    }
}