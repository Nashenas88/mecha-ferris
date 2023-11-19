package paulfaria.mechaferris.ui.controls

import androidx.compose.foundation.layout.Column
import androidx.compose.material3.Slider
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.ui.Alignment
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.tooling.preview.Preview
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import kotlinx.coroutines.launch
import paulfaria.mechaferris.ConnectedViewModel
import paulfaria.mechaferris.Translation

@Composable
fun ControlsView(viewModel: ConnectedViewModel) {
    val coroutineScope = rememberCoroutineScope()
    val deviceHeight by viewModel.height.collectAsStateWithLifecycle()
    val height = remember {
        mutableFloatStateOf(deviceHeight)
    }
    val deviceAnimationFactor by viewModel.animationFactorFlow.collectAsStateWithLifecycle()
    val animationFactor = remember {
        mutableFloatStateOf(deviceAnimationFactor ?: 1.0f)
    }

    val bodyTranslation by viewModel.bodyTranslationFlow.collectAsStateWithLifecycle()
    val context = LocalContext.current
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Text(text = "Requested Height: ${height.floatValue}", textAlign = TextAlign.Center)
        Text(text = "Device Height: $deviceHeight", textAlign = TextAlign.Center)
        Slider(value = height.floatValue, onValueChange = {
            height.floatValue = it
            coroutineScope.launch {
                val new =
                    bodyTranslation?.copy(y = height.floatValue)
                        ?: Translation(
                            0.0f,
                            height.floatValue,
                            0.0f
                        )
                viewModel.setBodyTranslation(context, new)
                viewModel.sync(context)
            }
        }, valueRange = 80.0f..250.0f)
        Text(
            text = "Requested Animation Factor: ${animationFactor.floatValue}",
            textAlign = TextAlign.Center
        )
        Text(text = "Device Animation Factor: $deviceAnimationFactor", textAlign = TextAlign.Center)
        Slider(value = animationFactor.floatValue, onValueChange = {
            animationFactor.floatValue = it
                coroutineScope.launch {
                    viewModel.setAnimationFactor(context, animationFactor.floatValue)
                    viewModel.sync(context)
            }
        }, valueRange = 0.1f..3.0f)
    }
}

@Preview(showBackground = true)
@Composable
fun ControlsViewPreview() {
    ControlsView(ConnectedViewModel())
}
