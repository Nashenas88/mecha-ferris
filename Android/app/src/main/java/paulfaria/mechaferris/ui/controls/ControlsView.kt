package paulfaria.mechaferris.ui.controls

import android.util.Log
import androidx.compose.foundation.layout.Column
import androidx.compose.material3.Slider
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.tooling.preview.Preview
import kotlinx.coroutines.launch
import paulfaria.mechaferris.ConnectedViewModel
import paulfaria.mechaferris.Translation

@Composable
fun ControlsView(viewModel: ConnectedViewModel) {
    val coroutineScope = rememberCoroutineScope()
    var height by rememberSaveable { mutableStateOf(viewModel.bodyTranslation?.y ?: 75.0f) }
    var animationFactor by rememberSaveable { mutableStateOf(viewModel.animationFactor ?: 1.0f) }
    val context = LocalContext.current
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Text(text = "Height: $height", textAlign = TextAlign.Center)
        Slider(value = height, onValueChange = {
            height = it
            if (!viewModel.awaitingBodyTranslation) {
                coroutineScope.launch {
                    viewModel.awaitingBodyTranslation = true
                    val new =
                        viewModel.bodyTranslation?.copy(y = height) ?: Translation(0.0f, height, 0.0f)
                    Log.i("ControlsView", "Sending new height: $new")
                    viewModel.setBodyTranslation(context, new)
                    viewModel.sync(context)
                    Log.i("ControlsView", "Sent new height: ${viewModel.bodyTranslation}")
                    viewModel.bodyTranslation?.run {
                        Log.i("ControlsView", "Body Translation: ($x, $y, $z)")
                        height = y
                    }
                    viewModel.awaitingBodyTranslation = false
                }
            }
        }, valueRange = 80.0f..250.0f)
        Text(text = "Animation Factor: $animationFactor", textAlign = TextAlign.Center)
        Slider(value = animationFactor, onValueChange = {
            animationFactor = it
            if (!viewModel.awaitingAnimationFactor) {
                coroutineScope.launch {
                    viewModel.awaitingAnimationFactor = true
                    Log.i("ControlsView", "Sending new animation factor: $animationFactor")
                    viewModel.setAnimationFactor(context, animationFactor)
                    viewModel.sync(context)
                    Log.i("ControlsView", "Sent new animation factor: ${viewModel.animationFactor}")
                    viewModel.animationFactor?.run {
                        Log.i("ControlsView", "Animation Factor: $animationFactor")
                        animationFactor = this
                    }
                    viewModel.awaitingAnimationFactor = false
                }
            }
        }, valueRange = 0.1f..3.0f)
    }
}

@Preview(showBackground = true)
@Composable
fun ControlsViewPreview() {
    ControlsView(ConnectedViewModel())
}
