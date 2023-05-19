package com.mechaferris.mechaferris.ui.calibrating

import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.wrapContentSize
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.Slider
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.tooling.preview.Preview

@Composable
fun CalibratingView() {
    var leg by rememberSaveable { mutableStateOf(0) }
    var legExpanded by remember { mutableStateOf(false) }
    val legs = listOf(1, 2, 3, 4, 5, 6)
    var joint by rememberSaveable { mutableStateOf(0) }
    var jointExpanded by remember { mutableStateOf(false) }
    val joints = listOf("Coxa", "Femur", "Tibia")
    var angle by rememberSaveable { mutableStateOf(0.0f) }
        Column(modifier = Modifier.fillMaxSize()) {
            Text(legs[leg].toString(), modifier = Modifier
                .fillMaxWidth()
                .clickable { legExpanded = true })
            DropdownMenu(
                expanded = legExpanded,
                onDismissRequest = { legExpanded = false },
                modifier = Modifier
                    .fillMaxSize()
                    .wrapContentSize(
                        Alignment.TopStart
                    )
            ) {
                legs.forEachIndexed { index, l ->
                    DropdownMenuItem(text = { Text(text = l.toString()) }, onClick = {
                        leg = index
                        legExpanded = false
                    })
                }
            }
            Text(joints[joint], modifier = Modifier
                .fillMaxWidth()
                .clickable { legExpanded = true })
            DropdownMenu(
                expanded = jointExpanded,
                onDismissRequest = { jointExpanded = false },
                modifier = Modifier
                    .fillMaxSize()
                    .wrapContentSize(
                        Alignment.TopStart
                    )
            ) {
                joints.forEachIndexed { index, j ->
                    DropdownMenuItem(text = { Text(text = j) }, onClick = {
                        joint = index
                        jointExpanded = false
                    })
                }
            }
            Row(verticalAlignment = Alignment.CenterVertically) {
                Text(text = "Pulse2: $angle", textAlign = TextAlign.Center)
                Slider(value = angle, onValueChange = { angle = it }, valueRange = 400.0f..2700.0f)
            }
        }
}

@Preview(showBackground = true)
@Composable
fun CalibratingViewPreview() {
    CalibratingView()
}