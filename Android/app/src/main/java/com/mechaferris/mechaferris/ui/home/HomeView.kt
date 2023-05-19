package com.mechaferris.mechaferris.ui.home

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.RowScope
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.mechaferris.mechaferris.ConnectedViewModel
import com.mechaferris.mechaferris.R
import com.mechaferris.mechaferris.StateMachine
import com.mechaferris.mechaferris.Vector3
import com.mechaferris.mechaferris.toPrettyString
import com.mechaferris.mechaferris.ui.theme.MechaFerrisTheme

@Composable
fun HomeView(state: ConnectedViewModel) {
    Row(
        modifier = Modifier
            .clip(RoundedCornerShape(12.dp))
            .background(MaterialTheme.colorScheme.primaryContainer)
    ) {
        Column(
            Modifier
                .fillMaxWidth()
                .padding(12.dp)
        ) {
            HomeViewRow(label = "Device Name", value = state.deviceName ?: "Unknown name")
            HomeViewRow(
                label = "Device Address",
                value = state.deviceAddress ?: "Unknown address"
            )
            HomeViewRow(label = "Battery Level") {
                BatteryIcon(batteryLevel = state.batteryLevel)
            }
            HomeViewRow(
                label = "State Machine",
                value = state.stateMachine?.toString() ?: "Unknown state"
            )
            HomeViewRow(
                label = "Animation Factor",
                value = state.animationFactor?.let { "${state.animationFactor} m/s" } ?: "Unknown")
            HomeViewRow(
                label = "Angular Velocity",
                value = state.angularVelocity?.let { "${state.angularVelocity} rad/s" }
                    ?: "Unknown")
            HomeViewRow(
                label = "Motion Vector",
                value = state.motionVector?.toPrettyString() ?: "Unknown"
            )
            HomeViewRow(
                label = "Body Translation",
                value = state.bodyTranslation?.toPrettyString() ?: "Unknown"
            )
            HomeViewRow(
                label = "Body Rotation",
                value = state.bodyRotation?.toPrettyString() ?: "Unknown"
            )
            HomeViewRow(
                label = "Leg Radius",
                value = state.legRadius?.let { "${state.legRadius} m" } ?: "Unknown")
        }
    }
}

@Composable
fun BatteryIcon(batteryLevel: Int?) {
    val (id, contentDescription) = when (batteryLevel) {
        in 0..8 -> Pair(R.drawable.baseline_battery_alert_24, "battery alert")
        in 9..21 -> Pair(R.drawable.baseline_battery_0_bar_24, "battery 0/7 bar")
        in 22..34 -> Pair(R.drawable.baseline_battery_1_bar_24, "battery 1/7 bar")
        in 35..47 -> Pair(R.drawable.baseline_battery_2_bar_24, "battery 2/7 bar")
        in 48..60 -> Pair(R.drawable.baseline_battery_3_bar_24, "battery 3/7 bar")
        in 61..73 -> Pair(R.drawable.baseline_battery_4_bar_24, "battery 4/7 bar")
        in 74..86 -> Pair(R.drawable.baseline_battery_5_bar_24, "battery 5/7 bar")
        in 87..99 -> Pair(R.drawable.baseline_battery_6_bar_24, "battery 6/7 bar")
        in 100..100 -> Pair(R.drawable.baseline_battery_full_24, "battery full bar")
        else -> Pair(R.drawable.baseline_battery_alert_24, "unknown battery alert")
    }
    Row {
        Text(
            text = if (batteryLevel in 0..100) {
                "$batteryLevel%"
            } else {
                "--%"
            }, color = MaterialTheme.colorScheme.onPrimaryContainer
        )
        Icon(
            painter = painterResource(id),
            contentDescription = contentDescription,
            tint = MaterialTheme.colorScheme.onPrimaryContainer
        )
    }
}

@Composable
fun HomeViewRow(
    modifier: Modifier = Modifier,
    label: String = "",
    value: String = "",
    content: @Composable (RowScope.() -> Unit)? = null
) {
    Row {
        Text(
            text = label,
            color = MaterialTheme.colorScheme.onPrimaryContainer,
            modifier = modifier
        )
        Spacer(modifier = Modifier.weight(1f))
        if (content == null) {
            Text(
                text = value,
                color = MaterialTheme.colorScheme.onPrimaryContainer,
                modifier = modifier
            )
        } else {
            content()
        }
    }
}

@Preview
@Composable
fun HomeViewPreview() {
    val state = ConnectedViewModel()
    state.batteryLevel = 25
    state.stateMachine = StateMachine.PAUSED
    state.legRadius = 100f
    state.bodyTranslation = Vector3(0f, 75f, 0f)
    MechaFerrisTheme(darkTheme = true) {
        HomeView(state = state)
    }
}
