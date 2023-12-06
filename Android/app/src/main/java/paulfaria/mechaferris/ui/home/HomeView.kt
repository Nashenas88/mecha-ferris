@file:OptIn(ExperimentalUnsignedTypes::class)

package paulfaria.mechaferris.ui.home

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
import androidx.compose.runtime.getValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.flow
import paulfaria.mechaferris.Quaternion
import paulfaria.mechaferris.R
import paulfaria.mechaferris.StateMachine
import paulfaria.mechaferris.Translation
import paulfaria.mechaferris.Vector3
import paulfaria.mechaferris.toPrettyString
import paulfaria.mechaferris.ui.theme.MechaFerrisTheme

@OptIn(ExperimentalUnsignedTypes::class)
@Composable
fun HomeView(
    deviceName: String?,
    deviceAddress: String?,
    batteryLevel: UInt?,
    stateMachine: StateMachine?,
    animationFactor: Float?,
    angularVelocity: Float?,
    motionVector: Vector3?,
    bodyTranslation: Translation?,
    bodyRotation: Quaternion?,
    legRadius: Float?,) {
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
            HomeViewRow(label = "Device Name", value = deviceName ?: "Unknown name")
            HomeViewRow(
                label = "Device Address",
                value = deviceAddress ?: "Unknown address"
            )
            HomeViewRow(label = "Battery Level") {
                BatteryIcon(batteryLevel = batteryLevel)
            }
            HomeViewRow(
                label = "State Machine",
                value = stateMachine?.toString() ?: "Unknown state"
            )
            HomeViewRow(
                label = "Animation Factor",
                value = animationFactor?.let { "$animationFactor /s" } ?: "Unknown")
            HomeViewRow(
                label = "Angular Velocity",
                value = angularVelocity?.let { "$angularVelocity rad/s" }
                    ?: "Unknown")
            HomeViewRow(
                label = "Motion Vector",
                value = motionVector?.toPrettyString() ?: "Unknown"
            )
            HomeViewRow(
                label = "Body Translation",
                value = bodyTranslation?.toPrettyString() ?: "Unknown"
            )
            HomeViewRow(
                label = "Body Rotation",
                value = bodyRotation?.toPrettyString() ?: "Unknown"
            )
            HomeViewRow(
                label = "Leg Radius",
                value = legRadius?.let { "$legRadius mm" } ?: "Unknown")
        }
    }
}

@Composable
fun BatteryIcon(batteryLevel: UInt?) {
    val (id, contentDescription) = when (batteryLevel) {
        in 0u..8u -> Pair(R.drawable.baseline_battery_alert_24, "battery alert")
        in 9u..21u -> Pair(R.drawable.baseline_battery_0_bar_24, "battery 0/7 bar")
        in 22u..34u -> Pair(R.drawable.baseline_battery_1_bar_24, "battery 1/7 bar")
        in 35u..47u -> Pair(R.drawable.baseline_battery_2_bar_24, "battery 2/7 bar")
        in 48u..60u -> Pair(R.drawable.baseline_battery_3_bar_24, "battery 3/7 bar")
        in 61u..73u -> Pair(R.drawable.baseline_battery_4_bar_24, "battery 4/7 bar")
        in 74u..86u -> Pair(R.drawable.baseline_battery_5_bar_24, "battery 5/7 bar")
        in 87u..99u -> Pair(R.drawable.baseline_battery_6_bar_24, "battery 6/7 bar")
        in 100u..100u -> Pair(R.drawable.baseline_battery_full_24, "battery full bar")
        else -> Pair(R.drawable.baseline_battery_alert_24, "unknown battery alert")
    }
    Row {
        Text(
            text = if (batteryLevel in 0u..100u) {
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

@OptIn(ExperimentalUnsignedTypes::class)
@Preview
@Composable
fun HomeViewPreview() {
    val batteryLevel by flow {
        val delayMs = 100L
        while (true) {
            for (i in 0..100) {
                delay(delayMs)
                emit(i.toUInt())
            }
            for (i in 100 downTo 0) {
                delay(delayMs)
                emit(i.toUInt())
            }
        }
    }.collectAsStateWithLifecycle(initialValue = 0.toUInt())

    MechaFerrisTheme(darkTheme = true) {
        HomeView(
            deviceName = "Mecha Ferris",
            deviceAddress = "00:00:00:00:00:00",
            batteryLevel = batteryLevel,
            stateMachine = StateMachine.PAUSED,
            animationFactor = 1.0f,
            angularVelocity = 0.0f,
            motionVector = Vector3(0.0f, 0.0f, 0.0f),
            bodyTranslation = Translation(0.0f, 0.0f, 0.0f),
            bodyRotation = Quaternion(0.0f, 0.0f, 0.0f, 0.0f),
            legRadius = 100.0f
        )
    }
}

@Preview
@Composable
fun NullHomeViewPreview() {
    MechaFerrisTheme(darkTheme = true) {
        HomeView(
            deviceName = null,
            deviceAddress = null,
            batteryLevel = null,
            stateMachine = null,
            animationFactor = null,
            angularVelocity = null,
            motionVector = null,
            bodyTranslation = null,
            bodyRotation = null,
            legRadius = null,
        )
    }
}
