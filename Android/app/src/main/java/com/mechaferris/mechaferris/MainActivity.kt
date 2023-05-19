package com.mechaferris.mechaferris

import android.annotation.SuppressLint
import android.content.ComponentName
import android.content.Intent
import android.content.ServiceConnection
import android.os.Bundle
import android.os.IBinder
import android.os.Parcelable
import androidx.activity.ComponentActivity
import androidx.activity.viewModels
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.WindowInsets
import androidx.compose.foundation.layout.consumeWindowInsets
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.statusBars
import androidx.compose.foundation.layout.windowInsetsTopHeight
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.platform.ComposeView
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewmodel.compose.viewModel
import com.mechaferris.mechaferris.Constants.Companion.DEVICE_ADDRESS_EXTRA
import com.mechaferris.mechaferris.Constants.Companion.DEVICE_NAME_EXTRA
import com.mechaferris.mechaferris.ui.theme.MechaFerrisTheme
import kotlinx.parcelize.Parcelize

class MainActivity : ComponentActivity() {
    private val dialogViewModel: DialogViewModel by viewModels()
    private val mainViewModel: MainViewModel by viewModels()
    private var bleServiceData: BleService.BluetoothServiceBinder? = null
    private var bleServiceConn: MFServiceConn? = null

    private inner class MFServiceConn : ServiceConnection {
        @SuppressLint("MissingPermission")
        override fun onServiceConnected(name: ComponentName?, service: IBinder?) {
            if (BuildConfig.DEBUG && BleService::class.java.name != name?.className) {
                error("Connected to unknown service")
            } else {
                bleServiceData = service as BleService.BluetoothServiceBinder
                bleServiceData?.setDeviceCallbacks(onDeviceFound = {
                    val state = BluetoothDeviceState(it.name, it.address)
                    val prev = mainViewModel.devicesSet.put(it.address, state)
                    if (prev == null) {
                        mainViewModel.devices.add(state)
                    } else if (prev.name != it.name) {
                        mainViewModel.devices[mainViewModel.devices.indexOf(prev)] = state
                    }
                }, onDeviceLost = {
                    val state = BluetoothDeviceState(it.name, it.address)
                    mainViewModel.devicesSet.remove(it.address)
                    mainViewModel.devices.removeIf { d -> d.address == state.address }
                })
            }
        }

        override fun onServiceDisconnected(name: ComponentName?) {
            if (BuildConfig.DEBUG && BleService::class.java.name != name?.className) {
                error("Disconnected from unknown service")
            } else {
                bleServiceData = null
                dialogViewModel.onDisconnected()
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        setContentView(ComposeView(this).apply {
            consumeWindowInsets = true
            val intent = Intent(context, Connected::class.java)
            setContent {
                MechaFerrisTheme {
                    MainView(onConnect = { device ->
                        intent.putExtra(DEVICE_NAME_EXTRA, device.name)
                        intent.putExtra(DEVICE_ADDRESS_EXTRA, device.address)
                        startActivity(intent)
                    })
                }
            }
        })

        startService(Intent(this, BleService::class.java))
    }


    override fun onStart() {
        super.onStart()
        val latestServiceConn = MFServiceConn()
        if (bindService(
                Intent(BleService.DATA_PLANE_ACTION, null, this, BleService::class.java),
                latestServiceConn,
                0
            )
        ) {
            bleServiceConn = latestServiceConn
        }
    }

    override fun onStop() {
        super.onStop()

        if (bleServiceConn != null) {
            unbindService(bleServiceConn!!)
            bleServiceConn = null
        }
    }
}

@Composable
fun MainView(viewModel: MainViewModel = viewModel(), onConnect: (BluetoothDeviceState) -> Unit) {
    val devices = remember { viewModel.devices }
    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background)
            .padding(top = 12.dp), horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Spacer(Modifier.windowInsetsTopHeight(WindowInsets.statusBars))
        if (devices.size == 0) {
            Row(
                modifier = Modifier
                    .height(56.dp)
                    .fillMaxWidth()
                    .padding(horizontal = 12.dp),
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.Center
            ) {
                Text(
                    text = "Please connect a MechaFerris device over bluetooth",
                    style = MaterialTheme.typography.bodyMedium,
                    color = MaterialTheme.colorScheme.onBackground,
                    textAlign = TextAlign.Center,
                )
            }
        }

        // Recycler View of devices
        LazyColumn(
            modifier = Modifier
                .fillMaxSize()
                .padding(top = 12.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            items(devices) { device ->
                DeviceRow(modifier = Modifier
                    .height(56.dp)
                    .fillMaxWidth()
                    .padding(horizontal = 12.dp)
                    .clip(CircleShape)
                    .clickable { onConnect(device) }
                    .then(Modifier.background(MaterialTheme.colorScheme.secondaryContainer)), device)
            }
        }
    }
}

class MainViewModel : ViewModel() {
    var devicesSet = HashMap<String, BluetoothDeviceState>()
    val devices = mutableStateListOf<BluetoothDeviceState>()
}

@Composable
fun DeviceRow(modifier: Modifier = Modifier, device: BluetoothDeviceState? = null) {
    Row(
        modifier = modifier,
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.Center
    ) {
        Text(
            text = device?.name ?: "Unknown",
            style = MaterialTheme.typography.bodyMedium,
            color = MaterialTheme.colorScheme.onSecondaryContainer,
            textAlign = TextAlign.Start,
            modifier = Modifier.padding(start = 12.dp)
        )
        Spacer(modifier = Modifier.weight(0.1f))
        Text(
            text = device?.address ?: "Unknown",
            style = MaterialTheme.typography.bodyMedium,
            color = MaterialTheme.colorScheme.onSecondaryContainer,
            textAlign = TextAlign.End,
            modifier = Modifier.padding(end = 12.dp)
        )
    }
}

@Parcelize
data class BluetoothDeviceState(
    val name: String, val address: String
) : Parcelable


@Composable
@Preview
fun MainViewPreview() {
    MechaFerrisTheme(darkTheme = true) {
        val viewModel = remember {
            MainViewModel().apply {
                devices.apply {
                    add(BluetoothDeviceState("MechaFerris", "87:b2:ac:21:1e:6c"))
                }
            }
        }
        Surface {
            MainView(viewModel) {}
        }
    }
}

@Composable
@Preview
fun MainViewEmptyPreview() {
    MechaFerrisTheme(darkTheme = true) {
        val viewModel = remember {
            MainViewModel()
        }
        Surface {
            MainView(viewModel) {}
        }
    }
}
