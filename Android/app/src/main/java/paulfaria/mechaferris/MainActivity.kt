package paulfaria.mechaferris

import android.Manifest
import android.app.Application
import android.os.Bundle
import android.os.Parcelable
import androidx.activity.ComponentActivity
import androidx.activity.viewModels
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Arrangement.Center
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxScope
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.consumeWindowInsets
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Delete
import androidx.compose.material.icons.filled.Refresh
import androidx.compose.material.icons.filled.Warning
import androidx.compose.material3.Button
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.ProvideTextStyle
import androidx.compose.material3.Snackbar
import androidx.compose.material3.Text
import androidx.compose.material3.TopAppBar
import androidx.compose.material3.contentColorFor
import androidx.compose.runtime.Composable
import androidx.compose.runtime.SideEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Alignment.Companion.BottomCenter
import androidx.compose.ui.Alignment.Companion.CenterHorizontally
import androidx.compose.ui.Alignment.Companion.CenterVertically
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.platform.ComposeView
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.google.accompanist.permissions.ExperimentalPermissionsApi
import com.google.accompanist.permissions.MultiplePermissionsState
import com.google.accompanist.permissions.rememberMultiplePermissionsState
import com.juul.kable.AndroidAdvertisement
import com.juul.kable.Bluetooth
import com.juul.kable.Reason
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.catch
import kotlinx.coroutines.flow.onCompletion
import kotlinx.coroutines.launch
import kotlinx.coroutines.withTimeoutOrNull
import kotlinx.parcelize.Parcelize
import paulfaria.mechaferris.ScanStatus.Stopped
import paulfaria.mechaferris.icons.BluetoothDisabled
import paulfaria.mechaferris.icons.LocationDisabled
import paulfaria.mechaferris.ui.theme.MechaFerrisTheme
import java.util.concurrent.TimeUnit
import kotlin.coroutines.cancellation.CancellationException

class MainActivity : ComponentActivity() {
    private val mainViewModel: MainViewModel by viewModels()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        setContentView(ComposeView(this).apply {
            consumeWindowInsets = true
            setContent {
                MechaFerrisTheme {
                    Column(Modifier.background(color = MaterialTheme.colorScheme.background)) {
                        val bluetooth = Bluetooth.availability.collectAsState(initial = null).value
                        AppBar(mainViewModel, bluetooth)
                        Box(Modifier.weight(1f)) {
                            ScanPane(bluetooth)
                            StatusSnackbar(mainViewModel)
                        }
                    }
                }
            }
        })
    }

    @OptIn(ExperimentalPermissionsApi::class)
    @Composable
    private fun ScanPane(bluetooth: Bluetooth.Availability?) {
        ProvideTextStyle(
            TextStyle(color = contentColorFor(backgroundColor = MaterialTheme.colorScheme.background))
        ) {
            val permissionsState = rememberMultiplePermissionsState(listOf(
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_CONNECT
            ))

            var didAskForPermission by remember { mutableStateOf(false) }
            if (!didAskForPermission) {
                didAskForPermission = true
                SideEffect {
                    permissionsState.launchMultiplePermissionRequest()
                }
            }

            if (permissionsState.allPermissionsGranted) {
                PermissionGranted(bluetooth)
            } else {
                if (permissionsState.shouldShowRationale) {
                    BluetoothPermissionsNotGranted(permissionsState)
                } else {
                    BluetoothPermissionsNotAvailable(::openAppDetails)
                }
            }
        }
    }

    @Composable
    private fun PermissionGranted(bluetooth: Bluetooth.Availability?) {
        when (bluetooth) {
            Bluetooth.Availability.Available -> {
                AdvertisementsList(
                    advertisements = mainViewModel.advertisements.collectAsState().value,
                    onRowClick = ::onAdvertisementClicked
                )
            }
            is Bluetooth.Availability.Unavailable -> when (bluetooth.reason) {
                Reason.LocationServicesDisabled -> LocationServicesDisabled(::showLocationSettings)
                Reason.Off, Reason.TurningOff -> BluetoothDisabled(::enableBluetooth)
                Reason.TurningOn -> Loading()
                null -> BluetoothUnavailable()
            }
            null -> Loading()
        }
    }

    @Composable
    private fun BluetoothUnavailable() {
        Column(
            Modifier
                .fillMaxSize()
                .padding(20.dp),
            horizontalAlignment = CenterHorizontally,
            verticalArrangement = Center,
        ) {
            Text(text = "Bluetooth unavailable.")
        }
    }

    private fun onAdvertisementClicked(advertisement: AndroidAdvertisement) {
        mainViewModel.stop()
        val intent = ConnectedIntent(
            context = this@MainActivity,
            macAddress = advertisement.address,
            deviceName = advertisement.name,
        )
        startActivity(intent)
    }

    override fun onPause() {
        super.onPause()
        mainViewModel.stop()
    }
}



@OptIn(ExperimentalMaterial3Api::class)
@Composable
private fun AppBar(viewModel: MainViewModel, bluetooth: Bluetooth.Availability?) {
    val status = viewModel.status.collectAsState().value

    TopAppBar(
        title = {
            Text("Mecha Ferris")
        },
        actions = {
            if (bluetooth == Bluetooth.Availability.Available) {
                if (status !is ScanStatus.Scanning) {
                    IconButton(onClick = viewModel::start) {
                        Icon(Icons.Filled.Refresh, contentDescription = "Refresh")
                    }
                }
                IconButton(onClick = viewModel::clear) {
                    Icon(Icons.Filled.Delete, contentDescription = "Clear")
                }
            }
        }
    )
}

@Composable
private fun BoxScope.StatusSnackbar(viewModel: MainViewModel) {
    val status = viewModel.status.collectAsState().value

    if (status !is Stopped) {
        val text = when (status) {
            ScanStatus.Scanning -> "Scanning"
            Stopped -> "Idle"
            is ScanStatus.Failed -> "Error: ${status.message}"
        }
        Snackbar(
            Modifier
                .align(BottomCenter)
                .padding(10.dp)
        ) {
            Text(text, style = MaterialTheme.typography.bodySmall)
        }
    }
}

@Composable
private fun ActionRequired(
    icon: ImageVector,
    contentDescription: String?,
    description: String,
    buttonText: String,
    onClick: () -> Unit,
) {
    Column(
        Modifier
            .fillMaxSize()
            .padding(20.dp),
        horizontalAlignment = CenterHorizontally,
        verticalArrangement = Center,
    ) {
        Icon(
            modifier = Modifier.size(150.dp),
            tint = contentColorFor(backgroundColor = MaterialTheme.colorScheme.background),
            imageVector = icon,
            contentDescription = contentDescription,
        )
        Spacer(Modifier.size(8.dp))
        Text(
            modifier = Modifier
                .fillMaxWidth()
                .align(CenterHorizontally),
            textAlign = TextAlign.Center,
            text = description,
        )
        Spacer(Modifier.size(15.dp))
        Button(onClick) {
            Text(buttonText)
        }
    }
}

@Composable
private fun BluetoothDisabled(enableAction: () -> Unit) {
    ActionRequired(
        icon = Icons.Filled.BluetoothDisabled,
        contentDescription = "Bluetooth disabled",
        description = "Bluetooth is disabled.",
        buttonText = "Enable",
        onClick = enableAction,
    )
}

@Composable
private fun LocationServicesDisabled(enableAction: () -> Unit) {
    ActionRequired(
        icon = Icons.Filled.LocationDisabled,
        contentDescription = "Location services disabled",
        description = "Location services are disabled.",
        buttonText = "Enable",
        onClick = enableAction,
    )
}

@OptIn(ExperimentalPermissionsApi::class)
@Composable
private fun BluetoothPermissionsNotGranted(permissions: MultiplePermissionsState) {
    ActionRequired(
        icon = Icons.Filled.LocationDisabled,
        contentDescription = "Bluetooth permissions required",
        description = "Bluetooth permissions are required for scanning. Please grant the permission.",
        buttonText = "Continue",
        onClick = permissions::launchMultiplePermissionRequest,
    )
}

@Composable
private fun BluetoothPermissionsNotAvailable(openSettingsAction: () -> Unit) {
    ActionRequired(
        icon = Icons.Filled.Warning,
        contentDescription = "Bluetooth permissions required",
        description = "Bluetooth permission denied. Please, grant access on the Settings screen.",
        buttonText = "Open Settings",
        onClick = openSettingsAction,
    )
}

@Composable
private fun Loading() {
    Column(
        Modifier
            .fillMaxSize()
            .padding(20.dp),
        horizontalAlignment = CenterHorizontally,
        verticalArrangement = Center,
    ) {
        CircularProgressIndicator()
    }
}

@Composable
private fun AdvertisementsList(
    advertisements: List<AndroidAdvertisement>,
    onRowClick: (AndroidAdvertisement) -> Unit
) {
    LazyColumn {
        items(advertisements.size) { index ->
            val advertisement = advertisements[index]
            AdvertisementRow(advertisement) { onRowClick(advertisement) }
        }
    }
}

@Composable
private fun AdvertisementRow(advertisement: AndroidAdvertisement, onClick: () -> Unit) {
    Row(
        Modifier
            .fillMaxWidth()
            .padding(20.dp)
            .clickable(onClick = onClick)
    ) {
        Column(Modifier.weight(1f)) {
            Text(
                fontSize = 22.sp,
                text = advertisement.name ?: "Unknown",
            )
            Text(advertisement.address)
        }

        Text(
            modifier = Modifier.align(CenterVertically),
            text = "${advertisement.rssi} dBm",
        )
    }
}

//@OptIn(ExperimentalPermissionsApi::class)
//@Composable
//fun MainView(viewModel: MainViewModel = viewModel(), bluetooth: Bluetooth.Availability?) {
//    val bleEnabled = viewModel.bleEnabled.collectAsStateWithLifecycle()
//    val permissionsState = rememberMultiplePermissionsState(
//        permissions = listOf(
//            android.Manifest.permission.BLUETOOTH_SCAN,
//            android.Manifest.permission.BLUETOOTH_CONNECT,
//        )
//    )
//    var didAskForPermission by remember { mutableStateOf(false) }
//    if (!didAskForPermission) {
//        didAskForPermission = true
//        SideEffect {
//            permissionsState.launchMultiplePermissionRequest()
//        }
//    }
//
//    if (bleEnabled.value) {
//        if (permissionsState.allPermissionsGranted) {
//            PermissionGranted(viewModel, bluetooth)
//        } else if (permissionsState.shouldShowRationale) {
//            BluetoothPermissionsNotGranted(permissionsState)
//        } else {
//            val activity = LocalContext.current.findActivity()
//            BluetoothPermissionsNotAvailable {
//                activity.openAppDetails()
//            }
//        }
//    } else {
//        EnableBluetooth()
//    }
//}
//
//fun Context.findActivity(): Activity {
//    var context = this
//    while (context is ContextWrapper) {
//        if (context is Activity) {
//            return context
//        }
//        context = context.baseContext
//    }
//    throw IllegalStateException("no activity")
//}

//@Composable
//fun PermissionGranted(viewModel: MainViewModel, onConnect: (BluetoothDeviceState) -> Unit) {
//    val devices = viewModel.devices.collectAsStateWithLifecycle()
//    Column(
//        modifier = Modifier
//            .fillMaxSize()
//            .background(MaterialTheme.colorScheme.background)
//            .padding(top = 12.dp), horizontalAlignment = CenterHorizontally
//    ) {
//        Spacer(Modifier.windowInsetsTopHeight(WindowInsets.statusBars))
//        if (devices.value.isEmpty()) {
//            Row(
//                modifier = Modifier
//                    .height(56.dp)
//                    .fillMaxWidth()
//                    .padding(horizontal = 12.dp),
//                verticalAlignment = Alignment.CenterVertically,
//                horizontalArrangement = Center
//            ) {
//                Text(
//                    text = "Please connect a MechaFerris device over bluetooth",
//                    style = MaterialTheme.typography.bodyMedium,
//                    color = MaterialTheme.colorScheme.onBackground,
//                    textAlign = TextAlign.Center,
//                )
//            }
//        }
//
//        // Recycler View of devices
//        LazyColumn(
//            modifier = Modifier
//                .fillMaxSize()
//                .padding(top = 12.dp),
//            verticalArrangement = Arrangement.spacedBy(8.dp),
//            horizontalAlignment = Alignment.CenterHorizontally
//        ) {
//            devices.value.forEach { device ->
//                item {
//                    DeviceRow(modifier = Modifier
//                        .height(56.dp)
//                        .fillMaxWidth()
//                        .padding(horizontal = 12.dp)
//                        .clip(CircleShape)
//                        .clickable { onConnect(device) }
//                        .then(Modifier.background(MaterialTheme.colorScheme.secondaryContainer)),
//                        device)
//                }
//            }
//        }
//    }
//}

//@Composable
//fun EnableBluetooth() {
//    val activity = LocalContext.current.findActivity()
//    ActionRequired(
//        contentDescription = "Bluetooth disabled",
//        description = "Bluetooth is disabled.",
//        buttonText = "Enable",
//        onClick = {
//            activity.startActivity(Intent(Settings.ACTION_BLUETOOTH_SETTINGS))
//        },
//    )
//}

private val SCAN_DURATION_MILLIS = TimeUnit.SECONDS.toMillis(10)

sealed class ScanStatus {
    data object Stopped : ScanStatus()
    data object Scanning : ScanStatus()
    data class Failed(val message: CharSequence) : ScanStatus()
}

class MainViewModel(application: Application) : AndroidViewModel(application) {

    private val scanScope = viewModelScope.childScope()
    private val found = hashMapOf<String, AndroidAdvertisement>()

    private val _status = MutableStateFlow<ScanStatus>(Stopped)
    val status = _status.asStateFlow()

    private val _advertisements = MutableStateFlow<List<AndroidAdvertisement>>(emptyList())
    val advertisements = _advertisements.asStateFlow()

    fun start() {
        if (_status.value == ScanStatus.Scanning) return // Scan already in progress.
        _status.value = ScanStatus.Scanning

        scanScope.launch {
            withTimeoutOrNull(SCAN_DURATION_MILLIS) {
                scanner
                    .advertisements
                    .catch { cause -> _status.value = ScanStatus.Failed(cause.message ?: "Unknown error") }
                    .onCompletion { cause -> if (cause == null || cause is CancellationException) _status.value = Stopped }
                    .collect { advertisement ->
                        found[advertisement.address] = advertisement
                        _advertisements.value = found.values.toList()
                    }
            }
        }
    }

    fun stop() {
        scanScope.cancelChildren()
    }

    fun clear() {
        stop()
        _advertisements.value = emptyList()
    }
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

@Composable
private fun ActionRequired(
    contentDescription: String?,
    description: String,
    buttonText: String,
    onClick: () -> Unit,
) {
    Column(
        Modifier
            .fillMaxSize()
            .padding(20.dp),
        horizontalAlignment = CenterHorizontally,
        verticalArrangement = Center,
    ) {
        if (contentDescription != null) {
            Text(
                modifier = Modifier.size(150.dp),
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.onBackground,
                textAlign = TextAlign.Center,
                text = contentDescription,
            )
        }
        Spacer(Modifier.size(8.dp))
        Text(
            modifier = Modifier
                .fillMaxWidth()
                .align(CenterHorizontally),
            style = MaterialTheme.typography.bodyMedium,
            color = MaterialTheme.colorScheme.onBackground,
            textAlign = TextAlign.Center,
            text = description,
        )
        Spacer(Modifier.size(15.dp))
        Button(onClick) {
            Text(buttonText)
        }
    }
}

@Parcelize
data class BluetoothDeviceState(
    val name: String, val address: String
) : Parcelable


//@Composable
//@Preview
//fun MainViewPreview() {
//    val devices = MutableStateFlow(setOf(BluetoothDeviceState("MechaFerris", "87:b2:ac:21:1e:6c")))
//    val bleEnabled = MutableStateFlow(false)
//    val viewModel = MainViewModel(devices, bleEnabled)
//    LaunchedEffect(Unit) {
//        while (true) {
//            for (i in 0..10) {
//                delay(500)
//                devices.value = devices.value + BluetoothDeviceState("MechaFerris $i", "87:b2:ac:21:1e:6c")
//            }
//            delay(1000)
//            devices.value = devices.value - BluetoothDeviceState("MechaFerris", "87:b2:ac:21:1e:6c")
//            delay(1000)
//        }
//    }
//    MechaFerrisTheme(darkTheme = true) {
//        Surface {
//            MainView(viewModel) {}
//        }
//    }
//}
//
//@Composable
//@Preview
//fun MainViewEmptyPreview() {
//    MechaFerrisTheme(darkTheme = true) {
//        val viewModel = remember {
//            MainViewModel()
//        }
//        Surface {
//            MainView(viewModel) {}
//        }
//    }
//}
