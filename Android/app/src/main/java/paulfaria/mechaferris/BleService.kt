@file:OptIn(ExperimentalUnsignedTypes::class)

package paulfaria.mechaferris

import android.Manifest
import android.annotation.SuppressLint
import android.app.Service
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
import android.bluetooth.BluetoothManager
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.os.Binder
import android.os.IBinder
import android.util.Log
import androidx.core.app.ActivityCompat
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.channels.SendChannel
import kotlinx.coroutines.launch
import kotlinx.coroutines.suspendCancellableCoroutine
import no.nordicsemi.android.ble.BleManager
import no.nordicsemi.android.ble.data.Data
import paulfaria.mechaferris.calibration.ServoCalibration
import java.util.UUID

class BleService : Service() {
    private val defaultScope = CoroutineScope(Dispatchers.Default)
    private lateinit var bluetoothObserver: BroadcastReceiver
    private var batteryChangeNotifications: SendChannel<UInt>? = null
    private var stateMachineChangeNotifications: SendChannel<StateMachine>? = null
    private var animationFactorChangeNotifications: SendChannel<Float>? = null
    private var angularVelocityChangeNotifications: SendChannel<Float>? = null
    private var motionVectorChangeNotifications: SendChannel<Vector3>? = null
    private var bodyTranslationChangeNotifications: SendChannel<Translation>? = null
    private var bodyRotationChangeNotifications: SendChannel<Quaternion>? = null
    private var legRadiusChangeNotifications: SendChannel<Float>? = null
    private var batteryUpdateIntervalMsChangeNotifications: SendChannel<UInt>? = null
    private var getCalibrationResultChangeNotifications: SendChannel<CalibrationDatum>? = null
    private var setCalibrationResultChangeNotifications: SendChannel<SetResult>? = null
    private var onDeviceFound: ((BluetoothDevice) -> Unit)? = null
    private var onDeviceLost: ((BluetoothDevice) -> Unit)? = null
    private var clientManagers = mutableMapOf<String, ClientManager>()

    override fun onCreate() {
        super.onCreate()
        bluetoothObserver = object : BroadcastReceiver() {
            @SuppressLint("MissingPermission")
            override fun onReceive(context: Context?, intent: Intent?) {
                when (intent?.action) {
                    BluetoothAdapter.ACTION_STATE_CHANGED -> {
                        val bluetoothState = intent.getIntExtra(
                            BluetoothAdapter.EXTRA_STATE, -1
                        )
                        when (bluetoothState) {
                            BluetoothAdapter.STATE_ON -> enableBleServices()
                            BluetoothAdapter.STATE_OFF -> disableBleServices()
                        }
                    }

                    BluetoothDevice.ACTION_BOND_STATE_CHANGED -> {
                        val device = intent.getParcelableExtra(
                            BluetoothDevice.EXTRA_DEVICE, BluetoothDevice::class.java
                        )
                        Log.d(
                            TAG,
                            "Bond state changed for device ${device?.address}: ${device?.bondState}"
                        )
                        when (device?.bondState) {
                            BluetoothDevice.BOND_BONDED -> addDevice(device)
                            BluetoothDevice.BOND_NONE -> removeDevice(device)
                        }
                    }

                }
            }
        }
        registerReceiver(bluetoothObserver, IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED))
        registerReceiver(bluetoothObserver, IntentFilter(BluetoothDevice.ACTION_BOND_STATE_CHANGED))

        val bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        if (bluetoothManager.adapter?.isEnabled == true) {
            enableBleServices()
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        unregisterReceiver(bluetoothObserver)
        disableBleServices()
    }

    private fun addDevice(device: BluetoothDevice) {
        if (!clientManagers.containsKey(device.address)) {
            val clientManager = ClientManager()
            clientManager.connect(device).useAutoConnect(true).enqueue()
            clientManagers[device.address] = clientManager
            onDeviceFound?.invoke(device)
        }
    }

    private fun removeDevice(device: BluetoothDevice) {
        clientManagers.remove(device.address)?.close()
        onDeviceLost?.invoke(device)
    }

    @SuppressLint("MissingPermission")
    private fun enableBleServices() {
        val bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        if (bluetoothManager.adapter?.isEnabled == true) {
            Log.i(TAG, "Enabling BLE services")
            bluetoothManager.adapter.bondedDevices.forEach { addDevice(it) }
        } else {
            Log.w(
                TAG, "Cannot enable BLE services. There is no Bluetooth adapter or it is disabled"
            )
        }
    }

    private fun disableBleServices() {
        Log.d(TAG, "Disabling BLE services")
        clientManagers.values.forEach { it.close() }
        clientManagers.clear()
    }

    private fun missingScanPermission(): Boolean {
        return ActivityCompat.checkSelfPermission(
            this, Manifest.permission.BLUETOOTH_SCAN
        ) != PackageManager.PERMISSION_GRANTED
    }

    private fun missingConnectPermission(): Boolean {
        return ActivityCompat.checkSelfPermission(
            this, Manifest.permission.BLUETOOTH_CONNECT
        ) != PackageManager.PERMISSION_GRANTED
    }

    private fun requestPermissions() {
//        requestMultiplePermissions.launch(
//            arrayOf(
//                Manifest.permission.BLUETOOTH_CONNECT, Manifest.permission.BLUETOOTH_SCAN
//            )
//        )
    }

    inner class BluetoothServiceBinder : Binder() {
        suspend fun setBatteryLevel(address: String, level: UInt) {
            clientManagers[address]?.setBatteryLevel(level)
        }

        @SuppressLint("MissingPermission")
        suspend fun sync(address: String, all: Boolean) {
            clientManagers[address]?.sync(all)
        }

        suspend fun setStateMachine(address: String, stateMachine: StateMachine) {
            clientManagers[address]?.setStateMachine(stateMachine)
        }

        suspend fun setAnimationFactor(address: String, animationFactor: Float) {
            clientManagers[address]?.setAnimationFactor(animationFactor)
        }

        suspend fun setAngularVelocity(address: String, angularVelocity: Float) {
            clientManagers[address]?.setAngularVelocity(angularVelocity)
        }

        suspend fun setMotionVector(address: String, motionVector: Vector3) {
            clientManagers[address]?.setMotionVector(motionVector)
        }

        suspend fun setBodyTranslation(address: String, bodyTranslation: Translation) {
            Log.i("BleService", "Sending $bodyTranslation")
            clientManagers[address]?.setBodyTranslation(bodyTranslation)
        }

        suspend fun setBodyRotation(address: String, bodyRotation: Quaternion) {
            clientManagers[address]?.setBodyRotation(bodyRotation)
        }

        suspend fun setLegRadius(address: String, radius: Float) {
            clientManagers[address]?.setLegRadius(radius)
        }

        suspend fun setBatteryUpdateIntervalMs(address: String, intervalMs: UInt) {
            clientManagers[address]?.setBatteryUpdateIntervalMs(intervalMs)
        }

        suspend fun setCalibrationPulse(address: String, calibration: ServoCalibration) {
            clientManagers[address]?.setCalibrationPulse(
                calibration.leg.toUByte(),
                calibration.joint.number.toUByte(),
                calibration.kind.number.toUByte(),
                calibration.pulse
            )
        }

        fun setBatteryChangedChannel(channel: SendChannel<UInt>) {
            batteryChangeNotifications = channel
        }

        fun setStateMachineChangedChannel(channel: SendChannel<StateMachine>) {
            stateMachineChangeNotifications = channel
        }

        fun setAnimationFactorChangedChannel(channel: SendChannel<Float>) {
            animationFactorChangeNotifications = channel
        }

        fun setAngularVelocityChangedChannel(channel: SendChannel<Float>) {
            angularVelocityChangeNotifications = channel
        }

        fun setMotionVectorChangedChannel(channel: SendChannel<Vector3>) {
            motionVectorChangeNotifications = channel
        }

        fun setBodyTranslationChangedChannel(channel: SendChannel<Translation>) {
            bodyTranslationChangeNotifications = channel
        }

        fun setBodyRotationChangedChannel(channel: SendChannel<Quaternion>) {
            bodyRotationChangeNotifications = channel
        }

        fun setLegRadiusChangedChannel(channel: SendChannel<Float>) {
            legRadiusChangeNotifications = channel
        }

        fun setBatteryUpdateIntervalMsChangedChannel(channel: SendChannel<UInt>) {
            batteryUpdateIntervalMsChangeNotifications = channel
        }

        fun setGetCalibrationResultChangedChannel(channel: SendChannel<CalibrationDatum>) {
            getCalibrationResultChangeNotifications = channel
        }

        fun setSetCalibrationResultChangedChannel(channel: SendChannel<SetResult>) {
            setCalibrationResultChangeNotifications = channel
        }

        suspend fun getBatteryLevel(address: String): UInt? {
            return clientManagers[address]?.getBatteryLevel()
        }

        suspend fun getStateMachine(address: String): StateMachine? {
            return clientManagers[address]?.getStateMachine()
        }

        suspend fun getAnimationFactor(address: String): Float? {
            return clientManagers[address]?.getAnimationFactor()
        }

        suspend fun getAngularVelocity(address: String): Float? {
            return clientManagers[address]?.getAngularVelocity()
        }

        suspend fun getMotionVector(address: String): Vector3? {
            return clientManagers[address]?.getMotionVector()
        }

        suspend fun getBodyTranslation(address: String): Translation? {
            return clientManagers[address]?.getBodyTranslation()
        }

        suspend fun getBodyRotation(address: String): Quaternion? {
            return clientManagers[address]?.getBodyRotation()
        }

        suspend fun getLegRadius(address: String): Float? {
            return clientManagers[address]?.getLegRadius()
        }

        suspend fun getBatteryUpdateIntervalMs(address: String): UInt? {
            return clientManagers[address]?.getBatteryUpdateIntervalMs()
        }

        suspend fun requestCalibrationData(
            address: String, leg: UByte, joint: UByte, kind: UByte
        ) {
            clientManagers[address]?.requestCalibrationPulse(leg, joint, kind)
        }

        fun setDeviceCallbacks(
            onDeviceFound: (BluetoothDevice) -> Unit, onDeviceLost: (BluetoothDevice) -> Unit
        ) {
            this@BleService.onDeviceFound = onDeviceFound
            this@BleService.onDeviceLost = onDeviceLost
            clientManagers.values.forEach { clientManager ->
                if (clientManager.isReady) {
                    clientManager.bluetoothDevice?.let { device ->
                        onDeviceFound(device)
                    }
                }
            }
        }
    }

    override fun onBind(intent: Intent?): IBinder? = when (intent?.action) {
        DATA_PLANE_ACTION -> BluetoothServiceBinder()
        else -> null
    }

    override fun onUnbind(intent: Intent?): Boolean = when (intent?.action) {
        DATA_PLANE_ACTION -> {
            batteryChangeNotifications = null
            stateMachineChangeNotifications = null
            animationFactorChangeNotifications = null
            angularVelocityChangeNotifications = null
            motionVectorChangeNotifications = null
            bodyTranslationChangeNotifications = null
            bodyRotationChangeNotifications = null
            legRadiusChangeNotifications = null
            batteryUpdateIntervalMsChangeNotifications = null
            onDeviceFound = null
            onDeviceLost = null

            true
        }

        else -> false
    }

    companion object {
        /**
         * A binding action to return a binding that can be used in relation to the service's data
         */
        const val DATA_PLANE_ACTION = "data-plane"

        private const val TAG = "gatt-service"
    }

    private inner class ClientManager : BleManager(this@BleService) {
        override fun getMinLogPriority(): Int = Log.VERBOSE

        private var batteryCharacteristic: BluetoothGattCharacteristic? = null
        private var syncCharacteristic: BluetoothGattCharacteristic? = null
        private var stateMachineCharacteristic: BluetoothGattCharacteristic? = null
        private var animationFactorCharacteristic: BluetoothGattCharacteristic? = null
        private var angularVelocityCharacteristic: BluetoothGattCharacteristic? = null
        private var motionVectorCharacteristic: BluetoothGattCharacteristic? = null
        private var bodyTranslationCharacteristic: BluetoothGattCharacteristic? = null
        private var bodyRotationCharacteristic: BluetoothGattCharacteristic? = null
        private var legRadiusCharacteristic: BluetoothGattCharacteristic? = null
        private var batteryUpdateIntervalMsCharacteristic: BluetoothGattCharacteristic? = null
        private var getCalibrationForCharacteristic: BluetoothGattCharacteristic? = null
        private var getCalibrationResultCharacteristic: BluetoothGattCharacteristic? = null
        private var setCalibrationDatumCharacteristic: BluetoothGattCharacteristic? = null
        private var setCalibrationResultCharacteristic: BluetoothGattCharacteristic? = null


        override fun isRequiredServiceSupported(gatt: BluetoothGatt): Boolean {
            val bas = gatt.getService(MechaFerrisServiceProfile.basUuid)
            if (bas != null) {
                batteryCharacteristic =
                    bas.getCharacteristic(MechaFerrisServiceProfile.batteryLevelCharacteristicUuid)
            }
            val service = gatt.getService(MechaFerrisServiceProfile.controllerServiceUuid)
            if (service != null) {
                syncCharacteristic =
                    service.getCharacteristic(MechaFerrisServiceProfile.characteristicSync)
                stateMachineCharacteristic =
                    service.getCharacteristic(MechaFerrisServiceProfile.characteristicStateMachine)
                animationFactorCharacteristic =
                    service.getCharacteristic(MechaFerrisServiceProfile.characteristicAnimationFactor)
                angularVelocityCharacteristic =
                    service.getCharacteristic(MechaFerrisServiceProfile.characteristicAngularVelocity)
                motionVectorCharacteristic =
                    service.getCharacteristic(MechaFerrisServiceProfile.characteristicMotionVector)
                bodyTranslationCharacteristic =
                    service.getCharacteristic(MechaFerrisServiceProfile.characteristicBodyTranslation)
                bodyRotationCharacteristic =
                    service.getCharacteristic(MechaFerrisServiceProfile.characteristicBodyRotation)
                legRadiusCharacteristic =
                    service.getCharacteristic(MechaFerrisServiceProfile.characteristicLegRadius)
                batteryUpdateIntervalMsCharacteristic =
                    service.getCharacteristic(MechaFerrisServiceProfile.characteristicBatteryUpdateIntervalMs)
                getCalibrationForCharacteristic =
                    service.getCharacteristic(MechaFerrisServiceProfile.characteristicGetCalibrationFor)
                getCalibrationResultCharacteristic =
                    service.getCharacteristic(MechaFerrisServiceProfile.characteristicGetCalibrationResult)
                setCalibrationDatumCharacteristic =
                    service.getCharacteristic(MechaFerrisServiceProfile.characteristicSetCalibrationDatum)
                setCalibrationResultCharacteristic =
                    service.getCharacteristic(MechaFerrisServiceProfile.characteristicSetCalibrationResult)
            }

            return !(batteryCharacteristic == null || syncCharacteristic == null || stateMachineCharacteristic == null || animationFactorCharacteristic == null || angularVelocityCharacteristic == null || motionVectorCharacteristic == null || bodyTranslationCharacteristic == null || bodyRotationCharacteristic == null || legRadiusCharacteristic == null || batteryUpdateIntervalMsCharacteristic == null || getCalibrationForCharacteristic == null || getCalibrationResultCharacteristic == null || setCalibrationDatumCharacteristic == null || setCalibrationResultCharacteristic == null)
        }

        override fun initialize() {
            setNotificationCallback(batteryCharacteristic).with { _, data ->
                val batteryLevel = data.value?.let { RustLibrary.ReadBatteryLevel(it) }
                batteryLevel?.let {
                    defaultScope.launch {
                        batteryChangeNotifications?.send(it)
                    }
                }
            }
//                setNotificationCallback(syncCharacteristic).with { _, data ->
//                    val sync = data.value?.let { RustLibrary.ReadSync(it) }
//                    sync?.let {
//                        defaultScope.launch {
//                            syncChangeNotifications?.send(it)
//                        }
//                    }
//                }
            setNotificationCallback(stateMachineCharacteristic).with { _, data ->
                val stateMachine = data.value?.let { RustLibrary.ReadState(it) }
                stateMachine?.let {
                    defaultScope.launch {
                        stateMachineChangeNotifications?.send(it)
                    }
                }
            }
            setNotificationCallback(animationFactorCharacteristic).with { _, data ->
                val animationFactor = data.value?.let {
                    RustLibrary.ReadAnimationFactor(it)
                }
                animationFactor?.let {
                    defaultScope.launch {
                        animationFactorChangeNotifications?.send(it)
                    }
                }
            }
            setNotificationCallback(angularVelocityCharacteristic).with { _, data ->
                val angularVelocity = data.value?.let {
                    RustLibrary.ReadAngularVelocity(it)
                }
                angularVelocity?.let {
                    defaultScope.launch {
                        angularVelocityChangeNotifications?.send(it)
                    }
                }
            }
            setNotificationCallback(motionVectorCharacteristic).with { _, data ->
                val motionVector = data.value?.let { RustLibrary.ReadMotionVector(it) }
                motionVector?.let {
                    defaultScope.launch {
                        motionVectorChangeNotifications?.send(it)
                    }
                }
            }
            setNotificationCallback(bodyTranslationCharacteristic).with { _, data ->
                val bodyTranslation = data.value?.let {
                    RustLibrary.ReadBodyTranslation(it)
                }
                bodyTranslation?.let {
                    defaultScope.launch {
                        bodyTranslationChangeNotifications?.send(it)
                    }
                }
            }
            setNotificationCallback(bodyRotationCharacteristic).with { _, data ->
                val bodyRotation = data.value?.let {
                    RustLibrary.ReadBodyRotation(it)
                }
                bodyRotation?.let {
                    defaultScope.launch {
                        bodyRotationChangeNotifications?.send(it)
                    }
                }
            }
            setNotificationCallback(legRadiusCharacteristic).with { _, data ->
                val legRadius = data.value?.let {
                    RustLibrary.ReadLegRadius(it)
                }
                legRadius?.let {
                    defaultScope.launch {
                        legRadiusChangeNotifications?.send(it)
                    }
                }
            }
            setNotificationCallback(batteryUpdateIntervalMsCharacteristic).with { _, data ->
                val batteryUpdateIntervalMs =
                    data.value?.let { RustLibrary.ReadBatteryUpdateInterval(it) }
                batteryUpdateIntervalMs?.let {
                    defaultScope.launch {
                        batteryUpdateIntervalMsChangeNotifications?.send(it)
                    }
                }
            }
            setNotificationCallback(getCalibrationResultCharacteristic).with { _, data ->
                val calibrationData = data.value?.let { RustLibrary.ReadGetCalibrationResult(it) }
                calibrationData?.let {
                    defaultScope.launch {
                        getCalibrationResultChangeNotifications?.send(it)
                    }
                }
            }
            setNotificationCallback(setCalibrationResultCharacteristic).with { _, data ->
                val calibrationResult = data.value?.let { RustLibrary.ReadSetCalibrationResult(it) }
                calibrationResult?.let {
                    defaultScope.launch {
                        setCalibrationResultChangeNotifications?.send(it)
                    }
                }
            }

            beginAtomicRequestQueue().add(enableNotifications(batteryCharacteristic).done {
                Log.i(
                    TAG, "Enabled battery notifications"
                )
            }.fail { _: BluetoothDevice?, status: Int ->
                log(Log.ERROR, "Could not subscribe to battery: $status")
                disconnect().enqueue()
            }).add(enableNotifications(stateMachineCharacteristic).done {
                Log.i(
                    TAG, "Enabled stateMachine notifications"
                )
            }.fail { _: BluetoothDevice?, status: Int ->
                log(Log.ERROR, "Could not subscribe to stateMachine: $status")
                disconnect().enqueue()
            }).add(enableNotifications(animationFactorCharacteristic).done {
                Log.i(
                    TAG, "Enabled animation factor notifications"
                )
            }.fail { _: BluetoothDevice?, status: Int ->
                log(Log.ERROR, "Could not subscribe to animation factor: $status")
                disconnect().enqueue()
            }).add(enableNotifications(angularVelocityCharacteristic).done {
                Log.i(
                    TAG, "Enabled angularVelocity notifications"
                )
            }.fail { _: BluetoothDevice?, status: Int ->
                log(Log.ERROR, "Could not subscribe to angularVelocity: $status")
                disconnect().enqueue()
            }).add(enableNotifications(motionVectorCharacteristic).done {
                Log.i(
                    TAG, "Enabled motionVector notifications"
                )
            }.fail { _: BluetoothDevice?, status: Int ->
                log(Log.ERROR, "Could not subscribe to motionVector: $status")
                disconnect().enqueue()
            }).add(enableNotifications(bodyTranslationCharacteristic).done {
                Log.i(
                    TAG, "Enabled bodyTranslation notifications"
                )
            }.fail { _: BluetoothDevice?, status: Int ->
                log(Log.ERROR, "Could not subscribe to bodyTranslation: $status")
                disconnect().enqueue()
            }).add(enableNotifications(bodyRotationCharacteristic).done {
                Log.i(
                    TAG, "Enabled bodyRotation notifications"
                )
            }.fail { _: BluetoothDevice?, status: Int ->
                log(Log.ERROR, "Could not subscribe to bodyRotation: $status")
                disconnect().enqueue()
            }).add(enableNotifications(legRadiusCharacteristic).done {
                Log.i(
                    TAG, "Enabled legRadius notifications"
                )
            }.fail { _: BluetoothDevice?, status: Int ->
                log(Log.ERROR, "Could not subscribe to legRadius: $status")
                disconnect().enqueue()
            }).add(enableNotifications(batteryUpdateIntervalMsCharacteristic).done {
                Log.i(
                    TAG, "Enabled batteryUpdateIntervalMs notifications"
                )
            }.fail { _: BluetoothDevice?, status: Int ->
                log(
                    Log.ERROR, "Could not subscribe to batteryUpdateInterval: $status"
                )
                disconnect().enqueue()
            }).add(enableNotifications(getCalibrationResultCharacteristic).done {
                Log.i(
                    TAG, "Enabled getCalibrationData notifications"
                )
            }.fail { _: BluetoothDevice?, status: Int ->
                log(Log.ERROR, "Could not subscribe to getCalibrationData: $status")
                disconnect().enqueue()
            }).add(enableNotifications(setCalibrationResultCharacteristic).done {
                Log.i(
                    TAG, "Enabled setCalibrationData notifications"
                )
            }.fail { _: BluetoothDevice?, status: Int ->
                log(Log.ERROR, "Could not subscribe to setCalibrationData: $status")
                disconnect().enqueue()
            }).done {
                Log.i(TAG, "Target initialized")
            }.enqueue()
        }

        override fun onServicesInvalidated() {
            batteryCharacteristic = null
            syncCharacteristic = null
            stateMachineCharacteristic = null
            animationFactorCharacteristic = null
            angularVelocityCharacteristic = null
            motionVectorCharacteristic = null
            bodyTranslationCharacteristic = null
            bodyRotationCharacteristic = null
            legRadiusCharacteristic = null
            batteryUpdateIntervalMsCharacteristic = null
            getCalibrationForCharacteristic = null
            getCalibrationResultCharacteristic = null
            setCalibrationDatumCharacteristic = null
            setCalibrationResultCharacteristic = null
        }

        override fun onDeviceReady() {
            log(Log.INFO, "Target ready")
        }

        override fun onManagerReady() {
            log(Log.INFO, "Manager ready")
            bluetoothDevice?.let {
                onDeviceFound?.invoke(it)
            }
        }

        suspend fun setBatteryLevel(batteryLevel: UInt) {
            suspendCancellableCoroutine { continuation ->
                writeCharacteristic(
                    batteryCharacteristic,
                    RustLibrary.SendBatteryLevel(batteryLevel),
                    BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                ).done { continuation.resumeWith(Result.success(Unit)) }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not set battery level: $status"))) }
                    .enqueue()
            }
        }

        suspend fun setStateMachine(stateMachine: StateMachine) {
            suspendCancellableCoroutine { continuation ->
                writeCharacteristic(
                    stateMachineCharacteristic,
                    RustLibrary.SendState(stateMachine),
                    BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                ).done { continuation.resumeWith(Result.success(Unit)) }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not set state machine: $status"))) }
                    .enqueue()
            }
        }

        suspend fun setAnimationFactor(animationFactor: Float) {
            suspendCancellableCoroutine { continuation ->
                writeCharacteristic(
                    animationFactorCharacteristic,
                    RustLibrary.SendAnimationFactor(animationFactor),
                    BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                ).done { continuation.resumeWith(Result.success(Unit)) }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not set animationFactor: $status"))) }
                    .enqueue()
            }
        }

        suspend fun setAngularVelocity(angularVelocity: Float) {
            suspendCancellableCoroutine { continuation ->
                writeCharacteristic(
                    angularVelocityCharacteristic,
                    RustLibrary.SendAngularVelocity(angularVelocity),
                    BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                ).done { continuation.resumeWith(Result.success(Unit)) }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not set angular velocity: $status"))) }
                    .enqueue()
            }
        }

        suspend fun setMotionVector(motionVector: Vector3) {
            suspendCancellableCoroutine { continuation ->
                writeCharacteristic(
                    motionVectorCharacteristic,
                    RustLibrary.SendMotionVector(motionVector),
                    BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                ).done { continuation.resumeWith(Result.success(Unit)) }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not set motion vector: $status"))) }
                    .enqueue()
            }
        }

        suspend fun setBodyTranslation(bodyTranslation: Translation) {
            suspendCancellableCoroutine { continuation ->
                writeCharacteristic(
                    bodyTranslationCharacteristic,
                    RustLibrary.SendBodyTranslation(bodyTranslation),
                    BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                ).done { continuation.resumeWith(Result.success(Unit)) }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not set body translation: $status"))) }
                    .enqueue()
            }
        }

        suspend fun setBodyRotation(bodyRotation: Quaternion) {
            suspendCancellableCoroutine { continuation ->
                writeCharacteristic(
                    bodyRotationCharacteristic,
                    RustLibrary.SendBodyRotation(bodyRotation),
                    BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                ).done { continuation.resumeWith(Result.success(Unit)) }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not set body rotation: $status"))) }
                    .enqueue()
            }
        }

        suspend fun setLegRadius(legRadius: Float) {
            suspendCancellableCoroutine { continuation ->
                writeCharacteristic(
                    legRadiusCharacteristic,
                    RustLibrary.SendLegRadius(legRadius),
                    BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                ).done { continuation.resumeWith(Result.success(Unit)) }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not set leg radius: $status"))) }
                    .enqueue()
            }
        }

        suspend fun setBatteryUpdateIntervalMs(batteryUpdateIntervalMs: UInt) {
            suspendCancellableCoroutine { continuation ->
                writeCharacteristic(
                    batteryUpdateIntervalMsCharacteristic,
                    RustLibrary.SendBatteryUpdateInterval(batteryUpdateIntervalMs),
                    BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                ).done { continuation.resumeWith(Result.success(Unit)) }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not set battery update interval: $status"))) }
                    .enqueue()
            }
        }

        suspend fun setCalibrationPulse(leg: UByte, joint: UByte, kind: UByte, pulse: Float) {
            suspendCancellableCoroutine { continuation ->
                writeCharacteristic(
                    setCalibrationDatumCharacteristic, RustLibrary.SendSetCalibrationDatum(
                        CalibrationDatum(
                            CalibrationIndex(
                                leg, joint, kind
                            ), pulse
                        )
                    ), WRITE_TYPE_DEFAULT
                ).done { continuation.resumeWith(Result.success(Unit)) }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not set calibration pulse: $status"))) }
                    .enqueue()
            }
        }

        @SuppressLint("MissingPermission")
        suspend fun sync(all: Boolean) {
            suspendCancellableCoroutine { continuation ->
                Log.d(
                    TAG,
                    "Sending sync request to ${this.bluetoothDevice?.name}: ${this.bluetoothDevice?.address}"
                )
                writeCharacteristic(
                    syncCharacteristic,
                    RustLibrary.SendSync(all),
                    BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                ).done {
                    Log.d(TAG, "Sync completed")
                    continuation.resumeWith(Result.success(Unit))
                }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not sync: $status"))) }
                    .enqueue()
            }
        }

        suspend fun getBatteryLevel(): UInt? {
            return suspendCancellableCoroutine { continuation ->
                readCharacteristic(batteryCharacteristic).with { _, data ->
                    continuation.resumeWith(Result.success(data.value?.let {
                        RustLibrary.ReadBatteryLevel(it)
                    }))
                }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not get battery level: $status"))) }
                    .enqueue()
            }
        }

        suspend fun getStateMachine(): StateMachine? {
            return suspendCancellableCoroutine { continuation ->
                readCharacteristic(stateMachineCharacteristic).with { _, data ->
                    continuation.resumeWith(
                        Result.success(data.value?.let {
                            RustLibrary.ReadState(it)
                        })
                    )
                }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not get state machine: $status"))) }
                    .enqueue()
            }
        }

        suspend fun getAnimationFactor(): Float? {
            return suspendCancellableCoroutine { continuation ->
                readCharacteristic(animationFactorCharacteristic).with { _, data ->
                    continuation.resumeWith(
                        Result.success(data.value?.let {
                            RustLibrary.ReadAnimationFactor(it)
                        })
                    )
                }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not get animation factor: $status"))) }
                    .enqueue()
            }
        }

        suspend fun getAngularVelocity(): Float? {
            return suspendCancellableCoroutine { continuation ->
                readCharacteristic(angularVelocityCharacteristic).with { _, data ->
                    continuation.resumeWith(Result.success(data.value?.let {
                        RustLibrary.ReadAngularVelocity(it)
                    }))
                }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not get angular velocity: $status"))) }
                    .enqueue()
            }
        }

        suspend fun getMotionVector(): Vector3? {
            return suspendCancellableCoroutine { continuation ->
                readCharacteristic(motionVectorCharacteristic).with { _, data ->
                    continuation.resumeWith(Result.success(data.value?.let {
                        RustLibrary.ReadMotionVector(it)
                    }))
                }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not get motion vector: $status"))) }
                    .enqueue()
            }
        }

        suspend fun getBodyTranslation(): Translation? {
            return suspendCancellableCoroutine { continuation ->
                readCharacteristic(bodyTranslationCharacteristic).with { _, data ->
                    Log.i("BodyTranslation", data.toString())
                    continuation.resumeWith(Result.success(data.value?.let {
                        RustLibrary.ReadBodyTranslation(it)
                    }))
                }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not get body translation: $status"))) }
                    .enqueue()
            }
        }

        suspend fun getBodyRotation(): Quaternion? {
            return suspendCancellableCoroutine { continuation ->
                readCharacteristic(bodyRotationCharacteristic).with { _, data ->
                    continuation.resumeWith(Result.success(data.value?.let {
                        RustLibrary.ReadBodyRotation(it)
                    }))
                }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not get body rotation: $status"))) }
                    .enqueue()
            }
        }

        suspend fun getLegRadius(): Float? {
            return suspendCancellableCoroutine { continuation ->
                readCharacteristic(legRadiusCharacteristic).with { _, data ->
                    continuation.resumeWith(Result.success(data.value?.let {
                        RustLibrary.ReadLegRadius(it)
                    }))
                }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not get leg radius: $status"))) }
                    .enqueue()
            }
        }

        suspend fun getBatteryUpdateIntervalMs(): UInt? {
            return suspendCancellableCoroutine { continuation ->
                readCharacteristic(batteryUpdateIntervalMsCharacteristic).with { _, data ->
                    continuation.resumeWith(Result.success(data.value?.let {
                        RustLibrary.ReadBatteryUpdateInterval(it)
                    }))
                }
                    .fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not get battery update interval: $status"))) }
                    .enqueue()
            }
        }

        suspend fun requestCalibrationPulse(leg: UByte, joint: UByte, kind: UByte) {
            return suspendCancellableCoroutine { continuation ->
                writeCharacteristic(
                    getCalibrationForCharacteristic,
                    Data(RustLibrary.SendGetCalibrationFor(CalibrationIndex(leg, joint, kind))),
                    WRITE_TYPE_DEFAULT
                ).fail { _, status -> continuation.resumeWith(Result.failure(Exception("Could not request calibration pulse: $status"))) }
                    .enqueue()
            }
        }
    }

    object MechaFerrisServiceProfile {
        val basUuid: UUID = UUID.fromString("0000180f-0000-1000-8000-00805f9b34fb")
        val batteryLevelCharacteristicUuid: UUID =
            UUID.fromString("00002a19-0000-1000-8000-00805f9b34fb")
        val controllerServiceUuid: UUID = UUID.fromString("77144c32-0ed7-469b-a3d3-0f2c9157333a")
        val characteristicStateMachine: UUID =
            UUID.fromString("f125c904-a0e2-4885-817c-55d7463630db")
        val characteristicSync: UUID = UUID.fromString("b0115f52-c6fd-4ea4-94cf-21626b9e3469")
        val characteristicAnimationFactor: UUID =
            UUID.fromString("7566bd93-3712-4850-8868-88ce30f6acc1")
        val characteristicAngularVelocity: UUID =
            UUID.fromString("6be80625-e69b-418e-bf01-9abc617cdd9f")
        val characteristicMotionVector: UUID =
            UUID.fromString("77d6f220-7057-4dc6-8746-8a23b06e53d6")
        val characteristicBodyTranslation: UUID =
            UUID.fromString("cde4ce10-edc2-44af-bd14-60865d30f2b6")
        val characteristicBodyRotation: UUID =
            UUID.fromString("ccfb948a-3421-47ed-a0c0-b84f7d307027")
        val characteristicLegRadius: UUID = UUID.fromString("2735f1d0-b944-4efe-960c-10380d061052")
        val characteristicBatteryUpdateIntervalMs: UUID =
            UUID.fromString("d007632f-10e5-427a-b158-482aeb48b90e")
        val characteristicGetCalibrationFor: UUID =
            UUID.fromString("22b55af4-32f7-41bc-88f8-021e5dce9f60")
        val characteristicGetCalibrationResult: UUID =
            UUID.fromString("be20dff1-36a5-49de-a7c8-cec51bae2c4e")
        val characteristicSetCalibrationDatum: UUID =
            UUID.fromString("f34c2b97-e619-4ed8-ae1b-3e623855bb8e")
        val characteristicSetCalibrationResult: UUID =
            UUID.fromString("a9b70415-bb6c-4aca-aeb3-ac93c6087910")
    }
}