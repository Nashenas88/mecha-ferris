@file:OptIn(ExperimentalUnsignedTypes::class)

package paulfaria.mechaferris

import android.util.Log
import com.juul.kable.Filter
import com.juul.kable.Peripheral
import com.juul.kable.Scanner
import com.juul.kable.WriteType
import com.juul.kable.characteristicOf
import com.juul.kable.logs.Logging
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map
import java.util.UUID

const val mechaFerrisUuid = "a6772cc6-302c-4de3-abcd-ce702762881e"
const val basUuid = "0000180f-0000-1000-8000-00805f9b34fb"
const val batteryLevelCharacteristicUuid = "00002a19-0000-1000-8000-00805f9b34fb"
const val controllerServiceUuid = "77144c32-0ed7-469b-a3d3-0f2c9157333a"
const val characteristicStateMachineUuid = "f125c904-a0e2-4885-817c-55d7463630db"
const val characteristicNotifyAllUuid = "3f6b77c2-0205-4a77-a809-2479b9533664"
const val characteristicSyncUuid = "b0115f52-c6fd-4ea4-94cf-21626b9e3469"
const val characteristicAnimationFactorUuid = "7566bd93-3712-4850-8868-88ce30f6acc1"
const val characteristicAngularVelocityUuid = "6be80625-e69b-418e-bf01-9abc617cdd9f"
const val characteristicMotionVectorUuid = "77d6f220-7057-4dc6-8746-8a23b06e53d6"
const val characteristicBodyTranslationUuid = "cde4ce10-edc2-44af-bd14-60865d30f2b6"
const val characteristicBodyRotationUuid = "ccfb948a-3421-47ed-a0c0-b84f7d307027"
const val characteristicLegRadiusUuid = "2735f1d0-b944-4efe-960c-10380d061052"
const val characteristicBatteryUpdateIntervalMsUuid = "d007632f-10e5-427a-b158-482aeb48b90e"
const val characteristicGetCalibrationForUuid = "22b55af4-32f7-41bc-88f8-021e5dce9f60"
const val characteristicGetCalibrationResultUuid = "be20dff1-36a5-49de-a7c8-cec51bae2c4e"
const val characteristicSetCalibrationDatumUuid = "f34c2b97-e619-4ed8-ae1b-3e623855bb8e"
const val characteristicSetCalibrationResultUuid = "a9b70415-bb6c-4aca-aeb3-ac93c6087910"

private val batteryLevelCharacteristic = characteristicOf(
    service = UUID.fromString(basUuid),
    characteristic = UUID.fromString(batteryLevelCharacteristicUuid),
)
private val stateMachineCharacteristic = characteristicOf(
    service = UUID.fromString(controllerServiceUuid),
    characteristic = UUID.fromString(characteristicStateMachineUuid),
)
private val notifyAllCharacteristic = characteristicOf(
    service = UUID.fromString(controllerServiceUuid),
    characteristic = UUID.fromString(characteristicNotifyAllUuid),
)
private val syncCharacteristic = characteristicOf(
    service = UUID.fromString(controllerServiceUuid),
    characteristic = UUID.fromString(characteristicSyncUuid),
)
private val animationFactorCharacteristic = characteristicOf(
    service = UUID.fromString(controllerServiceUuid),
    characteristic = UUID.fromString(characteristicAnimationFactorUuid),
)
private val angularVelocityCharacteristic = characteristicOf(
    service = UUID.fromString(controllerServiceUuid),
    characteristic = UUID.fromString(characteristicAngularVelocityUuid),
)
private val motionVectorCharacteristic = characteristicOf(
    service = UUID.fromString(controllerServiceUuid),
    characteristic = UUID.fromString(characteristicMotionVectorUuid),
)
private val bodyTranslationCharacteristic = characteristicOf(
    service = UUID.fromString(controllerServiceUuid),
    characteristic = UUID.fromString(characteristicBodyTranslationUuid),
)
private val bodyRotationCharacteristic = characteristicOf(
    service = UUID.fromString(controllerServiceUuid),
    characteristic = UUID.fromString(characteristicBodyRotationUuid),
)
private val legRadiusCharacteristic = characteristicOf(
    service = UUID.fromString(controllerServiceUuid),
    characteristic = UUID.fromString(characteristicLegRadiusUuid),
)
private val batteryUpdateIntervalMsCharacteristic = characteristicOf(
    service = UUID.fromString(controllerServiceUuid),
    characteristic = UUID.fromString(characteristicBatteryUpdateIntervalMsUuid),
)
private val getCalibrationForCharacteristic = characteristicOf(
    service = UUID.fromString(controllerServiceUuid),
    characteristic = UUID.fromString(characteristicGetCalibrationForUuid),
)
private val getCalibrationResultCharacteristic = characteristicOf(
    service = UUID.fromString(controllerServiceUuid),
    characteristic = UUID.fromString(characteristicGetCalibrationResultUuid),
)
private val setCalibrationDatumCharacteristic = characteristicOf(
    service = UUID.fromString(controllerServiceUuid),
    characteristic = UUID.fromString(characteristicSetCalibrationDatumUuid),
)
private val setCalibrationResultCharacteristic = characteristicOf(
    service = UUID.fromString(controllerServiceUuid),
    characteristic = UUID.fromString(characteristicSetCalibrationResultUuid),
)

val scanner = Scanner {
    logging {
        level = Logging.Level.Events
    }
    filters = listOf(Filter.Service(UUID.fromString(mechaFerrisUuid)))
//    scanSettings = ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build()
}

class MechaFerris(private val peripheral: Peripheral) : Peripheral by peripheral {
    val batteryLevel: Flow<UInt> = peripheral
        .observe(batteryLevelCharacteristic)
        .map { RustLibrary.ReadBatteryLevel(it) }
        .map { Log.i("Scanner", "Got battery level: {it}"); it }

    val stateMachine: Flow<StateMachine> = peripheral
        .observe(stateMachineCharacteristic)
        .map {
            Log.i("Scanner", "Got state machine")
            RustLibrary.ReadState(it)
        }
        .map {
            Log.i("Scanner", "Got state machine: $it")
            it
        }

    val animationFactor: Flow<Float> = peripheral
        .observe(animationFactorCharacteristic) {
            Log.i("Scanner", "Got animation factor")
        }
        .map { RustLibrary.ReadAnimationFactor(it) }
        .map { Log.i("Scanner", "Got animation factor: $it"); it }

    val angularVelocity: Flow<Float> = peripheral
        .observe(angularVelocityCharacteristic) {
            Log.i("Scanner", "Got angular velocity")
        }
        .map { RustLibrary.ReadAngularVelocity(it) }
        .map { Log.i("Scanner", "Got angular velocity: $it"); it }

    val motionVector: Flow<Vector3> = peripheral
        .observe(motionVectorCharacteristic) {
            Log.i("Scanner", "Got motion vector")
        }
        .map { RustLibrary.ReadMotionVector(it) }
        .map { Log.i("Scanner", "Got motion vector: $it"); it }

    val bodyTranslation: Flow<Translation> = peripheral
        .observe(bodyTranslationCharacteristic) {
            Log.i("Scanner", "Got body translation")
        }
        .map { RustLibrary.ReadBodyTranslation(it) }
        .map { Log.i("Scanner", "Got body translation: $it"); it }

    val bodyRotation: Flow<Quaternion> = peripheral
        .observe(bodyRotationCharacteristic) {
            Log.i("Scanner", "Got body rotation")
        }
        .map { RustLibrary.ReadBodyRotation(it) }
        .map { Log.i("Scanner", "Got body rotation: $it"); it }

    val legRadius: Flow<Float> = peripheral
        .observe(legRadiusCharacteristic) {
            Log.i("Scanner", "Got leg radius")
        }
        .map { RustLibrary.ReadLegRadius(it) }
        .map { Log.i("Scanner", "Got leg radius: $it"); it }

    val batteryUpdateIntervalMs: Flow<UInt> = peripheral
        .observe(batteryUpdateIntervalMsCharacteristic) {
            Log.i("Scanner", "Got battery update interval")
        }
        .map { RustLibrary.ReadBatteryUpdateInterval(it) }

    val getCalibrationResult: Flow<CalibrationDatum> = peripheral
        .observe(getCalibrationResultCharacteristic) {
            Log.i("Scanner", "Got get calibration result")
        }
        .map { RustLibrary.ReadGetCalibrationResult(it) }

    val setCalibrationResult: Flow<SetResult> = peripheral
        .observe(setCalibrationResultCharacteristic) {
            Log.i("Scanner", "Got set calibration result")
        }
        .map {
            val res = RustLibrary.ReadSetCalibrationResult(it)
            Log.i("Scanner", "Got set calibration result: $res")
            res
        }

    suspend fun notifyAll() {
        val bytes = RustLibrary.SendSync(true);
        peripheral.write(notifyAllCharacteristic, bytes, WriteType.WithResponse)
    }

    suspend fun sync(all: Boolean) {
        val bytes = RustLibrary.SendSync(all)
        peripheral.write(syncCharacteristic, bytes, WriteType.WithResponse)
    }

    @OptIn(ExperimentalUnsignedTypes::class)
    suspend fun setStateMachine(stateMachine: StateMachine) {
        val bytes = RustLibrary.SendState(stateMachine)
        peripheral.write(stateMachineCharacteristic, bytes, WriteType.WithResponse)
    }

    suspend fun setAnimationFactor(animationFactor: Float) {
        val bytes = RustLibrary.SendAnimationFactor(animationFactor)
        peripheral.write(animationFactorCharacteristic, bytes, WriteType.WithResponse)
    }

    suspend fun setAngularVelocity(angularVelocity: Float) {
        val bytes = RustLibrary.SendAngularVelocity(angularVelocity)
        peripheral.write(angularVelocityCharacteristic, bytes, WriteType.WithResponse)
    }

    suspend fun setMotionVector(motionVector: Vector3) {
        val bytes = RustLibrary.SendMotionVector(motionVector)
        peripheral.write(motionVectorCharacteristic, bytes, WriteType.WithResponse)
    }

    suspend fun setBodyTranslation(bodyTranslation: Translation) {
        val bytes = RustLibrary.SendBodyTranslation(bodyTranslation)
        peripheral.write(bodyTranslationCharacteristic, bytes, WriteType.WithResponse)
    }

    suspend fun setBodyRotation(bodyRotation: Quaternion) {
        val bytes = RustLibrary.SendBodyRotation(bodyRotation)
        peripheral.write(bodyRotationCharacteristic, bytes, WriteType.WithResponse)
    }

    suspend fun setLegRadius(legRadius: Float) {
        val bytes = RustLibrary.SendLegRadius(legRadius)
        peripheral.write(legRadiusCharacteristic, bytes, WriteType.WithResponse)
    }

    @OptIn(ExperimentalUnsignedTypes::class)
    suspend fun setBatteryUpdateIntervalMs(batteryUpdateIntervalMs: UInt) {
        val bytes = RustLibrary.SendBatteryUpdateInterval(batteryUpdateIntervalMs)
        peripheral.write(batteryUpdateIntervalMsCharacteristic, bytes, WriteType.WithResponse)
    }

    suspend fun setCalibrationPulse(leg: UByte, joint: UByte, kind: UByte, pulse: Float) {
        val bytes = RustLibrary.SendSetCalibrationDatum(
            CalibrationDatum(
                CalibrationIndex(leg, joint, kind),
                pulse
            )
        )
        peripheral.write(setCalibrationDatumCharacteristic, bytes, WriteType.WithResponse)
    }

    suspend fun getBatteryLevel(): UInt {
        val bytes = peripheral.read(batteryLevelCharacteristic)
        return RustLibrary.ReadBatteryLevel(bytes)
    }

    suspend fun getStateMachine(): StateMachine {
        val bytes = peripheral.read(stateMachineCharacteristic)
        return RustLibrary.ReadState(bytes)
    }

    suspend fun getAnimationFactor(): Float {
        val bytes = peripheral.read(animationFactorCharacteristic)
        return RustLibrary.ReadAnimationFactor(bytes)
    }

    suspend fun getAngularVelocity(): Float {
        val bytes = peripheral.read(angularVelocityCharacteristic)
        return RustLibrary.ReadAngularVelocity(bytes)
    }

    suspend fun getMotionVector(): Vector3 {
        val bytes = peripheral.read(motionVectorCharacteristic)
        return RustLibrary.ReadMotionVector(bytes)
    }

    suspend fun getBodyTranslation(): Translation {
        val bytes = peripheral.read(bodyTranslationCharacteristic)
        return RustLibrary.ReadBodyTranslation(bytes)
    }

    suspend fun getBodyRotation(): Quaternion {
        val bytes = peripheral.read(bodyRotationCharacteristic)
        return RustLibrary.ReadBodyRotation(bytes)
    }

    suspend fun getLegRadius(): Float {
        val bytes = peripheral.read(legRadiusCharacteristic)
        return RustLibrary.ReadLegRadius(bytes)
    }

    suspend fun getBatteryUpdateIntervalMs(): UInt {
        val bytes = peripheral.read(batteryUpdateIntervalMsCharacteristic)
        return RustLibrary.ReadBatteryUpdateInterval(bytes)
    }

    suspend fun requestCalibrationData(leg: UByte, joint: UByte, kind: UByte) {
        val bytes = RustLibrary.SendGetCalibrationFor(CalibrationIndex(leg, joint, kind))
        peripheral.write(getCalibrationForCharacteristic, bytes, WriteType.WithResponse)
    }

    suspend fun getCalibrationResult(): CalibrationDatum {
        val bytes = peripheral.read(getCalibrationResultCharacteristic)
        return RustLibrary.ReadGetCalibrationResult(bytes)
    }

    suspend fun setCalibrationResult(result: SetResult) {
        val bytes = RustLibrary.SendSetCalibrationResult(result)
        peripheral.write(setCalibrationResultCharacteristic, bytes, WriteType.WithResponse)
    }
}

private fun characteristicOf(service: UUID, characteristic: UUID) =
    characteristicOf(service.toString(), characteristic.toString())