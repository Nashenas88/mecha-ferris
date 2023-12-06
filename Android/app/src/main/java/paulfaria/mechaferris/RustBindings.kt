package paulfaria.mechaferris

@ExperimentalUnsignedTypes
enum class StateMachine(val r: UByte) {
    /// The robot is not moving.
    PAUSED(1u),
    /// The robot is homing.
    HOMING(2u),
    /// The robot is calibrating.
    CALIBRATING(3u),
    /// The robot is running a simple walking loop.
    LOOPING(4u),
    /// The robot is exploring its environment.
    EXPLORING(5u),
}

data class Vector3(val x: Float, val y: Float, val z: Float)
data class Translation(val x: Float, val y: Float, val z: Float)
data class Quaternion(val x: Float, val y: Float, val z: Float, val w: Float)

@ExperimentalUnsignedTypes
data class CalibrationIndex(val leg: UByte, val joint: UByte, val kind: UByte)

@ExperimentalUnsignedTypes
data class CalibrationDatum(val index: CalibrationIndex, val pulse: Float)

@ExperimentalUnsignedTypes
enum class SetRes(val r: UByte) {
    Ok(0u),
    InvalidIndex(1u),
    InvalidValue(2u),
    InvalidKind(3u),
    InvalidLeg(4u),
    InvalidJoint(5u),
}

@ExperimentalUnsignedTypes
data class SetResult(val index: CalibrationIndex, val result: SetRes)

class RustLibrary {
    companion object {
        init {
            System.loadLibrary("rust_kotlin")
        }

        external fun ReadBatteryLevel(input: ByteArray): UInt

        external fun SendSync(value: Boolean): ByteArray

        external fun ReadSync(input: ByteArray): Boolean

        @ExperimentalUnsignedTypes
        external fun SendState(value: StateMachine): ByteArray

        @ExperimentalUnsignedTypes
        external fun ReadState(input: ByteArray): StateMachine

        external fun SendAnimationFactor(value: Float): ByteArray

        external fun ReadAnimationFactor(input: ByteArray): Float

        external fun SendAngularVelocity(value: Float): ByteArray

        external fun ReadAngularVelocity(input: ByteArray): Float

        external fun SendMotionVector(value: Vector3): ByteArray

        external fun ReadMotionVector(input: ByteArray): Vector3

        external fun SendBodyTranslation(value: Translation): ByteArray

        external fun ReadBodyTranslation(input: ByteArray): Translation

        external fun SendBodyRotation(value: Quaternion): ByteArray

        external fun ReadBodyRotation(input: ByteArray): Quaternion

        external fun SendLegRadius(value: Float): ByteArray

        external fun ReadLegRadius(input: ByteArray): Float

        @ExperimentalUnsignedTypes
        external fun SendBatteryUpdateInterval(value: UInt): ByteArray

        @ExperimentalUnsignedTypes
        external fun ReadBatteryUpdateInterval(input: ByteArray): UInt

        @ExperimentalUnsignedTypes
        external fun SendGetCalibrationFor(value: CalibrationIndex): ByteArray

        @ExperimentalUnsignedTypes
        external fun ReadGetCalibrationFor(input: ByteArray): CalibrationIndex

        @ExperimentalUnsignedTypes
        external fun SendGetCalibrationResult(value: CalibrationDatum): ByteArray

        @ExperimentalUnsignedTypes
        external fun ReadGetCalibrationResult(input: ByteArray): CalibrationDatum

        @ExperimentalUnsignedTypes
        external fun SendSetCalibrationDatum(value: CalibrationDatum): ByteArray

        @ExperimentalUnsignedTypes
        external fun ReadSetCalibrationDatum(input: ByteArray): CalibrationDatum

        @ExperimentalUnsignedTypes
        external fun SendSetCalibrationResult(value: SetResult): ByteArray

        @ExperimentalUnsignedTypes
        external fun ReadSetCalibrationResult(input: ByteArray): SetResult
    }
}
