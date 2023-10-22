package paulfaria.mechaferris

import androidx.test.ext.junit.runners.AndroidJUnit4
import junit.framework.TestCase
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import kotlin.random.Random

/**
 * Instrumented test, which will execute on an Android device.
 *
 * See [testing documentation](http://d.android.com/tools/testing).
 */
@OptIn(ExperimentalUnsignedTypes::class)
@RunWith(AndroidJUnit4::class)
class RustLibraryTest {

    @Test
    fun testRustLibrary() {
        // TODO: split into multiple tests
        for (v in arrayOf(true, false)) {
            val buf = RustLibrary.SendSync(v)
            val res = RustLibrary.ReadSync(buf)
            TestCase.assertEquals(v, res)
        }

        for (v in StateMachine.values()) {
            val buf = RustLibrary.SendState(v)
            val res = RustLibrary.ReadState(buf)
            TestCase.assertEquals(v, res)
        }

        run {
            val v = Random.nextFloat()
            val buf = RustLibrary.SendAnimationFactor(v)
            val res = RustLibrary.ReadAnimationFactor(buf)
            TestCase.assertEquals(v, res)
        }

        run {
            val v = Vector3(Random.nextFloat(), Random.nextFloat(), Random.nextFloat())
            val buf = RustLibrary.SendMotionVector(v)
            val res = RustLibrary.ReadMotionVector(buf)
            TestCase.assertEquals(v.x, res.x)
            TestCase.assertEquals(v.y, res.y)
            TestCase.assertEquals(v.z, res.z)
        }

        run {
            val v = Translation(Random.nextFloat(), Random.nextFloat(), Random.nextFloat())
            val buf = RustLibrary.SendBodyTranslation(v)
            val res = RustLibrary.ReadBodyTranslation(buf)
            TestCase.assertEquals(v.x, res.x)
            TestCase.assertEquals(v.y, res.y)
            TestCase.assertEquals(v.z, res.z)
        }

        run {
            val v = Quaternion(
                Random.nextFloat(),
                Random.nextFloat(),
                Random.nextFloat(),
                Random.nextFloat()
            )
            val buf = RustLibrary.SendBodyRotation(v)
            val res = RustLibrary.ReadBodyRotation(buf)
            TestCase.assertEquals(v.w, res.w)
            TestCase.assertEquals(v.x, res.x)
            TestCase.assertEquals(v.y, res.y)
            TestCase.assertEquals(v.z, res.z)
        }

        run {
            val v = Random.nextInt().toUInt()
            val buf = RustLibrary.SendBatteryUpdateInterval(v)
            val res = RustLibrary.ReadBatteryUpdateInterval(buf)
            TestCase.assertEquals(v, res)
        }

        run {
            val v = CalibrationIndex(
                Random.nextInt().toUByte(),
                Random.nextInt().toUByte(),
                Random.nextInt().toUByte()
            )
            val buf = RustLibrary.SendGetCalibrationFor(v)
            val res = RustLibrary.ReadGetCalibrationFor(buf)
            TestCase.assertEquals(v.leg, res.leg)
            TestCase.assertEquals(v.joint, res.joint)
            TestCase.assertEquals(v.kind, res.kind)
        }

        run {
            val ci = CalibrationIndex(
                Random.nextInt().toUByte(),
                Random.nextInt().toUByte(),
                Random.nextInt().toUByte()
            )
            val v = CalibrationDatum(ci, Random.nextFloat())
            val buf = RustLibrary.SendGetCalibrationResult(v)
            val res = RustLibrary.ReadGetCalibrationResult(buf)
            TestCase.assertEquals(v.index.leg, res.index.leg)
            TestCase.assertEquals(v.index.joint, res.index.joint)
            TestCase.assertEquals(v.index.kind, res.index.kind)
            TestCase.assertEquals(v.pulse, res.pulse)
        }

        run {
            val ci = CalibrationIndex(
                Random.nextInt().toUByte(),
                Random.nextInt().toUByte(),
                Random.nextInt().toUByte()
            )
            val v = CalibrationDatum(ci, Random.nextFloat())
            val buf = RustLibrary.SendSetCalibrationDatum(v)
            val res = RustLibrary.ReadSetCalibrationDatum(buf)
            TestCase.assertEquals(v.index.leg, res.index.leg)
            TestCase.assertEquals(v.index.joint, res.index.joint)
            TestCase.assertEquals(v.index.kind, res.index.kind)
            TestCase.assertEquals(v.pulse, res.pulse)
        }

        for (result in SetRes.values()) {
            val ci = CalibrationIndex(
                Random.nextInt().toUByte(),
                Random.nextInt().toUByte(),
                Random.nextInt().toUByte()
            )
            val v = SetResult(ci, result)
            val buf = RustLibrary.SendSetCalibrationResult(v)
            val res = RustLibrary.ReadSetCalibrationResult(buf)
            TestCase.assertEquals(v.index.leg, res.index.leg)
            TestCase.assertEquals(v.index.joint, res.index.joint)
            TestCase.assertEquals(v.index.kind, res.index.kind)
            TestCase.assertEquals(v.result, res.result)
        }
    }
}