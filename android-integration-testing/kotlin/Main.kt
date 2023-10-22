import paulfaria.mechaferris.*
import kotlin.random.Random

@kotlin.ExperimentalUnsignedTypes
fun main(args: Array<String>) {
    println("Kotlin <3 Rust")

    for (v in arrayOf(true, false)) {
        val buf = RustLibrary.SendSync(v)
        println("SendSync ${v}: ${buf.contentToString()}")
        val res = RustLibrary.ReadSync(buf)
        println("ReadSync: ${res}\n")
    }

    for (v in StateMachine.values()) {
        val buf = RustLibrary.SendState(v)
        println("SendState ${v}: ${buf.contentToString()}")
        val res = RustLibrary.ReadState(buf)
        println("ReadState: ${res}\n")
    }

    run {
        val v = Random.nextFloat()
        val buf = RustLibrary.SendAnimationFactor(v)
        println("SendAnimationFactor ${v}: ${buf.contentToString()}")
        val res = RustLibrary.ReadAnimationFactor(buf)
        println("ReadAnimationFactor: ${res}\n")
    }

    run {
        val v = Vector3(Random.nextFloat(), Random.nextFloat(), Random.nextFloat())
        val buf = RustLibrary.SendMotionVector(v)
        println("SendMotionVector (${v.x}, ${v.y}, ${v.z}): ${buf.contentToString()}")
        val res = RustLibrary.ReadMotionVector(buf)
        println("ReadMotionVector: (${res.x}, ${res.y}, ${res.z})\n")
    }

    run {
        val v = Translation(Random.nextFloat(), Random.nextFloat(), Random.nextFloat())
        val buf = RustLibrary.SendBodyTranslation(v)
        println("SendBodyTranslation (${v.x}, ${v.y}, ${v.z}): ${buf.contentToString()}")
        val res = RustLibrary.ReadBodyTranslation(buf)
        println("ReadBodyTranslation: (${res.x}, ${res.y}, ${res.z})\n")
    }

    run {
        val v = Quaternion(Random.nextFloat(), Random.nextFloat(), Random.nextFloat(), Random.nextFloat())
        val buf = RustLibrary.SendBodyRotation(v)
        println("SendBodyRotation (${v.x}, ${v.y}, ${v.z}, ${v.w}): ${buf.contentToString()}")
        val res = RustLibrary.ReadBodyRotation(buf)
        println("ReadBodyRotation: (${res.x}, ${res.y}, ${res.z}, ${res.w})\n")
    }

    run {
        val v = Random.nextInt().toUInt()
        val buf = RustLibrary.SendBatteryUpdateInterval(v)
        println("SendBatteryUpdateInterval ${v}: ${buf.contentToString()}")
        val res = RustLibrary.ReadBatteryUpdateInterval(buf)
        println("ReadBatteryUpdateInterval: ${res}\n")
    }

    run {
        val v = CalibrationIndex(Random.nextInt().toUByte(), Random.nextInt().toUByte(), Random.nextInt().toUByte())
        val buf = RustLibrary.SendGetCalibrationFor(v)
        println("SendGetCalibrationFor (${v.leg}, ${v.joint}, ${v.kind}): ${buf.contentToString()}")
        val res = RustLibrary.ReadGetCalibrationFor(buf)
        println("ReadGetCalibrationFor: (${res.leg}, ${res.joint}, ${res.kind})\n")
    }

    run {
        val ci = CalibrationIndex(Random.nextInt().toUByte(), Random.nextInt().toUByte(), Random.nextInt().toUByte())
        val v = CalibrationDatum(ci, Random.nextFloat())
        val buf = RustLibrary.SendGetCalibrationResult(v)
        println("SendGetCalibrationResult (${v.index.leg}, ${v.index.joint}, ${v.index.kind}, ${v.pulse}): ${buf.contentToString()}")
        val res = RustLibrary.ReadGetCalibrationResult(buf)
        println("ReadGetCalibrationResult: (${res.index.leg}, ${res.index.joint}, ${res.index.kind}, ${res.pulse})\n")
    }

    run {
        val ci = CalibrationIndex(Random.nextInt().toUByte(), Random.nextInt().toUByte(), Random.nextInt().toUByte())
        val v = CalibrationDatum(ci, Random.nextFloat())
        val buf = RustLibrary.SendSetCalibrationDatum(v)
        println("SendSetCalibrationDatum (${v.index.leg}, ${v.index.joint}, ${v.index.kind}, ${v.pulse}): ${buf.contentToString()}")
        val res = RustLibrary.ReadSetCalibrationDatum(buf)
        println("ReadSetCalibrationDatum: (${res.index.leg}, ${res.index.joint}, ${res.index.kind}, ${res.pulse})\n")
    }

    for (res in SetRes.values()) {
        val ci = CalibrationIndex(Random.nextInt().toUByte(), Random.nextInt().toUByte(), Random.nextInt().toUByte())
        val v = SetResult(ci, res)
        val buf = RustLibrary.SendSetCalibrationResult(v)
        println("SendSetCalibrationResult (${v.index.leg}, ${v.index.joint}, ${v.index.kind}, ${v.result}): ${buf.contentToString()}")
        val res = RustLibrary.ReadSetCalibrationResult(buf)
        println("ReadSetCalibrationResult: (${res.index.leg}, ${res.index.joint}, ${res.index.kind}, ${res.result})\n")
    }

    println("Done")
}