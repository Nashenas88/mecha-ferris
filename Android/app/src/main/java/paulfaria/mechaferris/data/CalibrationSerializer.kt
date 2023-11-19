package paulfaria.mechaferris.data

import android.content.Context
import android.util.Log
import androidx.datastore.core.CorruptionException
import androidx.datastore.core.DataStore
import androidx.datastore.core.Serializer
import androidx.datastore.dataStore
import com.google.protobuf.InvalidProtocolBufferException
import paulfaria.mechaferris.calibration.Calibration
import paulfaria.mechaferris.calibration.ServoCalibration
import java.io.InputStream
import java.io.OutputStream

class CalibrationSerializer : Serializer<Calibration> {
    override val defaultValue: Calibration = defaultCalibration()

    companion object {
        private fun defaultCalibration(): Calibration {
            Log.i("CalibrationSerializer", "defaultCalibration being generated")
            val builder = Calibration.newBuilder()
            for (leg in 0 until 6) {
                for (joint in 0 until 3) {
                    for (kind in 0 until if (joint == 2) {
                        4
                    } else {
                        3
                    }) {
                        builder.addCalibrations(
                            ServoCalibration.newBuilder().setLeg(leg).setJointValue(joint)
                                .setKindValue(kind)
                                .setPulse(1500.0f)
                        )
                        Log.i("CalibrationSerializer", "defaultCalibration generated: $leg $joint $kind: 1500.0")
                    }
                }
            }
            val cals = builder.build()
            Log.i("CalibrationSerializer", "defaultCalibration generated: ${cals.calibrationsCount}")
            return cals
        }
    }

    override suspend fun readFrom(input: InputStream): Calibration {
        try {
            Log.i("CalibrationSerializer", "readFrom:")
            val c = Calibration.parseFrom(input)
            Log.i("CalibrationSerializer", "readFrom: ${c.calibrationsCount}")
            return c
        } catch (exception: InvalidProtocolBufferException) {
            throw CorruptionException("Cannot read proto.", exception)
        }
    }

    override suspend fun writeTo(t: Calibration, output: OutputStream) {
        Log.i("CalibrationSerializer", "writeTo: ${t.calibrationsCount}")
        t.writeTo(output)
    }
}

val Context.calibrationDataStore: DataStore<Calibration> by dataStore(
    fileName = "calibration.pb",
    serializer = CalibrationSerializer()
)
