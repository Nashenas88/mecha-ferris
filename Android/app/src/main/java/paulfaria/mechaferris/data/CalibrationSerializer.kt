package paulfaria.mechaferris.data

import android.content.Context
import androidx.datastore.core.CorruptionException
import androidx.datastore.core.DataStore
import androidx.datastore.core.Serializer
import androidx.datastore.dataStore
import com.google.protobuf.InvalidProtocolBufferException
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import paulfaria.mechaferris.calibration.Calibration
import java.io.InputStream
import java.io.OutputStream

class CalibrationSerializer: Serializer<Calibration> {
    override val defaultValue: Calibration = Calibration.getDefaultInstance()

    override suspend fun readFrom(input: InputStream): Calibration = withContext(Dispatchers.IO) {
        try {
            return@withContext Calibration.parseFrom(input)
        } catch (exception: InvalidProtocolBufferException) {
            throw CorruptionException("Cannot read proto.", exception)
        }
    }

    override suspend fun writeTo(t: Calibration, output: OutputStream) = withContext(Dispatchers.IO) {
        t.writeTo(output)
    }
}

val Context.calibrationDataStore: DataStore<Calibration> by dataStore(
    fileName = "calibration.pb",
    serializer = CalibrationSerializer()
)
