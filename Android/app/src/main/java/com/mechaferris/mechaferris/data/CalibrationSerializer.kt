package com.mechaferris.mechaferris.data

import android.content.Context
import androidx.datastore.core.CorruptionException
import androidx.datastore.core.DataStore
import androidx.datastore.core.Serializer
import androidx.datastore.dataStore
import com.google.protobuf.InvalidProtocolBufferException
import java.io.InputStream
import java.io.OutputStream
import com.mechaferris.calibration.Calibration
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext

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
