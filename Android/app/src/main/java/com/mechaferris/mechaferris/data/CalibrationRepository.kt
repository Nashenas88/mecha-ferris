package com.mechaferris.mechaferris.data

import androidx.datastore.core.DataStore
import com.mechaferris.calibration.Calibration
import com.mechaferris.calibration.ServoCalibration
import com.mechaferris.mechaferris.BleService.BluetoothServiceBinder
import com.mechaferris.mechaferris.utils.Mutex

class CalibrationRepository(
    private val address: String,
    // Calibration source of truth.
    private val serviceBinder: BluetoothServiceBinder,
    // Persistent storage.
    private val calibrationLocalDataSource: DataStore<Calibration>
) {
    // In-memory cache
    val calibration: Mutex<Calibration?> = Mutex(null)

    suspend fun update(block: suspend (Calibration) -> Calibration) {
        calibrationLocalDataSource.updateData { cal ->
            val calibration = block(cal)
            serviceBinder.setCalibrationPulse(address, calibration.calibrationsList[0])
            this@CalibrationRepository.calibration.withLock {
                it?.calibrationsList?.add(ServoCalibration.getDefaultInstance())
            }
            calibration
        }
    }
}
