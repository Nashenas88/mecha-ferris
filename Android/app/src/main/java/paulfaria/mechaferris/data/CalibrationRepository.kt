package paulfaria.mechaferris.data

import androidx.datastore.core.DataStore
import paulfaria.mechaferris.calibration.Calibration
import paulfaria.mechaferris.calibration.ServoCalibration
import paulfaria.mechaferris.utils.Mutex

class CalibrationRepository(
    // Persistent storage.
    private val calibrationLocalDataSource: DataStore<Calibration>
) {
    // In-memory cache
    val calibration: Mutex<Calibration?> = Mutex(null)

    suspend fun update(block: suspend (Calibration) -> Calibration) {
        calibrationLocalDataSource.updateData { cal ->
            val calibration = block(cal)
            this@CalibrationRepository.calibration.withLock {
                it?.calibrationsList?.add(ServoCalibration.getDefaultInstance())
            }
            calibration
        }
    }
}
