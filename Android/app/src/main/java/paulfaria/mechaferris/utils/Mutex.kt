package paulfaria.mechaferris.utils

import kotlinx.coroutines.sync.Mutex as KMutex

class Mutex<T>(private var value: T) {
    private val mutex: KMutex = KMutex(locked = false)

    suspend fun lock(): MutexGuard<T> {
        mutex.lock()
        return MutexGuard(mutex, value)
    }

    suspend fun <R>withLock(accessor: suspend (T) -> R): R {
        mutex.lock()
        try {
            return accessor(value)
        } finally {
            mutex.unlock()
        }
    }
}

class MutexGuard<T>(private val mutex: KMutex, var value: T) {
    fun finalize() {
        mutex.unlock()
    }
}
