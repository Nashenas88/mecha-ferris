package com.mechaferris.mechaferris.ui.home

import android.bluetooth.BluetoothDevice
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel

class HomeViewModel : ViewModel() {

    val device: MutableLiveData<BluetoothDevice?> = MutableLiveData<BluetoothDevice?>(null)
    val state: MutableLiveData<Int> = MutableLiveData<Int>(0)
}