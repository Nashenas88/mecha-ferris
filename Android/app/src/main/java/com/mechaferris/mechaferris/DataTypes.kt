package com.mechaferris.mechaferris

data class Vector3(
    val x: Float = 0f,
    val y: Float = 0f,
    val z: Float = 0f
)

data class Quaternion(
    val x: Float = 0f,
    val y: Float = 0f,
    val z: Float = 0f,
    val w: Float = 0f
)

enum class StateMachine(val num: Int) {
    PAUSED(1),
    HOMING(2),
    CALIBRATING(3),
    LOOPING(4),
    EXPLORING(5);

    companion object {
        fun fromInt(num: Int) = StateMachine.values().first { it.num == num }
    }
}

fun Vector3.toPrettyString(): String = "($x, $y, $z)"
fun Quaternion.toPrettyString(): String = "($x, $y, $z, $w)"