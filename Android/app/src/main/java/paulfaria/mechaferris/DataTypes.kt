package paulfaria.mechaferris

enum class Joint(val num: Int) {
    COXA(0),
    FEMUR(1),
    TIBIA(2)
}

enum class CalibrationKind(val num: Int) {
    MIN(0),
    MID(1),
    MAX(2),
    HOME(3),
}

data class CalibrationData(
    val leg: Int,
    val joint: Joint,
    val kind: CalibrationKind,
    val pulse: Float,
)

fun Vector3.toPrettyString(): String = "($x, $y, $z)"
fun Translation.toPrettyString(): String = "($x, $y, $z)"
fun Quaternion.toPrettyString(): String = "($x, $y, $z, $w)"
