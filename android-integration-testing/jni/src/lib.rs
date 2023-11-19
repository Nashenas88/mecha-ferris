extern crate jni;
use bluetooth_comms::wrappers::{
    Quaternion, StateMachine, Translation, Translation3, UQuaternion, UnitQuaternion, Vector,
    Vector3, SM,
};
use bluetooth_comms::{
    CalibrationDatum, CalibrationIndex, SetRes, SetResult, TestableFixedGattValue,
};
use jni::objects::{JByteArray, JClass, JObject, JObjectArray, JValueGen};
use jni::sys::{jboolean, jfloat, jint, jsize};
use jni::JNIEnv;
use std::mem::MaybeUninit;

#[rustfmt::skip]
#[export_name =
    "Java_paulfaria_mechaferris_RustLibrary_00024Companion_SendBatteryLevel_0002dWZ4Q5Ns"
]
pub extern "system" fn send_battery_level<'a>(
    env: JNIEnv<'a>,
    _class: JClass,
    value: jint,
) -> JByteArray<'a> {
    env.byte_array_from_slice((value as u32).to_gatt()).unwrap()
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_ReadBatteryLevel"]
pub extern "system" fn read_battery_level(env: JNIEnv, _class: JClass, input: JByteArray) -> jint {
    let input: Vec<u8> = env.convert_byte_array(input).unwrap();
    let result = process(Kind::BatteryLevel, &input);
    if let Ok(Data::BatteryLevel(v)) = result {
        v as i32
    } else {
        0
    }
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_SendSync"]
pub extern "system" fn send_sync<'a>(
    env: JNIEnv<'a>,
    _class: JClass,
    value: jboolean,
) -> JByteArray<'a> {
    let b = value != 0;
    env.byte_array_from_slice(b.to_gatt()).unwrap()
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_ReadSync"]
pub extern "system" fn read_sync(env: JNIEnv, _class: JClass, input: JByteArray) -> jboolean {
    let input: Vec<u8> = env.convert_byte_array(input).unwrap();
    let result = process(Kind::Sync, &input);
    if let Ok(Data::Sync(v)) = result {
        v as jboolean
    } else {
        2
    }
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_SendState"]
pub extern "system" fn send_state<'a>(
    mut env: JNIEnv<'a>,
    _class: JClass,
    state: JObject,
) -> JByteArray<'a> {
    let field = env.get_field(&state, "r", "B").unwrap().b().unwrap() as u8;
    env.byte_array_from_slice(
        SM(match field {
            1 => StateMachine::Paused,
            2 => StateMachine::Homing,
            3 => StateMachine::Calibrating,
            4 => StateMachine::Looping,
            5 => StateMachine::Exploring,
            _ => panic!("Bad state machine value: {field}"),
        })
        .to_gatt(),
    )
    .unwrap()
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_ReadState"]
pub extern "system" fn read_state<'a>(
    mut env: JNIEnv<'a>,
    _class: JClass,
    input: JByteArray,
) -> JObject<'a> {
    let input: Vec<u8> = env.convert_byte_array(input).unwrap();
    let result = process(Kind::State, &input);
    let Ok(Data::State(SM(sm))) = result else {
        return JObject::null();
    };
    let values: JObjectArray = env
        .get_static_field(
            "paulfaria/mechaferris/StateMachine",
            "$VALUES",
            "[Lpaulfaria/mechaferris/StateMachine;",
        )
        .unwrap()
        .l()
        .unwrap()
        .into();
    let elem = env.get_object_array_element(values, (sm as jsize) - 1);
    elem.unwrap_or_else(|e| {
        println!("Error: {e:?}");
        JObject::null()
    })
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_SendAnimationFactor"]
pub extern "system" fn send_animation_factor<'a>(
    env: JNIEnv<'a>,
    _class: JClass,
    value: jfloat,
) -> JByteArray<'a> {
    env.byte_array_from_slice(value.to_gatt()).unwrap()
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_ReadAnimationFactor"]
pub extern "system" fn read_animation_factor(
    env: JNIEnv,
    _class: JClass,
    input: JByteArray,
) -> jfloat {
    let input: Vec<u8> = env.convert_byte_array(input).unwrap();
    let result = process(Kind::AnimationFactor, &input);
    if let Ok(Data::AnimationFactor(v)) = result {
        v
    } else {
        -1.0
    }
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_SendAngularVelocity"]
pub extern "system" fn send_angular_velocity<'a>(
    env: JNIEnv<'a>,
    _class: JClass,
    value: jfloat,
) -> JByteArray<'a> {
    env.byte_array_from_slice(value.to_gatt()).unwrap()
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_ReadAngularVelocity"]
pub extern "system" fn read_angular_velocity(
    env: JNIEnv,
    _class: JClass,
    input: JByteArray,
) -> jfloat {
    let input: Vec<u8> = env.convert_byte_array(input).unwrap();
    let result = process(Kind::AngularVelocity, &input);
    if let Ok(Data::AngularVelocity(v)) = result {
        v
    } else {
        -1.0
    }
}

fn get_floats<const SIZE: usize>(
    env: &mut JNIEnv,
    obj: &JObject,
    fields: [&str; SIZE],
) -> [f32; SIZE] {
    // Safety: Array is always initialized.
    let mut res: [MaybeUninit<f32>; SIZE] = unsafe { MaybeUninit::uninit().assume_init() };

    for (field, val) in fields.iter().zip(res.iter_mut()) {
        val.write(env.get_field(obj, field, "F").unwrap().f().unwrap());
    }

    // Safety: All values are initialized.
    unsafe {
        (*(&MaybeUninit::<[MaybeUninit<f32>; SIZE]>::new(res) as *const _
            as *const MaybeUninit<[f32; SIZE]>))
            .assume_init_read()
    }
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_SendMotionVector"]
pub extern "system" fn send_motion_vector<'a>(
    mut env: JNIEnv<'a>,
    _class: JClass,
    value: JObject,
) -> JByteArray<'a> {
    let [x, y, z] = get_floats(&mut env, &value, ["x", "y", "z"]);
    env.byte_array_from_slice(Vector(Vector3::new(x, y, z)).to_gatt())
        .unwrap()
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_ReadMotionVector"]
pub extern "system" fn read_motion_vector<'a>(
    mut env: JNIEnv<'a>,
    _class: JClass,
    input: JByteArray,
) -> JObject<'a> {
    let input: Vec<u8> = env.convert_byte_array(input).unwrap();
    let result = process(Kind::MotionVector, &input);
    if let Ok(Data::MotionVector(v)) = result {
        env.new_object(
            "paulfaria/mechaferris/Vector3",
            "(FFF)V",
            &[
                JValueGen::Float(v.0[0]),
                JValueGen::Float(v.0[1]),
                JValueGen::Float(v.0[2]),
            ],
        )
        .unwrap()
    } else {
        JObject::null()
    }
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_SendBodyTranslation"]
pub extern "system" fn send_body_translation<'a>(
    mut env: JNIEnv<'a>,
    _class: JClass,
    value: JObject,
) -> JByteArray<'a> {
    let [x, y, z] = get_floats(&mut env, &value, ["x", "y", "z"]);
    env.byte_array_from_slice(Translation(Translation3::new(x, y, z)).to_gatt())
        .unwrap()
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_ReadBodyTranslation"]
pub extern "system" fn read_body_translation<'a>(
    mut env: JNIEnv<'a>,
    _class: JClass,
    input: JByteArray,
) -> JObject<'a> {
    let input: Vec<u8> = env.convert_byte_array(input).unwrap();
    let result = process(Kind::BodyTranslation, &input);
    if let Ok(Data::BodyTranslation(v)) = result {
        env.new_object(
            "paulfaria/mechaferris/Translation",
            "(FFF)V",
            &[
                JValueGen::Float(v.0.vector[0]),
                JValueGen::Float(v.0.vector[1]),
                JValueGen::Float(v.0.vector[2]),
            ],
        )
        .unwrap()
    } else {
        JObject::null()
    }
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_SendBodyRotation"]
pub extern "system" fn send_body_rotation<'a>(
    mut env: JNIEnv<'a>,
    _class: JClass,
    value: JObject,
) -> JByteArray<'a> {
    let [x, y, z, w] = get_floats(&mut env, &value, ["x", "y", "z", "w"]);
    env.byte_array_from_slice(
        UQuaternion(UnitQuaternion::new_unchecked(Quaternion::new(w, x, y, z))).to_gatt(),
    )
    .unwrap()
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_ReadBodyRotation"]
pub extern "system" fn read_body_rotation<'a>(
    mut env: JNIEnv<'a>,
    _class: JClass,
    input: JByteArray,
) -> JObject<'a> {
    let input: Vec<u8> = env.convert_byte_array(input).unwrap();
    let result = process(Kind::BodyRotation, &input);
    if let Ok(Data::BodyRotation(q)) = result {
        env.new_object(
            "paulfaria/mechaferris/Quaternion",
            "(FFFF)V",
            &[
                JValueGen::Float(q.0.quaternion()[0]),
                JValueGen::Float(q.0.quaternion()[1]),
                JValueGen::Float(q.0.quaternion()[2]),
                JValueGen::Float(q.0.quaternion()[3]),
            ],
        )
        .unwrap()
    } else {
        JObject::null()
    }
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_SendLegRadius"]
pub extern "system" fn send_leg_radius<'a>(
    env: JNIEnv<'a>,
    _class: JClass,
    value: jfloat,
) -> JByteArray<'a> {
    env.byte_array_from_slice(value.to_gatt()).unwrap()
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_ReadLegRadius"]
pub extern "system" fn read_leg_radius(env: JNIEnv, _class: JClass, input: JByteArray) -> jfloat {
    let input: Vec<u8> = env.convert_byte_array(input).unwrap();
    let result = process(Kind::LegRadius, &input);
    if let Ok(Data::LegRadius(v)) = result {
        v
    } else {
        -1.0
    }
}

#[rustfmt::skip]
#[export_name =
    "Java_paulfaria_mechaferris_RustLibrary_00024Companion_SendBatteryUpdateInterval_0002dWZ4Q5Ns"
]
pub extern "system" fn send_battery_update_interval<'a>(
    env: JNIEnv<'a>,
    _class: JClass,
    value: jint,
) -> JByteArray<'a> {
    env.byte_array_from_slice((value as u32).to_gatt()).unwrap()
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_ReadBatteryUpdateInterval_0002dOGnWXxg"]
pub extern "system" fn read_battery_update_interval(
    env: JNIEnv,
    _class: JClass,
    input: JByteArray,
) -> jint {
    let input: Vec<u8> = env.convert_byte_array(input).unwrap();
    let result = process(Kind::BatteryUpdateInterval, &input);
    if let Ok(Data::BatteryUpdateInterval(v)) = result {
        v as i32
    } else {
        0
    }
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_SendGetCalibrationFor"]
pub extern "system" fn send_get_calibration_for<'a>(
    mut env: JNIEnv<'a>,
    _class: JClass,
    value: JObject,
) -> JByteArray<'a> {
    let leg = env.get_field(&value, "leg", "B").unwrap().b().unwrap() as u8;
    let joint = env.get_field(&value, "joint", "B").unwrap().b().unwrap() as u8;
    let kind = env.get_field(&value, "kind", "B").unwrap().b().unwrap() as u8;
    env.byte_array_from_slice(CalibrationIndex { leg, joint, kind }.to_gatt())
        .unwrap()
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_ReadGetCalibrationFor"]
pub extern "system" fn read_get_calibration_for<'a>(
    mut env: JNIEnv<'a>,
    _class: JClass,
    input: JByteArray,
) -> JObject<'a> {
    let input: Vec<u8> = env.convert_byte_array(input).unwrap();
    let result = process(Kind::GetCalibrationFor, &input);
    if let Ok(Data::GetCalibrationFor(v)) = result {
        env.new_object(
            "paulfaria/mechaferris/CalibrationIndex",
            "(BBB)V",
            &[
                JValueGen::Byte(v.leg as i8),
                JValueGen::Byte(v.joint as i8),
                JValueGen::Byte(v.kind as i8),
            ],
        )
        .unwrap()
    } else {
        JObject::null()
    }
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_SendGetCalibrationResult"]
pub extern "system" fn send_get_calibration_result<'a>(
    mut env: JNIEnv<'a>,
    _class: JClass,
    value: JObject,
) -> JByteArray<'a> {
    let pulse = env.get_field(&value, "pulse", "F").unwrap().f().unwrap();
    let ci = env
        .get_field(&value, "index", "Lpaulfaria/mechaferris/CalibrationIndex;")
        .unwrap()
        .l()
        .unwrap();
    let leg = env.get_field(&ci, "leg", "B").unwrap().b().unwrap() as u8;
    let joint = env.get_field(&ci, "joint", "B").unwrap().b().unwrap() as u8;
    let kind = env.get_field(&ci, "kind", "B").unwrap().b().unwrap() as u8;
    env.byte_array_from_slice(
        CalibrationDatum::new(pulse, CalibrationIndex { leg, joint, kind }).to_gatt(),
    )
    .unwrap()
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_ReadGetCalibrationResult"]
pub extern "system" fn read_get_calibration_result<'a>(
    mut env: JNIEnv<'a>,
    _class: JClass,
    input: JByteArray,
) -> JObject<'a> {
    let input: Vec<u8> = env.convert_byte_array(input).unwrap();
    let result = process(Kind::GetCalibrationResult, &input);
    if let Ok(Data::GetCalibrationResult(v)) = result {
        let ci = env
            .new_object(
                "paulfaria/mechaferris/CalibrationIndex",
                "(BBB)V",
                &[
                    JValueGen::Byte(v.index().leg as i8),
                    JValueGen::Byte(v.index().joint as i8),
                    JValueGen::Byte(v.index().kind as i8),
                ],
            )
            .unwrap();
        env.new_object(
            "paulfaria/mechaferris/CalibrationDatum",
            "(Lpaulfaria/mechaferris/CalibrationIndex;F)V",
            &[JValueGen::Object(&ci), JValueGen::Float(v.value())],
        )
        .unwrap()
    } else {
        JObject::null()
    }
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_SendSetCalibrationDatum"]
pub extern "system" fn send_set_calibration_datum<'a>(
    mut env: JNIEnv<'a>,
    _class: JClass,
    value: JObject,
) -> JByteArray<'a> {
    let pulse = env.get_field(&value, "pulse", "F").unwrap().f().unwrap();
    let ci = env
        .get_field(&value, "index", "Lpaulfaria/mechaferris/CalibrationIndex;")
        .unwrap()
        .l()
        .unwrap();
    let leg = env.get_field(&ci, "leg", "B").unwrap().b().unwrap() as u8;
    let joint = env.get_field(&ci, "joint", "B").unwrap().b().unwrap() as u8;
    let kind = env.get_field(&ci, "kind", "B").unwrap().b().unwrap() as u8;
    env.byte_array_from_slice(
        CalibrationDatum::new(pulse, CalibrationIndex { leg, joint, kind }).to_gatt(),
    )
    .unwrap()
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_ReadSetCalibrationDatum"]
pub extern "system" fn read_set_calibration_datum<'a>(
    mut env: JNIEnv<'a>,
    _class: JClass,
    input: JByteArray,
) -> JObject<'a> {
    let input: Vec<u8> = env.convert_byte_array(input).unwrap();
    let result = process(Kind::SetCalibrationDatum, &input);
    if let Ok(Data::SetCalibrationDatum(v)) = result {
        let ci = env
            .new_object(
                "paulfaria/mechaferris/CalibrationIndex",
                "(BBB)V",
                &[
                    JValueGen::Byte(v.index().leg as i8),
                    JValueGen::Byte(v.index().joint as i8),
                    JValueGen::Byte(v.index().kind as i8),
                ],
            )
            .unwrap();
        env.new_object(
            "paulfaria/mechaferris/CalibrationDatum",
            "(Lpaulfaria/mechaferris/CalibrationIndex;F)V",
            &[JValueGen::Object(&ci), JValueGen::Float(v.value())],
        )
        .unwrap()
    } else {
        JObject::null()
    }
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_SendSetCalibrationResult"]
pub extern "system" fn send_set_calibration_result<'a>(
    mut env: JNIEnv<'a>,
    _class: JClass,
    value: JObject,
) -> JByteArray<'a> {
    let result = env
        .get_field(&value, "result", "Lpaulfaria/mechaferris/SetRes;")
        .unwrap()
        .l()
        .unwrap();
    let res = env.get_field(&result, "r", "B").unwrap().b().unwrap() as u8;
    let ci = env
        .get_field(&value, "index", "Lpaulfaria/mechaferris/CalibrationIndex;")
        .unwrap()
        .l()
        .unwrap();
    let leg = env.get_field(&ci, "leg", "B").unwrap().b().unwrap() as u8;
    let joint = env.get_field(&ci, "joint", "B").unwrap().b().unwrap() as u8;
    let kind = env.get_field(&ci, "kind", "B").unwrap().b().unwrap() as u8;
    env.byte_array_from_slice(
        SetResult {
            index: CalibrationIndex { leg, joint, kind },
            result: match res {
                0 => SetRes::Ok,
                1 => SetRes::InvalidIndex,
                2 => SetRes::InvalidValue,
                3 => SetRes::InvalidKind,
                4 => SetRes::InvalidLeg,
                5 => SetRes::InvalidJoint,
                _ => panic!("Unexpected res value {res}"),
            },
        }
        .to_gatt(),
    )
    .unwrap()
}

#[export_name = "Java_paulfaria_mechaferris_RustLibrary_00024Companion_ReadSetCalibrationResult"]
pub extern "system" fn read_set_calibration_result<'a>(
    mut env: JNIEnv<'a>,
    _class: JClass,
    input: JByteArray,
) -> JObject<'a> {
    let input: Vec<u8> = env.convert_byte_array(input).unwrap();
    let result = process(Kind::SetCalibrationResult, &input);
    let Ok(Data::SetCalibrationResult(v)) = result else {
        return JObject::null();
    };

    let values: JObjectArray = env
        .get_static_field(
            "paulfaria/mechaferris/SetRes",
            "$VALUES",
            "[Lpaulfaria/mechaferris/SetRes;",
        )
        .unwrap()
        .l()
        .unwrap()
        .into();
    let res_elem = env.get_object_array_element(values, v.result as jsize);
    let res_elem = res_elem.unwrap_or_else(|e| {
        println!("Error: {e:?}");
        JObject::null()
    });
    let ci = env
        .new_object(
            "paulfaria/mechaferris/CalibrationIndex",
            "(BBB)V",
            &[
                JValueGen::Byte(v.index.leg as i8),
                JValueGen::Byte(v.index.joint as i8),
                JValueGen::Byte(v.index.kind as i8),
            ],
        )
        .unwrap();
    env.new_object(
        "paulfaria/mechaferris/SetResult",
        "(Lpaulfaria/mechaferris/CalibrationIndex;Lpaulfaria/mechaferris/SetRes;)V",
        &[JValueGen::Object(&ci), JValueGen::Object(&res_elem)],
    )
    .unwrap()
}

#[derive(Debug)]
enum Error {}

fn process(kind: Kind, data: &[u8]) -> Result<Data, Error> {
    let data = match kind {
        Kind::BatteryLevel => Data::BatteryLevel(u8::from_gatt(data)),
        Kind::Sync => Data::Sync(bool::from_gatt(data)),
        Kind::State => Data::State(SM::from_gatt(data)),
        Kind::AnimationFactor => Data::AnimationFactor(f32::from_gatt(data)),
        Kind::AngularVelocity => Data::AngularVelocity(f32::from_gatt(data)),
        Kind::MotionVector => Data::MotionVector(Vector::from_gatt(data)),
        Kind::BodyTranslation => Data::BodyTranslation(Translation::from_gatt(data)),
        Kind::BodyRotation => Data::BodyRotation(UQuaternion::from_gatt(data)),
        Kind::LegRadius => Data::LegRadius(f32::from_gatt(data)),
        Kind::BatteryUpdateInterval => Data::BatteryUpdateInterval(u32::from_gatt(data)),
        Kind::GetCalibrationFor => Data::GetCalibrationFor(CalibrationIndex::from_gatt(data)),
        Kind::GetCalibrationResult => Data::GetCalibrationResult(CalibrationDatum::from_gatt(data)),
        Kind::SetCalibrationDatum => Data::SetCalibrationDatum(CalibrationDatum::from_gatt(data)),
        Kind::SetCalibrationResult => Data::SetCalibrationResult(SetResult::from_gatt(data)),
    };
    Ok(data)
}

enum Kind {
    BatteryLevel,
    Sync,
    State,
    AnimationFactor,
    AngularVelocity,
    MotionVector,
    BodyTranslation,
    BodyRotation,
    LegRadius,
    BatteryUpdateInterval,
    GetCalibrationFor,
    GetCalibrationResult,
    SetCalibrationDatum,
    SetCalibrationResult,
}

#[derive(Debug)]
enum Data {
    // read, notify
    BatteryLevel(u8),
    // write
    Sync(bool),
    // read, write, notify, indicate
    State(SM),
    // read, write, notify, indicate
    AnimationFactor(f32),
    // read, write, notify, indicate
    AngularVelocity(f32),
    // read, write, notify, indicate
    MotionVector(Vector),
    // read, write, notify, indicate
    BodyTranslation(Translation),
    // read, write, notify, indicate
    BodyRotation(UQuaternion),
    // read, write, notify, indicate
    LegRadius(f32),
    // read, write, notify, indicate
    BatteryUpdateInterval(u32),
    // write
    GetCalibrationFor(CalibrationIndex),
    // notify
    GetCalibrationResult(CalibrationDatum),
    // write
    SetCalibrationDatum(CalibrationDatum),
    // notify
    SetCalibrationResult(SetResult),
}
