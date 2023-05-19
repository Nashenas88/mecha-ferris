use super::*;
use proptest::prelude::*;

proptest! {
    #[test]
    fn f32_deser(f in any::<f32>()) {
        let mut buf = [0; 4];
        let size = f.serialize(&mut buf);
        assert_eq!(4, size);
        let res = <f32 as Deserialize>::deserialize(&buf);
        assert!(res.expect("should deserialize") == f);
    }

    #[test]
    fn f32_tuples_deser(f1 in any::<f32>(), f2 in any::<f32>(), f3 in any::<f32>()) {
        let mut buf = [0; 12];
        let t = (f1, f2, f3);
        let size = t.serialize(&mut buf);
        assert_eq!(12, size);
        let res = <(f32, f32, f32) as Deserialize>::deserialize(&buf);
        let tup = res.expect("should deserialize");
        assert!(tup.0 == t.0);
        assert!(tup.1 == t.1);
        assert!(tup.2 == t.2);
    }

    #[test]
    fn u32_deser(u in any::<u32>()) {
        let mut buf = [0; 4];
        let size = u.serialize(&mut buf);
        assert_eq!(4, size);
        let res = <u32 as Deserialize>::deserialize(&buf);
        assert!(res.expect("should deserialize") == u);
    }
}

#[test_case::test_case(StateMachine::Paused)]
#[test_case::test_case(StateMachine::Homing)]
#[test_case::test_case(StateMachine::Calibrating)]
#[test_case::test_case(StateMachine::Looping)]
#[test_case::test_case(StateMachine::Exploring)]
fn sate_machine_deser(sm: StateMachine) {
    let mut buf = [0; 4];
    let size = sm.serialize(&mut buf);
    assert_eq!(1, size);
    let res = <StateMachine as Deserialize>::deserialize(&buf).expect("should deserialize");
    assert_eq!(sm, res);
}

#[test_case::test_case(
    I2cRequest::ChangeState(StateMachine::Paused),
    2,
    I2cRequestOp::ChangeState
)]
#[test_case::test_case(
    I2cRequest::SetSpeed(1.5),
    5,
    I2cRequestOp::Set(I2cRequestField::Speed)
)]
#[test_case::test_case(I2cRequest::GetSpeed, 1, I2cRequestOp::Get(I2cRequestField::Speed))]
#[test_case::test_case(
    I2cRequest::SetAngularVelocity(1.5),
    5,
    I2cRequestOp::Set(I2cRequestField::AngularVelocity)
)]
#[test_case::test_case(
    I2cRequest::GetAngularVelocity,
    1,
    I2cRequestOp::Get(I2cRequestField::AngularVelocity)
)]
#[test_case::test_case(
    I2cRequest::SetMoveVector(1.5, 2.5, 3.5),
    13,
    I2cRequestOp::Set(I2cRequestField::MoveVector)
)]
#[test_case::test_case(
    I2cRequest::GetMoveVector,
    1,
    I2cRequestOp::Get(I2cRequestField::MoveVector)
)]
#[test_case::test_case(
    I2cRequest::SetBodyTranslation(1.5, 2.5, 3.5),
    13,
    I2cRequestOp::Set(I2cRequestField::BodyTranslation)
)]
#[test_case::test_case(
    I2cRequest::GetBodyTranslation,
    1,
    I2cRequestOp::Get(I2cRequestField::BodyTranslation)
)]
#[test_case::test_case(
    I2cRequest::SetBodyRotation(1.5, 2.5, 3.5),
    13,
    I2cRequestOp::Set(I2cRequestField::BodyRotation)
)]
#[test_case::test_case(
    I2cRequest::GetBodyRotation,
    1,
    I2cRequestOp::Get(I2cRequestField::BodyRotation)
)]
#[test_case::test_case(
    I2cRequest::SetLegRadius(1.5),
    5,
    I2cRequestOp::Set(I2cRequestField::LegRadius)
)]
#[test_case::test_case(
    I2cRequest::GetLegRadius,
    1,
    I2cRequestOp::Get(I2cRequestField::LegRadius)
)]
#[test_case::test_case(
    I2cRequest::SetBatteryUpdateInterval(1),
    5,
    I2cRequestOp::SetBatteryUpdateInterval
)]
#[test_case::test_case(I2cRequest::GetBatteryLevel, 1, I2cRequestOp::GetBatteryLevel)]
fn i2c_request_deser(req: I2cRequest, expected_size: usize, expected_op: I2cRequestOp) {
    let mut buf = [0; 16];
    let size = req.serialize(&mut buf);
    assert_eq!(size, expected_size);
    let op = <I2cRequestOp as Deserialize>::deserialize(&buf)
        .unwrap_or_else(|e| panic!("should deserialize {buf:?}: {e:?}"));
    assert_eq!(op, expected_op);
    let res = <I2cRequest as Deserialize>::deserialize((op, &buf));
    match (req, &res) {
        (
            I2cRequest::ChangeState(StateMachine::Paused),
            Ok(I2cRequest::ChangeState(StateMachine::Paused)),
        ) => {}
        (I2cRequest::SetAnimationFactor(f1), Ok(I2cRequest::SetAnimationFactor(f2)))
        | (I2cRequest::SetAngularVelocity(f1), Ok(I2cRequest::SetAngularVelocity(f2)))
        | (I2cRequest::SetLegRadius(f1), Ok(I2cRequest::SetLegRadius(f2)))
            if (f1 - f2).abs() < std::f32::EPSILON => {}
        (I2cRequest::SetMotionVector(x1, y1, z1), Ok(I2cRequest::SetMotionVector(x2, y2, z2)))
        | (
            I2cRequest::SetBodyTranslation(x1, y1, z1),
            Ok(I2cRequest::SetBodyTranslation(x2, y2, z2)),
        )
        | (I2cRequest::SetBodyRotation(x1, y1, z1), Ok(I2cRequest::SetBodyRotation(x2, y2, z2)))
            if (x1 - x2).abs() < std::f32::EPSILON
                && (y1 - y2).abs() < std::f32::EPSILON
                && (z1 - z2).abs() < std::f32::EPSILON => {}
        (I2cRequest::SetBatteryUpdateInterval(_), Ok(I2cRequest::SetBatteryUpdateInterval(_))) => {}
        (I2cRequest::GetAnimationFactor, Ok(I2cRequest::GetAnimationFactor))
        | (I2cRequest::GetAngularVelocity, Ok(I2cRequest::GetAngularVelocity))
        | (I2cRequest::GetMotionVector, Ok(I2cRequest::GetMotionVector))
        | (I2cRequest::GetBodyTranslation, Ok(I2cRequest::GetBodyTranslation))
        | (I2cRequest::GetBodyRotation, Ok(I2cRequest::GetBodyRotation))
        | (I2cRequest::GetLegRadius, Ok(I2cRequest::GetLegRadius))
        | (I2cRequest::GetBatteryLevel, Ok(I2cRequest::GetBatteryLevel)) => {}
        _ => panic!("failed comparison {req:?} {res:?}"),
    }
}

#[test]
fn vector3_deser() {
    let mut buf = [0; 16];
    let size = Vector3::new(1.5, 2.5, 3.5).serialize(&mut buf);
    assert_eq!(size, 12);
    let res = <Vector3<f32> as Deserialize>::deserialize(&buf).expect("should deserialize");
    assert_eq!(res, Vector3::new(1.5, 2.5, 3.5));
}

#[test]
fn vector4_deser() {
    let mut buf = [0; 16];
    let size = Vector4::new(1.5, 2.5, 3.5, 4.5).serialize(&mut buf);
    assert_eq!(size, 16);
    let res = <Vector4<f32> as Deserialize>::deserialize(&buf).expect("should deserialize");
    assert_eq!(res, Vector4::new(1.5, 2.5, 3.5, 4.5));
}

#[test]
fn translation3_deser() {
    let mut buf = [0; 16];
    let size = Translation3::new(1.5, 2.5, 3.5).serialize(&mut buf);
    assert_eq!(size, 12);
    let res = <Translation3<f32> as Deserialize>::deserialize(&buf).expect("should deserialize");
    assert_eq!(res, Translation3::new(1.5, 2.5, 3.5));
}

#[test]
fn quaternion_deser() {
    let mut buf = [0; 16];
    let axisangle = Vector3::y() * std::f32::consts::FRAC_PI_2;
    let orig = UnitQuaternion::new(axisangle);
    println!("Orig: {orig:?} {:?}", orig.as_vector());
    let size = orig.serialize(&mut buf);
    assert_eq!(size, 16);
    let res = <UnitQuaternion<f32> as Deserialize>::deserialize(&buf).expect("should deserialize");
    let expected = UnitQuaternion::new(axisangle);
    println!("Expected: {expected:?} {:?}", expected.as_vector());
    assert_eq!(res, expected);
}

#[test]
fn tup3_deser() {
    let mut buf = [0; 16];
    let size = (1.5, 2.5, 3.5).serialize(&mut buf);
    assert_eq!(size, 12);
    let res = <(f32, f32, f32) as Deserialize>::deserialize(&buf).expect("should deserialize");
    assert_eq!(res, (1.5, 2.5, 3.5));
}

#[test]
fn tup4_deser() {
    let mut buf = [0; 16];
    let size = (1.5, 2.5, 3.5, 4.5).serialize(&mut buf);
    assert_eq!(size, 16);
    let res = <(f32, f32, f32, f32) as Deserialize>::deserialize(&buf).expect("should deserialize");
    assert_eq!(res, (1.5, 2.5, 3.5, 4.5));
}
