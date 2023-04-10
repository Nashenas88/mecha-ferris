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
    let res = <StateMachine as Deserialize>::deserialize(&buf);
    match (sm, &res) {
        (StateMachine::Paused, Ok(StateMachine::Paused))
        | (StateMachine::Homing, Ok(StateMachine::Homing))
        | (StateMachine::Calibrating, Ok(StateMachine::Calibrating))
        | (StateMachine::Looping, Ok(StateMachine::Looping))
        | (StateMachine::Exploring, Ok(StateMachine::Exploring)) => {}
        _ => panic!("failed comparison {sm:?} {res:?}"),
    }
}
