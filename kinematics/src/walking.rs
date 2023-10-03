use nalgebra::{Matrix4, Point3, Rotation3, Scalar, Translation3, Vector3 as Vec3};
use state::RobotState;

use crate::animation::{Animation, AnimationManager};
use crate::leg::LegError;
use crate::{ExpensiveMath, Leg, LegConsts};

pub struct Walking<T, C> {
    animation_manager: AnimationManager,
    legs: [MechaLeg<f32, T, C>; 6],
    body_radius: f32,
}

impl<T, C> Walking<T, C> {
    pub fn new(body_radius: f32) -> Self {
        let mut legs = [MechaLeg::default(); 6];
        for (i, leg) in legs.iter_mut().enumerate() {
            leg.idx = i as u8;
        }
        Self {
            animation_manager: AnimationManager::new(),
            // walking_radius: initial_walking_radius,
            // body_rotation: UnitQuaternion::default(),
            legs,
            // body_height: initial_body_height,
            body_radius,
        }
    }

    pub fn state(&self) -> Animation {
        self.animation_manager.animation()
    }

    pub fn duration(&self) -> f64 {
        self.animation_manager.duration()
    }

    pub fn next_animation(&mut self) -> (bool, f64) {
        self.animation_manager.next_animation()
    }

    pub fn set_body_radius(&mut self, radius: f32) {
        self.body_radius = radius;
    }
}

impl<T, C> Walking<T, C>
where
    T: ExpensiveMath<f32>,
    C: LegConsts<f32>,
{
    pub fn home<V, J, const SPL: usize, const S: usize>(
        &mut self,
        robot_state: &RobotState<J, SPL, S>,
        visitor: &mut V,
        time: f32,
    ) where
        V: VisitLeg<f32, T, C>,
    {
        let targets = self
            .animation_manager
            .home::<T>(robot_state.leg_radius, time);
        for (leg, target) in self.legs.iter_mut().zip(targets.into_iter()) {
            visitor.before(target, leg);

            // Get target in each leg's coordinate space.
            let target = leg
                .origin_to_coxa(robot_state.body_translation.y, self.body_radius)
                .transform_point(&target);

            if let Err(e) = leg.leg.go_to(target) {
                visitor.on_error(leg, e);
            }
            visitor.after(target, leg);
        }
    }
}

pub struct MechaLeg<F, T, C> {
    leg: Leg<F, T, C>,
    idx: u8,
}

impl<F, T, C> Clone for MechaLeg<F, T, C>
where
    F: Clone,
{
    fn clone(&self) -> Self {
        Self {
            leg: self.leg.clone(),
            idx: self.idx,
        }
    }
}

impl<F, T, C> Copy for MechaLeg<F, T, C> where F: Copy {}

impl<F, T, C> Default for MechaLeg<F, T, C>
where
    F: Default,
{
    fn default() -> Self {
        Self {
            leg: Default::default(),
            idx: Default::default(),
        }
    }
}

impl<F, T, C> MechaLeg<F, T, C> {
    pub fn leg(&self) -> &Leg<F, T, C> {
        &self.leg
    }

    pub fn idx(&self) -> u8 {
        self.idx
    }
}

impl<F, T, C> MechaLeg<F, T, C> {
    /// Returns matrix that can transform a point in the body space to the coxa space for a leg.
    fn origin_to_coxa(&self, height: f32, radius: f32) -> Matrix4<f32> {
        let rot = self.idx as f32 * core::f32::consts::FRAC_PI_3;
        Rotation3::from_axis_angle(&Vec3::y_axis(), core::f32::consts::FRAC_PI_2).to_homogeneous()
            * Translation3::new(0.0, -height, -radius).to_homogeneous()
            * Rotation3::from_axis_angle(&Vec3::y_axis(), -rot).to_homogeneous()
    }
}

impl<T, C> Walking<T, C>
where
    T: ExpensiveMath<f32>,
    C: LegConsts<f32>,
{
    pub fn walk<V, J, const SPL: usize, const S: usize>(
        &mut self,
        robot_state: &RobotState<J, SPL, S>,
        leg_visitor: &mut V,
        time: f32,
    ) where
        V: VisitLeg<f32, T, C>,
    {
        // leg_visitor.position_start();
        // Get the targets for each foot.
        let targets = self
            .animation_manager
            .targets::<T>(robot_state.leg_radius, time);
        // leg_visitor.position_end();

        // if false {
        //     // self.body_rotation =
        //     //     UnitQuaternion::from_axis_angle(&Vec3::y_axis(), time * core::f32::consts::TAU)
        //     //         * UnitQuaternion::from_axis_angle(
        //     //             &Vec3::x_axis(),
        //     //             core::f32::consts::FRAC_PI_8,
        //     //         );
        //     // let point = //(Translation3::new(0.0, self.body_height as f32, 0.0).to_homogeneous()
        //     //     //*
        //     //     self.body_rotation.to_homogeneous()
        //     //     //* Translation3::new(0.0, -self.body_height as f32, 0.0).to_homogeneous())
        //     // .transform_point(&Point3::new(10.0, 0.0, 0.0));

        //     // for target in &mut targets {
        //     //     *target = self.body_rotation.transform_point(target);
        //     // }
        //     // for target in &mut targets {
        //     //     *target = Translation3::new(0.0, self.height_offset, 0.0).transform_point(target);
        //     // }
        // }

        // leg_visitor.servo_start();
        // Calculate the servo angles for each leg.
        for (target, leg) in targets.into_iter().zip(self.legs.iter_mut()) {
            leg_visitor.before(target, leg);

            // Get target in each leg's coordinate space.
            let target = leg
                .origin_to_coxa(robot_state.body_translation.y, self.body_radius)
                .transform_point(&target);

            if let Err(e) = leg.leg.go_to(target) {
                leg_visitor.on_error(leg, e);
            }

            leg_visitor.after(target, leg);
        }
        // leg_visitor.servo_end();
    }
}

pub trait VisitLeg<F, T, C>
where
    F: Scalar,
{
    fn before(&mut self, target: Point3<F>, leg: &MechaLeg<F, T, C>);
    fn on_error(&mut self, leg: &MechaLeg<F, T, C>, error: LegError<F>);
    fn after(&mut self, target: Point3<F>, leg: &MechaLeg<F, T, C>);
    fn position_start(&mut self);
    fn position_end(&mut self);
    fn servo_start(&mut self);
    fn servo_end(&mut self);
}

impl<F, T, C> VisitLeg<F, T, C> for ()
where
    F: Scalar,
{
    fn before(&mut self, _: Point3<F>, _: &MechaLeg<F, T, C>) {}
    fn on_error(&mut self, _: &MechaLeg<F, T, C>, _: LegError<F>) {}
    fn after(&mut self, _: Point3<F>, _: &MechaLeg<F, T, C>) {}
    fn position_start(&mut self) {}
    fn position_end(&mut self) {}
    fn servo_start(&mut self) {}
    fn servo_end(&mut self) {}
}
