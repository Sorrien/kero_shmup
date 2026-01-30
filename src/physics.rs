use kero::math::Vec2F;
use rapier2d::{
    control::{KinematicCharacterController, PidController},
    prelude::*,
};

pub struct PhysicsData {
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub integration_parameters: IntegrationParameters,
    pub physics_pipeline: PhysicsPipeline,
    pub island_manager: IslandManager,
    pub broad_phase: DefaultBroadPhase,
    pub narrow_phase: NarrowPhase,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub gravity: Vector2,
}

impl PhysicsData {
    pub fn new() -> Self {
        let rigid_body_set = RigidBodySet::new();
        let collider_set = ColliderSet::new();
        let integration_parameters = IntegrationParameters::default();
        let physics_pipeline = PhysicsPipeline::new();
        let island_manager = IslandManager::new();
        let broad_phase = DefaultBroadPhase::new();
        let narrow_phase = NarrowPhase::new();
        let impulse_joint_set = ImpulseJointSet::new();
        let multibody_joint_set = MultibodyJointSet::new();
        let ccd_solver = CCDSolver::new();

        Self {
            rigid_body_set,
            collider_set,
            integration_parameters,
            physics_pipeline,
            island_manager,
            broad_phase,
            narrow_phase,
            impulse_joint_set,
            multibody_joint_set,
            ccd_solver,
            gravity: Vector2::new(0.0, 9.81),
        }
    }
}

pub fn step(physics_data: &mut PhysicsData, event_handler: &ChannelEventCollector) {
    physics_data.physics_pipeline.step(
        physics_data.gravity,
        &physics_data.integration_parameters,
        &mut physics_data.island_manager,
        &mut physics_data.broad_phase,
        &mut physics_data.narrow_phase,
        &mut physics_data.rigid_body_set,
        &mut physics_data.collider_set,
        &mut physics_data.impulse_joint_set,
        &mut physics_data.multibody_joint_set,
        &mut physics_data.ccd_solver,
        &(),
        event_handler,
    );
}

const PHYSICS_SCALE: f32 = 50.0;

pub fn pixel_scale_to_physics_f(val: f32) -> f32 {
    val / PHYSICS_SCALE
}

pub fn physics_scale_to_pixels_f(val: f32) -> f32 {
    val * PHYSICS_SCALE
}

pub fn pixel_scale_to_physics(vec: Vec2F) -> Vec2F {
    vec / PHYSICS_SCALE
}

pub fn physics_scale_to_pixels(vec: Vec2F) -> Vec2F {
    vec * PHYSICS_SCALE
}

pub fn pixel_scale_to_physics_glam(vec: rapier2d::math::Vec2) -> rapier2d::math::Vec2 {
    vec / PHYSICS_SCALE
}

pub fn physics_scale_to_pixels_glam(vec: rapier2d::math::Vec2) -> rapier2d::math::Vec2 {
    vec * PHYSICS_SCALE
}

pub struct RigidBodyComponent {
    pub rigid_body_handle: RigidBodyHandle,
}

pub struct ColliderComponent {
    pub collider_handle: ColliderHandle,
}

pub struct CharacterControllerComponent {
    pub character_controller: KinematicCharacterController,
    pub character_body: RigidBodyHandle,
    pub character_collider: ColliderHandle,
    pub desired_movement: Vec2,
    pub is_grounded: bool,
    pub is_sliding_down_slope: bool,
    pub prev_position: Vec2F,
    pub is_kinematic: bool,
    pub pid_controller: Option<PidController>,
    pub ground_sensor_collider: ColliderHandle,
    pub count_ground_sensor_contacts: u32,
}

impl CharacterControllerComponent {
    pub fn new(
        character_body: RigidBodyHandle,
        character_collider: ColliderHandle,
        is_kinematic: bool,
        ground_sensor_collider: ColliderHandle,
    ) -> Self {
        let mut character_controller = KinematicCharacterController::default();
        character_controller.up = Vec2::NEG_Y;
        let pid_controller = if is_kinematic {
            None
        } else {
            Some(PidController::default())
        };

        Self {
            character_controller,
            character_body,
            character_collider,
            desired_movement: Vec2::ZERO,
            is_grounded: false,
            is_sliding_down_slope: false,
            prev_position: Vec2F::ZERO,
            is_kinematic,
            pid_controller,
            ground_sensor_collider,
            count_ground_sensor_contacts: 0,
        }
    }
    pub fn update_kinematic_character_controller(&mut self, physics_data: &mut PhysicsData) {
        let character_body = physics_data
            .rigid_body_set
            .get(self.character_body)
            .unwrap();
        let (character_shape, character_pos) = {
            let character_collider = physics_data
                .collider_set
                .get(self.character_collider)
                .unwrap();

            let shape = character_collider.shared_shape();
            (shape.clone(), character_collider.position().clone())
        };

        let character_mass = character_body.mass();
        let mut collisions = vec![];

        let desired_movement = Vec2::new(self.desired_movement.x, self.desired_movement.y);

        let filter = QueryFilter::new()
            .exclude_rigid_body(self.character_body)
            .exclude_sensors();

        let query_pipeline = physics_data.broad_phase.as_query_pipeline(
            physics_data.narrow_phase.query_dispatcher(),
            &physics_data.rigid_body_set,
            &physics_data.collider_set,
            filter,
        );

        let mvt = self.character_controller.move_shape(
            physics_data.integration_parameters.dt,
            &query_pipeline,
            &*character_shape,
            &character_pos,
            desired_movement + physics_data.gravity,
            |c| collisions.push(c),
        );

        self.is_grounded = mvt.grounded;
        self.is_sliding_down_slope = mvt.is_sliding_down_slope;

        let mut query_pipeline_mut = physics_data.broad_phase.as_query_pipeline_mut(
            physics_data.narrow_phase.query_dispatcher(),
            &mut physics_data.rigid_body_set,
            &mut physics_data.collider_set,
            filter,
        );

        self.character_controller
            .solve_character_collision_impulses(
                physics_data.integration_parameters.dt,
                &mut query_pipeline_mut,
                &*character_shape,
                character_mass,
                &*collisions,
            );

        let character_body = &mut physics_data.rigid_body_set[self.character_body];
        let pose = character_body.position();

        self.prev_position = Vec2F::new(pose.translation.x, pose.translation.y);
        character_body.set_next_kinematic_translation(pose.translation + mvt.translation);
    }

    pub fn update_character_controller(&mut self, physics_data: &mut PhysicsData) {
        if self.is_kinematic {
            self.update_kinematic_character_controller(physics_data);
        } else {
            self.update_pid_controller(physics_data);
        }
    }

    fn update_pid_controller(&mut self, physics_data: &mut PhysicsData) {
        if let Some(pid) = &mut self.pid_controller {
            let character_body = &mut physics_data.rigid_body_set[self.character_body];

            // Adjust the controlled axis depending on the keys pressed by the user.
            // - If the user is jumping, enable control over Y.
            // - If the user isn't pressing any key, disable all linear controls to let
            //   gravity/collision do their thing freely.
            let mut axes = AxesMask::ANG_Z;

            if self.desired_movement.length() != 0.0 {
                axes |= if self.desired_movement.y == 0.0 {
                    AxesMask::LIN_X
                } else {
                    AxesMask::LIN_X | AxesMask::LIN_Y
                }
            };

            pid.set_axes(axes);

            let corrective_vel = pid.rigid_body_correction(
                physics_data.integration_parameters.dt,
                character_body,
                Pose::from_translation(character_body.translation() + self.desired_movement),
                RigidBodyVelocity::zero(),
            );
            let new_vel = *character_body.vels() + corrective_vel;

            character_body.set_vels(new_vel, true);
        }
        self.is_grounded = self.count_ground_sensor_contacts > 0;
    }
}
