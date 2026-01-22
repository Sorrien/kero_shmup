use kero::math::Vec2F;
use rapier2d::{control::KinematicCharacterController, prelude::*};

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

pub fn step(physics_data: &mut PhysicsData) {
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
        &(),
    );
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
    pub desired_movement: Vec2F,
    pub is_grounded: bool,
    pub is_sliding_down_slope: bool,
    pub prev_position: Vec2F,
}

impl CharacterControllerComponent {
    pub fn new(character_body: RigidBodyHandle, character_collider: ColliderHandle) -> Self {
        let mut character_controller = KinematicCharacterController::default();
        character_controller.up = Vec2::NEG_Y;
        //character_controller.autostep = Some(rapier2d::control::CharacterAutostep { max_height: rapier2d::control::CharacterLength::Absolute(1.0), min_width: rapier2d::control::CharacterLength::Absolute(0.2), include_dynamic_bodies: false });
        Self {
            character_controller,
            character_body,
            character_collider,
            desired_movement: Vec2F::ZERO,
            is_grounded: false,
            is_sliding_down_slope: false,
            prev_position: Vec2F::ZERO,
        }
    }

    pub fn update_character_controller(&mut self, _last_step: f32, physics_data: &mut PhysicsData) {
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

        /*         for collision in &collisions {
                   let collider = &physics_data.collider_set[collision.handle];
                   if collider.parent().is_none() {
                       let normal = collision.hit.normal1;

                       let normal_dot_velocity = normal.dot(character_body.linvel().normalize());
                       let velocity_magnitude = character_body.linvel().length() * last_step;
                       let length_along_normal = velocity_magnitude * f32::max(normal_dot_velocity, 0.0);
                       if normal_dot_velocity >= -DEFAULT_EPSILON {
                           collision.hit.witness2
                       }
                   }
               }
        */
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
}

/*
pub fn test() {
    if body1.is_dynamic() && !body2.is_dynamic() {
            let normal = *context.normal;
            if rigid_body_1_linvel.norm() == 0.0 {
                return;
            }
            let normal_dot_velocity = normal.dot(&rigid_body_1_linvel.normalize());
            let velocity_magnitude = rigid_body_1_linvel.magnitude() * self.last_step;
            let length_along_normal = velocity_magnitude * Real::max(normal_dot_velocity, 0.0);
            if normal_dot_velocity >= -DEFAULT_EPSILON {
                context.solver_contacts.retain(|contact| {
                    let dist = -contact.dist;
                    let diff = dist - length_along_normal;
                    if diff < 0.5 && dist.abs() < self.ghost_collision_distance {
                        return false;
                    }
                    true
                });
            }
        } else if body2.is_dynamic() && !body1.is_dynamic() {
            let normal = -*context.normal;
            if rigid_body_2_linvel.norm() == 0.0 {
                return;
            }
            let normal_dot_velocity = normal.dot(&rigid_body_2_linvel.normalize());
            let velocity_magnitude = rigid_body_2_linvel.magnitude() * self.last_step;
            let length_along_normal = velocity_magnitude * Real::max(normal_dot_velocity, 0.0);
            if normal_dot_velocity >= -DEFAULT_EPSILON {
                context.solver_contacts.retain(|contact| {
                    let dist = -contact.dist;
                    let diff = dist - length_along_normal;
                    if diff < 0.5 && dist.abs() < self.ghost_collision_distance {
                        return false;
                    }
                    true
                });
            }
        }
} */
