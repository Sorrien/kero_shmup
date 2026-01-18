use kero::math::Vec2F;

pub struct PlayerComponent {
    pub desired_velocity: Vec2F,
    pub aim_angle: f32,
}

impl PlayerComponent {
    pub fn new() -> Self {
        Self {
            desired_velocity: Vec2F::ZERO,
            aim_angle: 0.0,
        }
    }
}
