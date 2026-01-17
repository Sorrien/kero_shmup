use kero::math::Vec2F;

pub struct PlayerComponent {
    pub desired_velocity: Vec2F,
}

impl PlayerComponent {
    pub fn new() -> Self {
        Self {
            desired_velocity: Vec2F::ZERO,
        }
    }
}