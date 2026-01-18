use kero::math::Vec2F;

pub struct PlayerComponent {
    pub desired_velocity: Vec2F,
    pub aim_angle: f32,
    pub wants_to_shoot: bool,
    pub time_since_last_shot: f32,
    pub shot_delay_time: f32,
    pub is_shooting: bool,
    pub time_since_shot_start: f32,
    pub time_per_shot: f32,
}

impl PlayerComponent {
    pub fn new() -> Self {
        Self {
            desired_velocity: Vec2F::ZERO,
            aim_angle: 0.0,
            wants_to_shoot: false,
            time_since_last_shot: 0.0,
            shot_delay_time: 0.2,
            time_since_shot_start: 0.0,
            time_per_shot: 0.1,
            is_shooting: false,
        }
    }
}
