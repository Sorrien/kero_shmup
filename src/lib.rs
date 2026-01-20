use std::{any::Any, collections::HashMap, fs::File, io::Read};

use crate::{
    ldtk::{TileGrid, TileType, Tileset},
    ldtk_json::LdtkJson,
    physics::{CharacterControllerComponent, PhysicsData, step},
    player::PlayerComponent,
};
use hecs::{Entity, World};
use kero::{grid::Grid, prelude::*};
use rapier2d::prelude::{
    ColliderBuilder, ColliderHandle, Group, InteractionGroups, QueryFilter, Ray, RigidBodyBuilder,
    RigidBodyHandle,
};

pub mod ldtk;
pub mod ldtk_json;
pub mod physics;
pub mod player;

const PLAYER_HEIGHT: f32 = 17.0;
const PLAYER_WIDTH: f32 = 14.0;

const TEST_ENEMY_HEIGHT: f32 = 20.;
const TEST_ENEMY_WIDTH: f32 = 24.;

const TERRAIN_GROUP: Group = Group::GROUP_1;
const PLAYER_GROUP: Group = Group::GROUP_2;
const ENEMY_GROUP: Group = Group::GROUP_3;
const PLAYER_PROJECTILE_GROUP: Group = Group::GROUP_4;
const ENEMY_PROJECTILE_GROUP: Group = Group::GROUP_5;

pub fn to_logical_f(pos: Vec2F, scale: f32) -> Vec2F {
    pos / scale
}

pub struct RaycastProjectile {
    pub position: Vec2F,
    pub velocity: Vec2F,
    pub solid: bool,
    pub life_time: f32,
    pub max_life_time: f32,
    pub penetrate: bool,
}

pub struct HealthComponent {
    pub health: f32,
}

pub struct PlayerOwned {
    pub owner: Entity,
}
pub struct EnemyOwned {
    pub owner: Entity,
}

pub struct ProjectileImpact {
    pub hit_entity_id: u32,
    pub _proj_entity: Entity,
}

pub struct EnemyComponent {}

pub struct Camera {
    pub position: Vec2F,
    pub width: f32,
    pub height: f32,
    pub zoom_amount: f32,
}

impl Camera {
    pub fn new(position: Vec2F, width: f32, height: f32, zoom_amount: f32) -> Self {
        Self {
            position,
            width,
            height,
            zoom_amount,
        }
    }
}

pub struct PlayerAnimComponent {
    run_index: usize,
    is_crouching: bool,
    is_jumping: bool,
    flip_x: bool,
    run_frame_time: f32,
    run_frame_time_acc: f32,
    aim_angle_index: u32,
    is_muzzle_flashing: bool,
}

impl PlayerAnimComponent {
    pub fn new() -> Self {
        Self {
            run_index: 0,
            is_crouching: false,
            is_jumping: false,
            flip_x: false,
            run_frame_time: 0.1,
            run_frame_time_acc: 0.0,
            aim_angle_index: 0,
            is_muzzle_flashing: false,
        }
    }
}

pub struct SpriteResources {
    pub sprites: Vec<Texture>,
}

impl SpriteResources {
    pub fn new() -> Self {
        Self {
            sprites: Vec::new(),
        }
    }

    pub fn insert(&mut self, texture: Texture) -> usize {
        let index = self.sprites.len();
        self.sprites.push(texture);
        index
    }
}

pub struct SpriteComponent {
    pub sprite_id: usize,
    pub sprite_width: u32,
    pub sprite_height: u32,
    pub sub_offset: Vec2F,
    pub vert_color: Rgba8,
    pub color_mode: ColorMode,
    pub flip: Vec2<bool>,
}

pub struct Shmup {
    tilesets: HashMap<i64, Tileset>,
    grids: Vec<TileGrid>,
    physics_data: PhysicsData,
    balls: Vec<RigidBodyHandle>,
    player_start: Vec2F,
    spawn_btn: VirtualButton,
    world: World,
    right_button: VirtualButton,
    left_button: VirtualButton,
    player_entity: hecs::Entity,
    player_texture: Texture,
    down_button: VirtualButton,
    jump_button: VirtualButton,
    camera: Camera,
    player_weapon_flash_texture: Texture,
    fire_button: VirtualButton,
    right_stick: VirtualStick,
    controller_switch_button: VirtualButton,
    use_controller: bool,
    player_projectile_texture: Texture,
    enemies_sheet_1_texture: Texture,
    sprite_resources: SpriteResources,
    time_since_last_phys_update: f32,
    render_debug_collision: bool,
    debug_btn: VirtualButton,
}

impl Game for Shmup {
    type Config = ();

    fn new(ctx: &Context, _cfg: Self::Config) -> Result<Self, GameError>
    where
        Self: Sized,
    {
        let mut tilesets = HashMap::new();
        let mut grids = Vec::new();
        let mut physics_data = PhysicsData::new();
        let mut world = World::new();
        let mut sprite_resources = SpriteResources::new();

        let player_texture = ctx
            .graphics
            .load_png_from_file("assets/char-sheet-alpha.png", true)
            .unwrap();
        let player_texture_id = sprite_resources.insert(player_texture);

        let player_weapon_flash_texture = ctx
            .graphics
            .load_png_from_file("assets/weaponflash-sheet-colour-1-alpha.png", true)
            .unwrap();

        let player_weapon_flash_texture_id = sprite_resources.insert(player_weapon_flash_texture);

        let player_projectile_texture = ctx
            .graphics
            .load_png_from_file("assets/projectiles-sheet-alpha.png", true)
            .unwrap();

        let player_projectile_texture_id = sprite_resources.insert(player_projectile_texture);

        let enemies_sheet_1_texture = ctx
            .graphics
            .load_png_from_file("assets/enemies-sheet-alpha.png", true)
            .unwrap();

        let enemies_sheet_1_texture_id = sprite_resources.insert(enemies_sheet_1_texture);

        // initialize your game state here, such as creating graphics resources, etc.
        let mut level_file =
            File::open("assets/levels/PlatformerTest.ldtk").expect("failed to open ldtk file");
        let mut serialized_level = String::new();
        level_file
            .read_to_string(&mut serialized_level)
            .expect("failed to read ldtk file!");
        let ldtk = serde_json::from_str::<LdtkJson>(&serialized_level)
            .expect("failed to parse ldtk file!");

        let mut player_start = None;

        let tile_sets = ldtk.defs.tilesets;
        for tile_set in tile_sets {
            if let Some(path) = tile_set.rel_path {
                let tile_texture = ctx.graphics.load_png_from_file(path, true).unwrap();

                let tile_types = tile_set
                    .custom_data
                    .iter()
                    .map(|custom_data| {
                        let tile_type = if let Ok(int) = custom_data.data.parse::<u8>() {
                            TileType::from(int)
                        } else {
                            TileType::None
                        };
                        (custom_data.tile_id, tile_type)
                    })
                    .collect::<HashMap<_, _>>();

                let tiles = tile_texture.split_into_tiles(Vec2U::new(
                    tile_set.tile_grid_size as u32,
                    tile_set.tile_grid_size as u32,
                ));

                tilesets.insert(
                    tile_set.uid,
                    Tileset {
                        tiles,
                        grid_size: tile_set.tile_grid_size,
                        tile_types,
                    },
                );
            }
        }
        for level in ldtk.levels {
            //println!("level: {}", level.identifier);
            //let mut merge_map: HashMap<i64, MinMax> = HashMap::new();

            if let Some(mut layers) = level.layer_instances {
                layers.reverse();
                for layer in layers {
                    //let mut voxels = Vec::new();
                    let grid_pos = Vec2F::new(level.world_x as f32, level.world_y as f32);
                    if layer.visible && layer.tileset_def_uid.is_some() {
                        //println!("layer: {}", layer.identifier);
                        let tiles = layer.auto_layer_tiles;
                        let mut colliders = Vec::new();
                        if layer.identifier == "Collisions" {
                            let tile_set = tilesets.get(&layer.tileset_def_uid.unwrap()).unwrap();
                            let size = tile_set.grid_size as f32 / 2.;
                            for tile in &tiles {
                                if let Some(tile_type) = tile_set.tile_types.get(&tile.t) {
                                    match tile_type {
                                        TileType::None => (),
                                        TileType::Collider => {
                                            let translation = grid_pos
                                                + Vec2F::new(tile.px[0] as f32, tile.px[1] as f32);

                                            //somehow we need a check for if this is contiguous
                                            /*                                          if let Some(minmax) =
                                                                                           merge_map.get_mut(&(translation.y as i64))
                                                                                       {
                                                                                           if minmax.min > translation.x as i64 {
                                                                                               minmax.min = translation.x as i64;
                                                                                           }

                                                                                           if minmax.max < translation.x as i64 {
                                                                                               minmax.max = translation.x as i64;
                                                                                           }
                                                                                       } else {
                                                                                           merge_map.insert(
                                                                                               translation.y as i64,
                                                                                               MinMax {
                                                                                                   min: translation.x as i64,
                                                                                                   max: translation.x as i64,
                                                                                                   size: size as i64,
                                                                                               },
                                                                                           );
                                                                                       }
                                            */
                                            let radius = 0.2;
                                            let collider = ColliderBuilder::round_cuboid(
                                                size - radius,
                                                size - radius,
                                                radius,
                                            )
                                            .translation(rapier2d::math::Vector2::new(
                                                translation.x + size,
                                                translation.y + size,
                                            ))
                                            .collision_groups(InteractionGroups::new(
                                                TERRAIN_GROUP,
                                                PLAYER_GROUP | ENEMY_GROUP,
                                                rapier2d::prelude::InteractionTestMode::And,
                                            ));
                                            /*                                             voxels.push(rapier2d::math::Vector2::new(
                                                translation.x as f32,
                                                translation.y as f32,
                                            )); */

                                            colliders
                                                .push(physics_data.collider_set.insert(collider));
                                        }
                                    }
                                }
                            }
                        }
                        grids.push(TileGrid {
                            int_grid_csv: layer.int_grid_csv,
                            width: layer.c_wid,
                            height: layer.c_hei,
                            pos: grid_pos,
                            tile_set_id: layer.tileset_def_uid.unwrap(),
                            tiles,
                            opacity: layer.opacity,
                            colliders,
                        });
                        /*                         if voxels.len() > 0 {
                            let tile_set = tilesets.get(&layer.tileset_def_uid.unwrap()).unwrap();

                            let collider = ColliderBuilder::voxels_from_points(
                                rapier2d::math::Vec2::splat(tile_set.grid_size as f32),
                                &voxels,
                            )
                            .translation(rapier2d::math::Vector2::new(level.world_x as f32, level.world_y as f32));

                            physics_data.collider_set.insert(collider);
                        } */
                    }

                    for entity_instance in layer.entity_instances {
                        match entity_instance.identifier.as_str() {
                            "Player" => {
                                player_start = Some(
                                    grid_pos
                                        + Vec2F::new(
                                            entity_instance.px[0] as f32,
                                            entity_instance.px[1] as f32,
                                        ),
                                );
                            }
                            "Mob" => {
                                let pos = grid_pos
                                    + Vec2F::new(
                                        entity_instance.px[0] as f32,
                                        entity_instance.px[1] as f32,
                                    );

                                let character_body = RigidBodyBuilder::kinematic_position_based()
                                    .translation(rapier2d::math::Vec2::new(
                                        pos.x,
                                        pos.y - TEST_ENEMY_HEIGHT - 2.,
                                    ));
                                let character_body =
                                    physics_data.rigid_body_set.insert(character_body);

                                let character_collider =
                                    ColliderBuilder::ball(TEST_ENEMY_WIDTH / 2.0) /* capsule_y(
                                            TEST_ENEMY_HEIGHT / 2.,
                                            TEST_ENEMY_WIDTH / 2.,
                                        ) */
                                        .collision_groups(InteractionGroups::new(
                                            ENEMY_GROUP,
                                            TERRAIN_GROUP | PLAYER_GROUP | PLAYER_PROJECTILE_GROUP,
                                            rapier2d::prelude::InteractionTestMode::Or,
                                        ));
                                let character_collider =
                                    physics_data.collider_set.insert_with_parent(
                                        character_collider,
                                        character_body,
                                        &mut physics_data.rigid_body_set,
                                    );
                                let entity = world.spawn((
                                    CharacterControllerComponent::new(
                                        character_body,
                                        character_collider,
                                    ),
                                    EnemyComponent {},
                                    HealthComponent { health: 100.0 },
                                ));
                                physics_data
                                    .collider_set
                                    .get_mut(character_collider)
                                    .unwrap()
                                    .user_data = entity.id() as u128;
                            }
                            _ => {}
                        }
                    }
                }
            }

            /*             for (y, minmax) in merge_map {
                let collider = ColliderBuilder::cuboid(
                    (minmax.max - minmax.min) as f32 / 2.,
                    minmax.size as f32,
                )
                .translation(rapier2d::math::Vector2::new(
                    (minmax.max - minmax.min) as f32 / 2.,
                    y as f32,
                ));
                physics_data.collider_set.insert(collider);
            } */
        }
        let player_start = if let Some(player_start) = player_start {
            player_start
        } else {
            Vec2F::ZERO
        };

        let src = VirtualSource::last_active(ctx);
        let spawn_btn = VirtualButton::new(&src, Key::Enter, None);
        let debug_btn = VirtualButton::new(&src, Key::Comma, None);

        let left_button = VirtualButton::new(&src, Key::A, GamepadButton::DPadLeft);
        let right_button = VirtualButton::new(&src, Key::D, GamepadButton::DPadRight);
        let down_button = VirtualButton::new(&src, Key::S, GamepadButton::DPadDown);

        let jump_button = VirtualButton::new(&src, Key::Space, GamepadButton::South);

        let fire_button = VirtualButton::new(&src, Key::F, GamepadButton::RightTrigger);

        let right_stick = VirtualStick::new(
            &src,
            VirtualAxis::new(&src, GamepadAxis::RightX, None, None),
            VirtualAxis::new(&src, GamepadAxis::RightY, None, None),
        );

        let character_body = RigidBodyBuilder::kinematic_position_based().translation(
            rapier2d::math::Vec2::new(player_start.x, player_start.y - PLAYER_HEIGHT - 2.),
        );
        let character_body = physics_data.rigid_body_set.insert(character_body);

        let character_collider = ColliderBuilder::capsule_y(PLAYER_HEIGHT / 2., PLAYER_WIDTH / 2.)
            .collision_groups(InteractionGroups::new(
                PLAYER_GROUP,
                TERRAIN_GROUP | ENEMY_GROUP | ENEMY_PROJECTILE_GROUP,
                rapier2d::prelude::InteractionTestMode::Or,
            ));
        let character_collider = physics_data.collider_set.insert_with_parent(
            character_collider,
            character_body,
            &mut physics_data.rigid_body_set,
        );
        let player_entity = world.spawn((
            PlayerComponent::new(),
            CharacterControllerComponent::new(character_body, character_collider),
            PlayerAnimComponent::new(),
        ));
        physics_data
            .collider_set
            .get_mut(character_collider)
            .unwrap()
            .user_data = player_entity.id() as u128;

        let player_texture = ctx
            .graphics
            .load_png_from_file("assets/char-sheet-alpha.png", true)
            .unwrap();

        let player_weapon_flash_texture = ctx
            .graphics
            .load_png_from_file("assets/weaponflash-sheet-colour-1-alpha.png", true)
            .unwrap();

        let player_projectile_texture = ctx
            .graphics
            .load_png_from_file("assets/projectiles-sheet-alpha.png", true)
            .unwrap();

        let enemies_sheet_1_texture = ctx
            .graphics
            .load_png_from_file("assets/enemies-sheet-alpha.png", true)
            .unwrap();

        let zoom_amount = 1.;

        let scale = zoom_amount * 2.0 * ctx.window.scale_factor();
        let window_size = to_logical_f(ctx.window.size().to_f32(), scale);

        Ok(Self {
            tilesets,
            grids,
            physics_data,
            balls: Vec::new(),
            player_start,
            spawn_btn,
            world,
            left_button,
            right_button,
            down_button,
            jump_button,
            fire_button,
            controller_switch_button: VirtualButton::new(&src, Key::O, None),
            use_controller: false,
            right_stick,
            player_entity,
            player_texture,
            camera: Camera::new(
                vec2(player_start.x, player_start.y),
                window_size.x,
                window_size.y,
                1.,
            ),
            player_weapon_flash_texture,
            player_projectile_texture,
            enemies_sheet_1_texture,
            sprite_resources,
            time_since_last_phys_update: 0.,
            render_debug_collision: false,
            debug_btn,
        })
    }

    fn update(&mut self, ctx: &Context) -> Result<(), GameError> {
        //update inputs
        if let Ok((player)) = self
            .world
            .query_one_mut::<(&mut PlayerComponent)>(self.player_entity)
        {
            if self.right_button.pressed() {
                player.desired_velocity.x = 1.0;
            } else if self.right_button.released() {
                player.desired_velocity.x = 0.0;
            }
            if self.left_button.pressed() {
                player.desired_velocity.x = -1.0;
            } else if self.left_button.released() {
                player.desired_velocity.x = 0.0;
            }

            if self.fire_button.down() || ctx.mouse.left_down() {
                player.wants_to_shoot = true;
            } else {
                player.wants_to_shoot = false;
            }

            if self.jump_button.pressed() {
                player.wants_to_jump = true;
            } else if self.jump_button.released() {
                player.wants_to_jump = false;
            }
        }

        if self.debug_btn.pressed() {
            self.render_debug_collision = !self.render_debug_collision;
        }

        self.time_since_last_phys_update += ctx.dt();

        let min_physics_time = 1. / 60.;
        while self.time_since_last_phys_update >= min_physics_time {
            let dt = min_physics_time;
            // perform your game logic here
            let scale = self.camera.zoom_amount * 2.0 * ctx.window.scale_factor();
            let window_size = to_logical_f(ctx.window.size().to_f32(), scale);
            self.camera.width = window_size.x;
            self.camera.height = window_size.y;

            let mouse_world_pos = to_logical_f(ctx.mouse.pos(), scale) + self.camera.position;

            if self.controller_switch_button.pressed() {
                self.use_controller = !self.use_controller;
            }

            if self.spawn_btn.pressed() {
                let rigid_body = RigidBodyBuilder::dynamic()
                    .translation(rapier2d::math::Vector2::new(
                        self.player_start.x,
                        self.player_start.y - 10.,
                    ))
                    .build();
                let collider = ColliderBuilder::ball(5.).restitution(0.7).build();
                let ball_body_handle = self.physics_data.rigid_body_set.insert(rigid_body);
                self.physics_data.collider_set.insert_with_parent(
                    collider,
                    ball_body_handle,
                    &mut self.physics_data.rigid_body_set,
                );

                self.balls.push(ball_body_handle);
            }

            if let Ok((player_entity, player, character)) =
                self.world
                    .query_one_mut::<(Entity, &mut PlayerComponent, &CharacterControllerComponent)>(
                        self.player_entity,
                    )
            {
                /*                 if self.right_button.pressed() {
                    player.desired_velocity.x = 1.0;
                } else if self.right_button.released() {
                    player.desired_velocity.x = 0.0;
                }
                if self.left_button.pressed() {
                    player.desired_velocity.x = -1.0;
                } else if self.left_button.released() {
                    player.desired_velocity.x = 0.0;
                }

                if self.fire_button.down() || ctx.mouse.left_down() {
                    player.wants_to_shoot = true;
                } else {
                    player.wants_to_shoot = false;
                } */

                if player.is_shooting && player.time_since_shot_start >= player.time_per_shot {
                    player.is_shooting = false;
                } else if player.is_shooting {
                    player.time_since_shot_start += dt; //ctx.dt();
                }

                if player.time_since_last_shot >= player.shot_delay_time
                    && player.wants_to_shoot
                    && !player.is_shooting
                {
                    player.time_since_last_shot = 0.;
                    player.time_since_shot_start = 0.;
                    player.is_shooting = true;

                    //spawn projectile here

                    let body_translation =
                        self.physics_data.rigid_body_set[character.character_body].translation();
                    let body_translation = Vec2F::new(body_translation.x, body_translation.y);

                    let bullet_spawn_dist = 15.0;
                    let bullet_speed = 10.0;
                    let bullet_pos = body_translation + (player.aim_dir * bullet_spawn_dist);
                    let bullet_vel = player.aim_dir * bullet_speed;

                    /*                 let rigid_body = RigidBodyBuilder::dynamic()
                        .translation(rapier2d::math::Vector2::new(bullet_pos.x, bullet_pos.y))
                        .linvel(rapier2d::math::Vec2::new(bullet_vel.x, bullet_vel.y))
                        .build();
                    let collider = ColliderBuilder::ball(5.).restitution(0.7).build();
                    let ball_body_handle = self.physics_data.rigid_body_set.insert(rigid_body);
                    self.physics_data.collider_set.insert_with_parent(
                        collider,
                        ball_body_handle,
                        &mut self.physics_data.rigid_body_set,
                    );

                    self.balls.push(ball_body_handle); */
                    self.world.spawn((
                        RaycastProjectile {
                            position: bullet_pos,
                            velocity: bullet_vel,
                            solid: true,
                            life_time: 0.0,
                            max_life_time: 5.0,
                            penetrate: false,
                        },
                        PlayerOwned {
                            owner: player_entity,
                        },
                    ));
                } else if !player.is_shooting {
                    player.time_since_last_shot += dt; //ctx.dt();
                }
            }

            let _jump_height = 200.;
            let walk_speed = 4.0;

            for (player, character_controller) in self
                .world
                .query_mut::<(&mut PlayerComponent, &mut CharacterControllerComponent)>()
            {
                if player.wants_to_jump {
                    //println!("jump pressed");

                    if character_controller.is_grounded {
                        player.wants_to_jump = false;
                        player.desired_velocity.y = -self.physics_data.gravity.y * 1.5;
                        /*  player.desired_velocity.y = -self.physics_data.gravity.y
                         * (2.0 * jump_height / self.physics_data.gravity.y).sqrt();  */
                    }
                }
                player.desired_velocity.y -= -self.physics_data.gravity.y * dt; //ctx.dt();

                /* else {
                if player.desired_velocity.y > 0. {
                //player.desired_velocity.y -= -self.physics_data.gravity.y * ctx.dt();
                }
                else {
                player.desired_velocity.y = 0.;
                }
                //player.desired_velocity.y = 0.;
                } */

                let mut desired_velocity = player.desired_velocity;

                /* if character_controller.is_grounded {
                               desired_velocity.x *= 100.0;
                           } else {
                               desired_velocity.y = 0.;
                           }
                */
                desired_velocity.x *= walk_speed;

                character_controller.desired_movement = desired_velocity;

                let body = &self.physics_data.rigid_body_set[character_controller.character_body];
                let translation = body.position().translation;

                //self.camera.position.x = translation.x + 24.0 / 2.0 - self.camera.width as f32 / 2.;
                //self.camera.position.y = translation.y + 32.0 / 2.0 - self.camera.height as f32 / 2.;
                self.camera.position.x = translation.x + 24.0 / 2.0 - self.camera.width / 2.;
                self.camera.position.y = translation.y + 32.0 / 2.0 - self.camera.height / 2.; // + self.camera.height as f32;

                if self.use_controller {
                    let diff = Vec2F::new(self.right_stick.x(), self.right_stick.y()); //mouse_pos - body_pos;
                    let mut angle = f32::atan2(diff.y, diff.x);
                    if angle < 0. {
                        angle += f32::PI * 2.;
                    }
                    player.aim_angle = angle * (180. / f32::PI);
                    player.aim_dir = diff.norm();
                } else {
                    let body_pos = body.position().translation;
                    let body_pos = Vec2F::new(body_pos.x, body_pos.y);
                    let diff = mouse_world_pos - body_pos; //mouse_pos - body_pos;
                    let mut angle = f32::atan2(diff.y, diff.x);
                    //angle += f32::PI * 0.25;
                    if angle < 0. {
                        angle += f32::PI * 2.;
                    }
                    player.aim_angle = angle * (180. / f32::PI);
                    player.aim_dir = diff.norm();
                }
                //println!("mouse pos: {} body pos: {}", mouse_world_pos, body_pos);
                //println!("aim angle: {}", player.aim_angle);
            }

            step(&mut self.physics_data);

            for character_controller in self.world.query_mut::<&mut CharacterControllerComponent>()
            {
                character_controller.update_character_controller(dt, &mut self.physics_data);
            }

            let player_proj_filter = QueryFilter::new().groups(InteractionGroups::new(
                PLAYER_PROJECTILE_GROUP,
                ENEMY_GROUP,
                rapier2d::prelude::InteractionTestMode::And,
            ));

            let query_pipeline = self.physics_data.broad_phase.as_query_pipeline(
                self.physics_data.narrow_phase.query_dispatcher(),
                &self.physics_data.rigid_body_set,
                &self.physics_data.collider_set,
                player_proj_filter,
            );

            let mut projectile_impacts = Vec::new();

            let mut to_despawn = Vec::new();

            for (entity, raycast_projectile, _) in
                self.world
                    .query_mut::<(Entity, &mut RaycastProjectile, &PlayerOwned)>()
            {
                let mut hit = false;
                let direction = raycast_projectile.velocity.norm();
                if let Some((handle, _toi)) = query_pipeline.cast_ray(
                    &Ray::new(
                        rapier2d::math::Vec2::new(
                            raycast_projectile.position.x,
                            raycast_projectile.position.y,
                        ),
                        rapier2d::math::Vec2::new(direction.x, direction.y),
                    ),
                    raycast_projectile.velocity.len(),
                    raycast_projectile.solid,
                ) {
                    let hit_collider = &self.physics_data.collider_set[handle];
                    projectile_impacts.push(ProjectileImpact {
                        hit_entity_id: hit_collider.user_data as u32,
                        _proj_entity: entity,
                    });
                    hit = true;
                }

                raycast_projectile.position += raycast_projectile.velocity;

                raycast_projectile.life_time += dt; //ctx.dt();
                if raycast_projectile.life_time >= raycast_projectile.max_life_time
                    || (hit && !raycast_projectile.penetrate)
                {
                    to_despawn.push(entity);
                }
            }

            for projectile_impact in projectile_impacts {
                let hit_entity = unsafe {
                    self.world
                        .find_entity_from_id(projectile_impact.hit_entity_id)
                };
                to_despawn.push(hit_entity);
            }

            for entity in to_despawn {
                if let Ok(character_controller) = self
                    .world
                    .query_one_mut::<&CharacterControllerComponent>(entity)
                {
                    self.physics_data.rigid_body_set.remove(
                        character_controller.character_body,
                        &mut self.physics_data.island_manager,
                        &mut self.physics_data.collider_set,
                        &mut self.physics_data.impulse_joint_set,
                        &mut self.physics_data.multibody_joint_set,
                        true,
                    );
                    /*                 self.physics_data.collider_set.remove(
                        character_controller.character_collider,
                        &mut self.physics_data.island_manager,
                        &mut self.physics_data.rigid_body_set,
                        true,
                    ); */
                }

                self.world.despawn(entity).unwrap();
            }

            self.time_since_last_phys_update -= dt;
        }

        Ok(())
    }

    fn render(&mut self, ctx: &Context, draw: &mut Draw) -> Result<(), GameError> {
        // perform your drawing code here

        // draw a background color to the window
        draw.set_surface(None, rgb(0x476c6c));

        let scale = self.camera.zoom_amount * 2.0 * ctx.window.scale_factor();

        // let's scale everything up
        draw.push_scale_of(scale);

        for grid in &self.grids {
            let tile_set = self.tilesets.get(&grid.tile_set_id).unwrap();
            /*let tiles = tile_set.tiles.as_slice();
                         for (i, tile_id) in grid.int_grid_csv.iter().enumerate() {
                let offset = Vec2F::new(i as f32 % grid.width as f32, i as f32 / grid.height as f32);
                //draw.push_translation(grid.pos + offset);

                let sub = &tiles[*tile_id as usize + 1];

                draw.subtexture_at(sub, offset);
            } */
            let mut white = Rgba8::WHITE;
            white.a = (grid.opacity * 255.0) as u8;
            //println!("opacity: {}", white.a);
            for tile in &grid.tiles {
                let pos = grid.pos + Vec2F::new(tile.px[0] as f32, tile.px[1] as f32);
                //draw.push_translation();
                /*                 println!(
                    "src: {},{} coord: {},{}",
                    tile.src[0],
                    tile.src[1],
                    tile.src[0] as u32 / tile_set.grid_size as u32,
                    tile.src[1] as u32 / tile_set.grid_size as u32,
                ); */
                let sub = tile_set
                    .tiles
                    .get(
                        tile.src[0] as u32 / tile_set.grid_size as u32,
                        tile.src[1] as u32 / tile_set.grid_size as u32,
                    )
                    .unwrap();

                let flip = match tile.f {
                    0 => {
                        Vec2::new(false, false)
                        //no flip
                    }
                    1 => Vec2::new(true, false), //x flip only
                    2 => Vec2::new(false, true), //y flip only
                    3 => Vec2::new(true, true),  //both flips
                    _ => panic!("unrecognized flip!"),
                };

                draw.subtexture_at_flipped(
                    sub,
                    pos - self.camera.position,
                    white,
                    ColorMode::MULT,
                    flip,
                );
                //draw.subtexture_at(sub, pos);
            }
        }

        for ball in &self.balls {
            let ball_body = &self.physics_data.rigid_body_set[*ball];
            let translation = ball_body.translation();

            draw.circle(
                circle(
                    Vec2F::new(translation.x, translation.y) - self.camera.position,
                    5.,
                ),
                Rgba8::BLACK,
                None,
            );
        }

        for projectile in self.world.query_mut::<&RaycastProjectile>() {
            let flip_x = projectile.velocity.x < 0.;
            let flip_y = projectile.velocity.y > 0.;

            let diff = projectile.velocity.norm();
            Vec2F::new(self.right_stick.x(), self.right_stick.y());
            let mut angle = f32::atan2(diff.y, diff.x);
            if angle < 0. {
                angle += f32::PI * 2.;
            }
            angle = angle * (180. / f32::PI);

            let aim_angle = if flip_x {
                let x = 180.0 - angle;
                let x = if x < 0. { 360. + x } else { x };
                360.0 - x
            } else {
                360.0 - angle
            };

            let four_quad_size = 360.0 / 4.0;
            let quadrant = (aim_angle / four_quad_size).floor();

            let aim_angle = aim_angle - (quadrant) * 90.0;

            let aim_angle = if flip_y { 90.0 - aim_angle } else { aim_angle };

            let quadrant_size = 90.0 / 12.0;
            let quadrant_index = (aim_angle / quadrant_size).ceil() as u32;

            let sub =
                self.player_projectile_texture
                    .sub(rect(quadrant_index as u32 * 18, 0, 18, 18));

            draw.subtexture_at_flipped(
                sub,
                Vec2F::new(projectile.position.x - 18., projectile.position.y - 18.)
                    - self.camera.position,
                Rgba8::WHITE,
                ColorMode::MULT,
                Vec2::new(flip_x, flip_y),
            );
        }

        for (player_anim, player, character_controller) in self.world.query_mut::<(
            &mut PlayerAnimComponent,
            &PlayerComponent,
            &CharacterControllerComponent,
        )>() {
            let body = &self.physics_data.rigid_body_set[character_controller.character_body];
            let translation = body.translation();
            /*             draw.circle(
                circle(Vec2F::new(translation.x, translation.y), 5.),
                Rgba8::RED,
                None,
            ); */
            //let lin_vel = body.linvel();

            let mut animation_x = 0;

            player_anim.is_muzzle_flashing = player.is_shooting;

            if !character_controller.is_grounded {
                player_anim.is_jumping = true;
            } else {
                player_anim.is_jumping = false;
            }
            if self.down_button.pressed() {
                player_anim.is_crouching = true;
            } else if self.down_button.released() {
                player_anim.is_crouching = false;
            }

            let is_running = player.desired_velocity.x.abs() > 0.; //lin_vel.x.abs() > 0.;

            if is_running && !player_anim.is_jumping {
                player_anim.run_frame_time_acc += ctx.dt();
                if player_anim.run_frame_time_acc > player_anim.run_frame_time {
                    player_anim.run_index = inc_wrap_index(player_anim.run_index, 4);
                    player_anim.run_frame_time_acc = 0.;
                }
                animation_x = player_anim.run_index + 2;
            } else {
                player_anim.run_index = 0;

                if player_anim.is_crouching || player_anim.is_jumping {
                    animation_x = 1;
                }
            }

            /* if lin_vel.x > 0. {
                player_anim.flip_x = false;
            } else if lin_vel.x < 0. {
                player_anim.flip_x = true;
            } */
            if player.desired_velocity.x > 0. {
                player_anim.flip_x = false;
            } else if player.desired_velocity.x < 0. {
                player_anim.flip_x = true;
            }

            let aim_angle = if player_anim.flip_x {
                let x = 180.0 - player.aim_angle; //(player.aim_angle - 360.0).abs()
                let x = if x < 0. { 360. + x } else { x };
                360.0 - x
            } else {
                360.0 - player.aim_angle
            };
            let quadrant_size = 360.0 / 47.0;
            let quadrant_index = (aim_angle / quadrant_size).ceil() as u32;
            player_anim.aim_angle_index = quadrant_index;

            let animation_y = player_anim.aim_angle_index * 46;

            let sub = self
                .player_texture
                .sub(rect(animation_x as u32 * 46, animation_y, 46, 46));

            let character_pos = Vec2F::new(
                translation.x - 24.,
                translation.y - 38. + PLAYER_HEIGHT / 2.0,
            ) - self.camera.position;

            draw.subtexture_at_flipped(
                sub,
                character_pos,
                Rgba8::WHITE,
                ColorMode::MULT,
                Vec2::new(player_anim.flip_x, false),
            );

            if player_anim.is_muzzle_flashing {
                let sub_weapon_flash = self.player_weapon_flash_texture.sub(rect(
                    animation_x as u32 * 46,
                    player_anim.aim_angle_index * 54,
                    46,
                    54,
                ));

                draw.subtexture_at_flipped(
                    sub_weapon_flash,
                    character_pos,
                    Rgba8::WHITE,
                    ColorMode::MULT,
                    Vec2::new(player_anim.flip_x, false),
                );
            }

            /*             let rect_center = Vec2F::new(
                translation.x - PLAYER_WIDTH / 2.0,
                translation.y - PLAYER_HEIGHT / 2.0,
            ) - self.camera.position;
            draw.rect_outline(
                rect(rect_center.x, rect_center.y, PLAYER_WIDTH, PLAYER_HEIGHT),
                Rgba::GREEN,
            );

            draw.circle(
                circle(
                    Vec2F::new(translation.x, translation.y) - self.camera.position,
                    5.,
                ),
                Rgba8::GREEN,
                None,
            ); */
        }

        for (_, character_controller) in self
            .world
            .query_mut::<(&EnemyComponent, &CharacterControllerComponent)>()
        {
            let body = &self.physics_data.rigid_body_set[character_controller.character_body];
            let translation = body.translation();

            /* let rect_center = Vec2F::new(
                translation.x, //- TEST_ENEMY_WIDTH / 2.0,
                translation.y, //- TEST_ENEMY_HEIGHT / 2.0,
            ) - self.camera.position;
                        draw.rect_outline(
                rect(
                    rect_center.x,
                    rect_center.y,
                    TEST_ENEMY_WIDTH,
                    TEST_ENEMY_HEIGHT,
                ),
                Rgba::RED,
            ); */
            //draw.circle_outline(circle(rect_center, TEST_ENEMY_WIDTH / 2.0), Rgba::RED, None);

            let sub = self.enemies_sheet_1_texture.sub(rect(0, 58, 20, 24));

            draw.subtexture_at_flipped(
                sub,
                Vec2F::new(
                    translation.x - 10.,
                    translation.y - 24. + TEST_ENEMY_HEIGHT / 2.0,
                ) - self.camera.position,
                Rgba8::WHITE,
                ColorMode::MULT,
                Vec2::new(false, false),
            );
        }

        if self.render_debug_collision {
            for character_controller in self.world.query_mut::<&CharacterControllerComponent>() {
                draw_physics_shape(
                    draw,
                    &self.physics_data,
                    character_controller.character_collider,
                    self.camera.position,
                    Rgba8::GREEN,
                );
            }

            for tile_grid in &self.grids {
                for collider in &tile_grid.colliders {
                    draw_physics_shape(
                        draw,
                        &self.physics_data,
                        *collider,
                        self.camera.position,
                        Rgba8::BLACK,
                    );
                }
            }
        }

        draw.circle(
            circle(to_logical_f(ctx.mouse.pos(), scale), 5.),
            Rgba8::RED,
            None,
        );

        /*         draw.circle(circle(Vec2F::new(0., 0.), 5.), Rgba8::GREEN, None);

        let window_center = ctx.window.center();
        draw.circle(
            circle(
                Vec2F::new(window_center.x as f32, window_center.y as f32) / scale,
                5.,
            ),
            Rgba8::BLUE,
            None,
        ); */

        Ok(())
    }
}

pub fn draw_physics_shape(
    draw: &mut Draw,
    physics_data: &PhysicsData,
    collider_handle: ColliderHandle,
    camera_pos: Vec2F,
    color: Rgba8,
) {
    let collider = &physics_data.collider_set[collider_handle];
    let translation = collider.translation();
    match collider.shape().shape_type() {
        rapier2d::prelude::ShapeType::Ball => {
            let ball = collider.shape().as_ball().unwrap();
            let translation = Vec2F::new(translation.x, translation.y) - camera_pos;
            draw.circle_outline(circle(translation, ball.radius), color, None);
        }
        rapier2d::prelude::ShapeType::Cuboid => {
            let cuboid = collider.shape().as_cuboid().unwrap();

            let rect_center = Vec2F::new(
                translation.x - cuboid.half_extents.x,
                translation.y - cuboid.half_extents.y,
            ) - camera_pos;
            draw.rect_outline(
                rect(
                    rect_center.x,
                    rect_center.y,
                    cuboid.half_extents.x * 2.0,
                    cuboid.half_extents.y * 2.0,
                ),
                color,
            );
        }
        rapier2d::prelude::ShapeType::Capsule => {
            let capsule = collider.shape().as_capsule().unwrap();
            let aabb = capsule.local_aabb();
            let half = aabb.half_extents();
            let rect_center =
                Vec2F::new(translation.x - half.x, translation.y - half.y) - camera_pos;
            draw.rect_outline(
                rect(rect_center.x, rect_center.y, half.x * 2.0, half.y * 2.0),
                color,
            );
        }
        rapier2d::prelude::ShapeType::Segment => todo!(),
        rapier2d::prelude::ShapeType::Triangle => todo!(),
        rapier2d::prelude::ShapeType::Voxels => todo!(),
        rapier2d::prelude::ShapeType::TriMesh => todo!(),
        rapier2d::prelude::ShapeType::Polyline => todo!(),
        rapier2d::prelude::ShapeType::HalfSpace => todo!(),
        rapier2d::prelude::ShapeType::HeightField => todo!(),
        rapier2d::prelude::ShapeType::Compound => todo!(),
        rapier2d::prelude::ShapeType::ConvexPolygon => todo!(),
        rapier2d::prelude::ShapeType::RoundCuboid => {
            let shape = collider.shape().as_round_cuboid().unwrap();
            let cuboid = shape.inner_shape;

            let rect_center = Vec2F::new(
                translation.x - cuboid.half_extents.x,
                translation.y - cuboid.half_extents.y,
            ) - camera_pos;
            draw.rect_outline(
                rect(
                    rect_center.x,
                    rect_center.y,
                    cuboid.half_extents.x * 2.0,
                    cuboid.half_extents.y * 2.0,
                ),
                color,
            );
        }
        rapier2d::prelude::ShapeType::RoundTriangle => todo!(),
        rapier2d::prelude::ShapeType::RoundConvexPolygon => todo!(),
        rapier2d::prelude::ShapeType::Custom => todo!(),
    }
}

pub fn inc_wrap_index(index: usize, len: usize) -> usize {
    (index + 1) % len
}

pub struct MinMax {
    pub min: i64,
    pub max: i64,
    pub size: i64,
}
