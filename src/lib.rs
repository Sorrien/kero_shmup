use std::{
    collections::HashMap,
    fs::File,
    io::{BufReader, Read},
};

use crate::{
    audio::{AudioEmitter, load_audio, setup_output_stream, spawn_sound, update_audio_emitters},
    hierarchy::TransformComponent,
    ldtk::{TileGrid, TileType, Tileset},
    ldtk_json::LdtkJson,
    physics::{
        CharacterControllerComponent, PhysicsData, physics_scale_to_pixels,
        physics_scale_to_pixels_f, physics_scale_to_pixels_glam, pixel_scale_to_physics_f,
        pixel_scale_to_physics_glam, step,
    },
    player::PlayerComponent,
};
use cpal::traits::{DeviceTrait, HostTrait};
use glamx::Affine2;
use hecs::{Entity, World};
use kero::{grid::Grid, prelude::*};
use rapier2d::prelude::{
    ActiveEvents, ChannelEventCollector, ColliderBuilder, ColliderHandle, Group, InteractionGroups,
    QueryFilter, Ray, RigidBodyBuilder, RigidBodyHandle, SharedShape,
};
use serde::{Deserialize, Serialize};

pub mod audio;
pub mod hierarchy;
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
    frames_ungrounded: u32,
    flip_x: bool,
    run_frame_time: f32,
    run_frame_time_acc: f32,
    aim_angle_index: u32,
    is_muzzle_flashing: bool,
    prev_anim_x: u32,
    prev_anim_y: u32,
}

impl PlayerAnimComponent {
    pub fn new(run_frame_time: f32) -> Self {
        Self {
            run_index: 0,
            is_crouching: false,
            is_jumping: false,
            flip_x: false,
            run_frame_time,
            run_frame_time_acc: 0.0,
            aim_angle_index: 0,
            is_muzzle_flashing: false,
            prev_anim_x: 0,
            prev_anim_y: 0,
            frames_ungrounded: 0,
        }
    }
}

#[allow(unused)]
pub struct AudioData {
    sound_scene_handle: oddio::SpatialSceneControl,
    stream: cpal::Stream,
    sample_rate: u32,
    host: cpal::Host,
    device: cpal::Device,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct SoundCueConfig {
    pub id: String,
    pub sounds: Vec<String>,
}

pub struct SoundCue {
    pub sounds: Vec<usize>,
}

pub fn read_file_to_bytes(path: &str) -> Result<Vec<u8>, std::io::Error> {
    let file = File::open(path)?;
    let mut file = BufReader::new(file);
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes)?;
    Ok(bytes)
}

#[allow(unused)]
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
    step_button: VirtualButton,
    camera: Camera,
    player_weapon_flash_texture: Texture,
    fire_button: VirtualButton,
    right_stick: VirtualStick,
    controller_switch_button: VirtualButton,
    use_controller: bool,
    player_projectile_texture: Texture,
    enemies_sheet_1_texture: Texture,
    render_debug_collision: bool,
    debug_btn: VirtualButton,
    pause: bool,
    pause_btn: VirtualButton,
    event_handler: ChannelEventCollector,
    collision_recv: std::sync::mpsc::Receiver<rapier2d::prelude::CollisionEvent>,
    contact_force_recv: std::sync::mpsc::Receiver<rapier2d::prelude::ContactForceEvent>,
    cfg: GameConfig,
    audio: AudioData,
    audio_device_check_timer: f32,
    audio_device_check_time: f32,
    sounds: Vec<std::sync::Arc<oddio::Frames<f32>>>,
    sound_metadata: Vec<audio::AudioSample>,
    sound_cues: HashMap<String, SoundCue>,
    rand: Rand,
}

impl Game for Shmup {
    type Config = GameConfig;

    fn new(ctx: &Context, cfg: Self::Config) -> Result<Self, GameError>
    where
        Self: Sized,
    {
        // initialize your game state here, such as creating graphics resources, etc.
        let mut tilesets = HashMap::new();
        let mut grids = Vec::new();

        let mut physics_data = PhysicsData::new();
        let (collision_send, collision_recv) = std::sync::mpsc::channel();
        let (contact_force_send, contact_force_recv) = std::sync::mpsc::channel();
        let event_handler = ChannelEventCollector::new(collision_send, contact_force_send);

        let mut world = World::new();

        let player_texture = ctx
            .graphics
            .load_png_from_memory(
                &read_file_to_bytes("assets/char-sheet-alpha.png").unwrap(),
                true,
            )
            .unwrap();

        let player_weapon_flash_texture = ctx
            .graphics
            .load_png_from_memory(
                &read_file_to_bytes("assets/weaponflash-sheet-colour-1-alpha.png").unwrap(),
                true,
            )
            .unwrap();

        let player_projectile_texture = ctx
            .graphics
            .load_png_from_memory(
                &read_file_to_bytes("assets/projectiles-sheet-alpha.png").unwrap(),
                true,
            )
            .unwrap();

        let enemies_sheet_1_texture = ctx
            .graphics
            .load_png_from_memory(
                &read_file_to_bytes("assets/enemies-sheet-alpha.png").unwrap(),
                true,
            )
            .unwrap();

        let mut level_file = File::open(&cfg.default_level).expect("failed to open ldtk file");
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
                let tile_texture = ctx
                    .graphics
                    .load_png_from_memory(&read_file_to_bytes(&path).unwrap(), true)
                    .unwrap();

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
            if let Some(mut layers) = level.layer_instances {
                layers.reverse();
                for layer in layers {
                    let grid_pos = Vec2F::new(level.world_x as f32, level.world_y as f32);
                    if layer.visible && layer.tileset_def_uid.is_some() {
                        let tiles = layer.auto_layer_tiles;
                        let mut colliders = Vec::new();
                        if layer.identifier == "Collisions" {
                            let mut vert_slices: HashMap<i64, Vec<i64>> = HashMap::new();
                            let tile_set = tilesets.get(&layer.tileset_def_uid.unwrap()).unwrap();
                            let size = tile_set.grid_size as f32 / 2.;
                            for tile in &tiles {
                                if let Some(tile_type) = tile_set.tile_types.get(&tile.t) {
                                    match tile_type {
                                        TileType::None => (),
                                        TileType::Collider => {
                                            let tile_x = tile.px[0];
                                            let tile_y = tile.px[1];
                                            if cfg.merge_tiles {
                                                if vert_slices.contains_key(&tile_y) {
                                                    if let Some(slice) =
                                                        vert_slices.get_mut(&tile_y)
                                                    {
                                                        slice.push(tile_x);
                                                    }
                                                } else {
                                                    let slice = vec![tile_x];
                                                    vert_slices.insert(tile_y, slice);
                                                }
                                            } else {
                                                let translation = grid_pos
                                                    + Vec2F::new(tile_x as f32, tile_y as f32);

                                                let radius = 0.2;
                                                let collider = ColliderBuilder::round_cuboid(
                                                    pixel_scale_to_physics_f(size - radius),
                                                    pixel_scale_to_physics_f(size - radius),
                                                    pixel_scale_to_physics_f(radius),
                                                )
                                                .translation(pixel_scale_to_physics_glam(
                                                    rapier2d::math::Vector2::new(
                                                        translation.x + size,
                                                        translation.y + size,
                                                    ),
                                                ))
                                                .collision_groups(InteractionGroups::new(
                                                    TERRAIN_GROUP,
                                                    PLAYER_GROUP | ENEMY_GROUP,
                                                    rapier2d::prelude::InteractionTestMode::And,
                                                ));

                                                colliders.push(
                                                    physics_data.collider_set.insert(collider),
                                                );
                                            }
                                        }
                                    }
                                }
                            }

                            if cfg.merge_tiles {
                                for (y, slice) in &mut vert_slices {
                                    slice.sort();
                                    if slice.len() == 1 {
                                        let radius = 0.2;
                                        let collider = ColliderBuilder::round_cuboid(
                                            pixel_scale_to_physics_f(size - radius),
                                            pixel_scale_to_physics_f(size - radius),
                                            pixel_scale_to_physics_f(radius),
                                        )
                                        .translation(pixel_scale_to_physics_glam(
                                            rapier2d::math::Vector2::new(
                                                grid_pos.x + slice[0] as f32 + size,
                                                grid_pos.y + *y as f32 + size,
                                            ),
                                        ))
                                        .collision_groups(InteractionGroups::new(
                                            TERRAIN_GROUP,
                                            PLAYER_GROUP | ENEMY_GROUP,
                                            rapier2d::prelude::InteractionTestMode::And,
                                        ));
                                        colliders.push(physics_data.collider_set.insert(collider));
                                    } else {
                                        let mut start_index = 0;
                                        for i in 1..slice.len() {
                                            let x = slice[i];
                                            let x_2 = slice[i - 1];
                                            if (x - x_2).abs() > tile_set.grid_size {
                                                let length = (i - start_index) as f32
                                                    * tile_set.grid_size as f32;
                                                let half_length = length / 2.0;

                                                let radius = 0.2;
                                                let collider = ColliderBuilder::round_cuboid(
                                                    pixel_scale_to_physics_f(half_length - radius),
                                                    pixel_scale_to_physics_f(size - radius),
                                                    pixel_scale_to_physics_f(radius),
                                                )
                                                .translation(pixel_scale_to_physics_glam(
                                                    rapier2d::math::Vector2::new(
                                                        grid_pos.x
                                                            + slice[start_index] as f32
                                                            + half_length,
                                                        grid_pos.y + *y as f32 + size,
                                                    ),
                                                ))
                                                .collision_groups(InteractionGroups::new(
                                                    TERRAIN_GROUP,
                                                    PLAYER_GROUP | ENEMY_GROUP,
                                                    rapier2d::prelude::InteractionTestMode::And,
                                                ));

                                                colliders.push(
                                                    physics_data.collider_set.insert(collider),
                                                );
                                                start_index = i;
                                            }

                                            if i == slice.len() - 1 {
                                                let length = (i + 1 - start_index) as f32
                                                    * tile_set.grid_size as f32;
                                                let half_length = length / 2.0;

                                                let radius = 0.2;
                                                let collider = ColliderBuilder::round_cuboid(
                                                    pixel_scale_to_physics_f(half_length - radius),
                                                    pixel_scale_to_physics_f(size - radius),
                                                    pixel_scale_to_physics_f(radius),
                                                )
                                                .translation(pixel_scale_to_physics_glam(
                                                    rapier2d::math::Vector2::new(
                                                        grid_pos.x
                                                            + slice[start_index] as f32
                                                            + half_length,
                                                        grid_pos.y + *y as f32 + size,
                                                    ),
                                                ))
                                                .collision_groups(InteractionGroups::new(
                                                    TERRAIN_GROUP,
                                                    PLAYER_GROUP | ENEMY_GROUP,
                                                    rapier2d::prelude::InteractionTestMode::And,
                                                ));

                                                colliders.push(
                                                    physics_data.collider_set.insert(collider),
                                                );
                                                start_index = i;
                                            }
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
                            surface: ctx.graphics.create_rgba8_surface((
                                (layer.c_wid * layer.grid_size) as u32,
                                (layer.c_hei * layer.grid_size) as u32,
                            )),
                            rendered: false,
                        });
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
                                    .translation(pixel_scale_to_physics_glam(
                                        rapier2d::math::Vec2::new(
                                            pos.x,
                                            pos.y - TEST_ENEMY_HEIGHT - 2.,
                                        ),
                                    ));
                                let character_body =
                                    physics_data.rigid_body_set.insert(character_body);

                                let character_collider = ColliderBuilder::ball(
                                    pixel_scale_to_physics_f(TEST_ENEMY_WIDTH / 2.0),
                                )
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

                                let ground_sensor_collider =
                                    ColliderBuilder::new(SharedShape::ball(0.1))
                                        .translation(rapier2d::math::Vec2::new(
                                            0.0,
                                            -pixel_scale_to_physics_f(TEST_ENEMY_WIDTH / 2.0),
                                        ))
                                        .density(0.0)
                                        .friction(0.0)
                                        .sensor(true)
                                        .collision_groups(InteractionGroups::new(
                                            ENEMY_GROUP,
                                            TERRAIN_GROUP | PLAYER_GROUP,
                                            rapier2d::prelude::InteractionTestMode::Or,
                                        ))
                                        .build();

                                let ground_sensor_collider =
                                    physics_data.collider_set.insert_with_parent(
                                        ground_sensor_collider,
                                        character_body,
                                        &mut physics_data.rigid_body_set,
                                    );
                                let entity = world.spawn((
                                    CharacterControllerComponent::new(
                                        character_body,
                                        character_collider,
                                        false,
                                        ground_sensor_collider,
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
        }
        let player_start = if let Some(player_start) = player_start {
            player_start
        } else {
            Vec2F::ZERO
        };

        let src = VirtualSource::last_active(ctx);
        let spawn_btn = VirtualButton::new(&src, Key::Period, None);
        let debug_btn = VirtualButton::new(&src, Key::Comma, None);
        let step_button = VirtualButton::new(&src, Key::P, None);
        let pause_btn = VirtualButton::new(&src, Key::L, None);

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

        let character_body = RigidBodyBuilder::dynamic()
            .translation(pixel_scale_to_physics_glam(rapier2d::math::Vec2::new(
                player_start.x,
                player_start.y - PLAYER_HEIGHT - 2.,
            )))
            .gravity_scale(10.0)
            .soft_ccd_prediction(10.0);
        let character_body = physics_data.rigid_body_set.insert(character_body);

        let character_collider = ColliderBuilder::capsule_y(
            pixel_scale_to_physics_f(PLAYER_HEIGHT / 2.),
            pixel_scale_to_physics_f(PLAYER_WIDTH / 2.),
        )
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
        let ground_sensor_collider = ColliderBuilder::new(SharedShape::ball(
            pixel_scale_to_physics_f(PLAYER_WIDTH / 2.),
        ))
        .translation(rapier2d::math::Vec2::new(
            0.0,
            pixel_scale_to_physics_f(PLAYER_HEIGHT / 2.) + 0.01,
        ))
        .density(0.0)
        .friction(0.0)
        .sensor(true)
        .collision_groups(InteractionGroups::new(
            PLAYER_GROUP,
            TERRAIN_GROUP | ENEMY_GROUP,
            rapier2d::prelude::InteractionTestMode::Or,
        ))
        .active_events(ActiveEvents::COLLISION_EVENTS)
        .build();

        let ground_sensor_collider = physics_data.collider_set.insert_with_parent(
            ground_sensor_collider,
            character_body,
            &mut physics_data.rigid_body_set,
        );
        let player_entity = world.spawn((
            PlayerComponent::new(),
            CharacterControllerComponent::new(
                character_body,
                character_collider,
                false,
                ground_sensor_collider,
            ),
            PlayerAnimComponent::new(cfg.player_walk_anim_speed),
        ));
        physics_data
            .collider_set
            .get_mut(character_collider)
            .unwrap()
            .user_data = player_entity.id() as u128;

        let zoom_amount = 1.;

        let scale = zoom_amount * 2.0 * ctx.window.scale_factor();
        let window_size = to_logical_f(ctx.window.size().to_f32(), scale);

        let (sound_scene_handle, scene) = oddio::SpatialScene::new();

        /*         let host = cpal::default_host();

        let device = host
            .default_output_device()
            .expect("no output device available!");
        let sample_rate = device.default_output_config().unwrap().sample_rate();
        let stream_config = cpal::StreamConfig {
            channels: device.default_output_config().unwrap().channels(),
            sample_rate,
            buffer_size: cpal::BufferSize::Default,
        };

        // We send `scene` into this closure, where changes to `scene_handle` are reflected.
        // `scene_handle` is how we add new sounds and modify the scene live.
        let stream = device
            .build_output_stream(
                &stream_config,
                move |data: &mut [f32], _: &cpal::OutputCallbackInfo| {
                    let frames = oddio::frame_stereo(data);
                    oddio::run(&mut scene, sample_rate, frames);
                },
                move |err| {
                    eprintln!("{}", err);
                },
                None,
            )
            .unwrap();
        stream.play().unwrap(); */

        let (stream, sample_rate, host, device) = setup_output_stream(scene);

        let audio = AudioData {
            sound_scene_handle,
            stream,
            sample_rate,
            host,
            device,
        };

        let mut sounds = cfg
            .sound_cues
            .iter()
            .map(|sound_cue| sound_cue.sounds.clone())
            .flatten()
            .collect::<Vec<_>>();
        sounds.sort();
        sounds.dedup();

        let (sounds, sound_metadata, map) = load_audio(sounds)?;

        let sound_cues =
            HashMap::<String, SoundCue>::from_iter(cfg.sound_cues.iter().map(|sound_cue| {
                (
                    sound_cue.id.clone(),
                    SoundCue {
                        sounds: sound_cue
                            .sounds
                            .iter()
                            .map(|sound| *map.get(sound).unwrap())
                            .collect::<Vec<_>>(),
                    },
                )
            }));
        /*         let sound_cues = cfg
        .sound_cues
        .iter()
        .map(|sound_cue| SoundCue {
            sounds: sound_cue
                .sounds
                .iter()
                .map(|sound| *map.get(sound).unwrap())
                .collect::<Vec<_>>(),
        })
        .collect::<Vec<_>>(); */

        let rand = kero::rand::Rand::new();

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
                zoom_amount,
            ),
            player_weapon_flash_texture,
            player_projectile_texture,
            enemies_sheet_1_texture,
            render_debug_collision: false,
            debug_btn,
            pause: false,
            step_button,
            pause_btn,
            event_handler,
            collision_recv,
            contact_force_recv,
            cfg,
            audio,
            audio_device_check_timer: 0.0,
            audio_device_check_time: 1.0,
            sounds,
            sound_metadata,
            sound_cues,
            rand,
        })
    }

    fn update(&mut self, ctx: &Context) -> Result<(), GameError> {
        self.audio_device_check_timer += ctx.dt();
        if self.audio_device_check_timer >= self.audio_device_check_time {
            if self.audio.host.default_output_device().unwrap().id() != self.audio.device.id() {
                let (sound_scene_handle, scene) = oddio::SpatialScene::new();
                let (stream, sample_rate, host, device) = setup_output_stream(scene);

                self.audio = AudioData {
                    sound_scene_handle,
                    stream,
                    sample_rate,
                    host,
                    device,
                };
            }
            self.audio_device_check_timer = 0.0;
        }

        // perform your game logic here
        if self.debug_btn.pressed() {
            self.render_debug_collision = !self.render_debug_collision;
        }
        if self.pause_btn.pressed() {
            self.pause = !self.pause;
        }
        if self.pause {
            if self.step_button.pressed() {
            } else {
                return Ok(());
            }
        }

        //update inputs
        if let Ok(player) = self
            .world
            .query_one_mut::<&mut PlayerComponent>(self.player_entity)
        {
            let mut x = 0.0;
            if self.right_button.down() {
                x += 1.0;
            }
            if self.left_button.down() {
                x -= 1.0;
            }
            player.desired_velocity.x = x;

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

        let dt = ctx.dt();
        self.physics_data.integration_parameters.dt = dt;

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
                .translation(pixel_scale_to_physics_glam(rapier2d::math::Vector2::new(
                    self.player_start.x,
                    self.player_start.y - 10.,
                )))
                .build();
            let collider = ColliderBuilder::ball(pixel_scale_to_physics_f(5.))
                .restitution(0.7)
                .user_data(u128::MAX)
                .build();
            let ball_body_handle = self.physics_data.rigid_body_set.insert(rigid_body);
            self.physics_data.collider_set.insert_with_parent(
                collider,
                ball_body_handle,
                &mut self.physics_data.rigid_body_set,
            );

            self.balls.push(ball_body_handle);
        }

        let mut projectiles_to_spawn = Vec::new();
        let mut audio_emitters_to_spawn = Vec::new();
        let mut parented_audio_emitters_to_spawn = Vec::new();

        if let Ok((player_entity, player, character)) =
            self.world
                .query_one_mut::<(Entity, &mut PlayerComponent, &CharacterControllerComponent)>(
                    self.player_entity,
                )
        {
            if player.is_shooting && player.time_since_shot_start >= player.time_per_shot {
                player.is_shooting = false;
            } else if player.is_shooting {
                player.time_since_shot_start += dt;
            }

            if player.time_since_last_shot >= player.shot_delay_time
                && player.wants_to_shoot
                && !player.is_shooting
            {
                player.time_since_last_shot = 0.;
                player.time_since_shot_start = 0.;
                player.is_shooting = true;

                //spawn projectile
                let body_translation =
                    self.physics_data.rigid_body_set[character.character_body].translation();
                let body_translation = Vec2F::new(body_translation.x, body_translation.y);

                let bullet_spawn_dist = 0.1;
                let bullet_speed = 0.1;
                let bullet_pos = body_translation + (player.aim_dir * bullet_spawn_dist);
                let bullet_vel = player.aim_dir * bullet_speed;

                projectiles_to_spawn.push((
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
                /*                 self.world.spawn((
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
                )); */
                player.shoot_2_index = self.rand.range(0..self.sound_cues["shoot2"].sounds.len());
                //player.shoot_2_index =   inc_wrap_index(player.shoot_2_index, self.sound_cues["shoot2"].sounds.len());

                spawn_sound(
                    &mut audio_emitters_to_spawn,
                    &mut parented_audio_emitters_to_spawn,
                    pixel_scale_to_physics_glam(glamx::Vec2::new(
                        self.camera.position.x,
                        self.camera.position.y,
                    )),
                    glamx::Vec2::new(bullet_pos.x, bullet_pos.y),
                    25.0,
                    100.0,
                    &mut self.audio,
                    self.sounds[self.sound_cues["shoot2"].sounds[player.shoot_2_index]].clone(),
                    None,
                );
            } else if !player.is_shooting {
                player.time_since_last_shot += dt;
            }
        }

        self.world.spawn_batch(projectiles_to_spawn);
        self.world.spawn_batch(audio_emitters_to_spawn);
        self.world.spawn_batch(parented_audio_emitters_to_spawn);

        let _jump_height = 200.;
        let walk_speed = 10.;

        for (player, character_controller) in self
            .world
            .query_mut::<(&mut PlayerComponent, &mut CharacterControllerComponent)>()
        {
            if player.wants_to_jump {
                if character_controller.is_grounded {
                    player.wants_to_jump = false;
                    if character_controller.is_kinematic {
                        player.desired_velocity.y = -self.physics_data.gravity.y * 1.1;
                    } else {
                        self.physics_data.rigid_body_set[character_controller.character_body]
                            .apply_impulse(rapier2d::math::Vec2::new(0., -2.5), true);
                    }
                    /*  player.desired_velocity.y = -self.physics_data.gravity.y
                     * (2.0 * jump_height / self.physics_data.gravity.y).sqrt();  */
                }
            }
            if character_controller.is_kinematic {
                player.desired_velocity.y -= -self.physics_data.gravity.y * dt;
            }

            let mut desired_velocity = player.desired_velocity;

            if character_controller.is_kinematic {
                desired_velocity.x *= walk_speed * dt;
            } else {
                desired_velocity.x *= 0.05;
            }

            character_controller.desired_movement =
                rapier2d::math::Vec2::new(desired_velocity.x, desired_velocity.y);
        }

        step(&mut self.physics_data, &self.event_handler);

        while let Ok(collision_event) = self.collision_recv.try_recv() {
            // Handle the collision event.
            //println!("Received collision event: {:?}", collision_event);
            for character_controller in self.world.query_mut::<&mut CharacterControllerComponent>()
            {
                if !character_controller.is_kinematic {
                    let collider_handle = if collision_event.collider1()
                        == character_controller.ground_sensor_collider
                    {
                        Some(collision_event.collider1())
                    } else if collision_event.collider2()
                        == character_controller.ground_sensor_collider
                    {
                        Some(collision_event.collider2())
                    } else {
                        None
                    };

                    if let Some(_) = collider_handle {
                        if collision_event.started() {
                            character_controller.count_ground_sensor_contacts += 1;
                        }
                        if collision_event.stopped() {
                            character_controller.count_ground_sensor_contacts -= 1;
                        }
                    }
                }
            }
        }

        while let Ok(_contact_force_event) = self.contact_force_recv.try_recv() {
            // Handle the contact force event.
            //println!("Received contact force event: {:?}", contact_force_event);
        }

        for character_controller in self.world.query_mut::<&mut CharacterControllerComponent>() {
            character_controller.update_character_controller(&mut self.physics_data);
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
                if hit_collider.user_data != u128::MAX {
                    projectile_impacts.push(ProjectileImpact {
                        hit_entity_id: hit_collider.user_data as u32,
                        _proj_entity: entity,
                    });
                    hit = true;
                }
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

        for (player, character_controller) in self
            .world
            .query_mut::<(&mut PlayerComponent, &mut CharacterControllerComponent)>()
        {
            let body = &self.physics_data.rigid_body_set[character_controller.character_body];
            let translation = body.position().translation;
            let translation = physics_scale_to_pixels_glam(translation);

            self.camera.position.x = translation.x + 24.0 / 2.0 - self.camera.width / 2.;
            self.camera.position.y = translation.y + 32.0 / 2.0 - self.camera.height / 2.;

            if self.use_controller {
                let diff = Vec2F::new(self.right_stick.x(), self.right_stick.y());
                let mut angle = f32::atan2(diff.y, diff.x);
                if angle < 0. {
                    angle += f32::PI * 2.;
                }
                player.aim_angle = angle * (180. / f32::PI);
                player.aim_dir = diff.norm();
            } else {
                let body_pos = Vec2F::new(translation.x, translation.y);
                let diff = mouse_world_pos - body_pos;
                let mut angle = f32::atan2(diff.y, diff.x);
                if angle < 0. {
                    angle += f32::PI * 2.;
                }
                player.aim_angle = angle * (180. / f32::PI);
                player.aim_dir = diff.norm();
            }
        }

        update_audio_emitters(&self.camera, &mut self.world, &mut to_despawn);

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
            }

            self.world.despawn(entity).unwrap();
        }

        Ok(())
    }

    fn render(&mut self, ctx: &Context, draw: &mut Draw) -> Result<(), GameError> {
        draw.set_main_sampler(Sampler::new(
            AddressMode::Clamp,
            AddressMode::Clamp,
            FilterMode::Linear,
            FilterMode::Nearest,
        ));
        for grid in &mut self.grids {
            if !grid.rendered {
                draw.set_surface(grid.surface.clone(), Rgba8::TRANSPARENT);
                let tile_set = self.tilesets.get(&grid.tile_set_id).unwrap();
                let mut white = Rgba8::WHITE;
                white.a = (grid.opacity * 255.0) as u8;
                for tile in &grid.tiles {
                    let pos = Vec2F::new(tile.px[0] as f32, tile.px[1] as f32);
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

                    draw.subtexture_at_flipped(sub, pos, white, ColorMode::MULT, flip);
                }

                grid.rendered = true;
            }
        }

        // draw a background color to the window
        draw.set_surface(None, rgb(0x476c6c));

        let scale = self.camera.zoom_amount * 2.0 * ctx.window.scale_factor();

        // let's scale everything up
        draw.push_scale_of(scale);

        for grid in &self.grids {
            draw.texture_at(&grid.surface, grid.pos - self.camera.position);
        }

        for ball in &self.balls {
            let ball_body = &self.physics_data.rigid_body_set[*ball];
            let translation = physics_scale_to_pixels_glam(ball_body.translation());

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

            let pixel_translation = physics_scale_to_pixels(projectile.position);

            draw.subtexture_at_flipped(
                sub,
                Vec2F::new(pixel_translation.x - 18., pixel_translation.y - 18.)
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
            let translation = physics_scale_to_pixels_glam(body.translation());
            let translation = Vec2F::new(translation.x, translation.y);

            let mut animation_x = 0;

            player_anim.is_muzzle_flashing = player.is_shooting;

            if character_controller.is_grounded {
                player_anim.frames_ungrounded = 0;
            } else {
                player_anim.frames_ungrounded += 1;
            }
            if !character_controller.is_grounded && player_anim.frames_ungrounded > 3 {
                player_anim.is_jumping = true;
            } else {
                player_anim.is_jumping = false;
            }
            if self.down_button.pressed() {
                player_anim.is_crouching = true;
            } else if self.down_button.released() {
                player_anim.is_crouching = false;
            }

            let is_running = player.desired_velocity.x.abs() > 0.;

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

            if player.desired_velocity.x > 0. {
                player_anim.flip_x = false;
            } else if player.desired_velocity.x < 0. {
                player_anim.flip_x = true;
            }

            let aim_angle = if player_anim.flip_x {
                let x = 180.0 - player.aim_angle;
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

            player_anim.prev_anim_x = animation_x as u32 * 46;
            player_anim.prev_anim_y = animation_y;
        }

        for (_, character_controller) in self
            .world
            .query_mut::<(&EnemyComponent, &CharacterControllerComponent)>()
        {
            let body = &self.physics_data.rigid_body_set[character_controller.character_body];
            let translation = body.translation();
            let translation = physics_scale_to_pixels_glam(translation);

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
                draw_physics_shape(
                    draw,
                    &self.physics_data,
                    character_controller.ground_sensor_collider,
                    self.camera.position,
                    Rgba8::BLUE,
                );
            }

            for tile_grid in &self.grids {
                for collider in &tile_grid.colliders {
                    draw_physics_shape(
                        draw,
                        &self.physics_data,
                        *collider,
                        self.camera.position,
                        Rgba8::RED,
                    );
                }
            }
        }

        draw.circle(
            circle(to_logical_f(ctx.mouse.pos(), scale), 5.),
            Rgba8::RED,
            None,
        );

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
    let translation = physics_scale_to_pixels_glam(collider.translation());
    match collider.shape().shape_type() {
        rapier2d::prelude::ShapeType::Ball => {
            let ball = collider.shape().as_ball().unwrap();
            let translation = Vec2F::new(translation.x, translation.y) - camera_pos;
            draw.circle_outline(
                circle(translation, physics_scale_to_pixels_f(ball.radius)),
                color,
                None,
            );
        }
        rapier2d::prelude::ShapeType::Cuboid => {
            let cuboid = collider.shape().as_cuboid().unwrap();
            let half_extents = physics_scale_to_pixels_glam(cuboid.half_extents);

            let rect_center = Vec2F::new(
                translation.x - half_extents.x,
                translation.y - half_extents.y,
            ) - camera_pos;
            draw.rect_outline(
                rect(
                    rect_center.x,
                    rect_center.y,
                    half_extents.x * 2.0,
                    half_extents.y * 2.0,
                ),
                color,
            );
        }
        rapier2d::prelude::ShapeType::Capsule => {
            let capsule = collider.shape().as_capsule().unwrap();
            let aabb = capsule.local_aabb();
            let half = physics_scale_to_pixels_glam(aabb.half_extents());
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
            let half_extents = physics_scale_to_pixels_glam(cuboid.half_extents);

            let rect_center = Vec2F::new(
                translation.x - half_extents.x,
                translation.y - half_extents.y,
            ) - camera_pos;
            draw.rect_outline(
                rect(
                    rect_center.x,
                    rect_center.y,
                    half_extents.x * 2.0,
                    half_extents.y * 2.0,
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

#[derive(Debug, Deserialize, Serialize)]
pub struct GameConfig {
    pub window_size: (u32, u32),
    pub window_title: String,
    pub default_level: String,
    pub player_walk_anim_speed: f32,
    pub merge_tiles: bool,
    pub sound_cues: Vec<SoundCueConfig>,
}
