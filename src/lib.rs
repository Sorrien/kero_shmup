use std::{collections::HashMap, fs::File, io::Read};

use crate::{
    ldtk::{TileGrid, TileType, Tileset},
    ldtk_json::LdtkJson,
    physics::{CharacterControllerComponent, PhysicsData, step},
    player::PlayerComponent,
};
use hecs::World;
use kero::{grid::Grid, prelude::*};
use rapier2d::prelude::{ColliderBuilder, RigidBodyBuilder, RigidBodyHandle};

pub mod ldtk;
pub mod ldtk_json;
pub mod physics;
pub mod player;

pub struct Camera {
    pub position: Vec2F,
    pub width: u32,
    pub height: u32,
    pub zoom_amount: f32,
}

impl Camera {
    pub fn new(position: Vec2F, width: u32, height: u32, zoom_amount: f32) -> Self {
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
    flip_x: bool,
    run_frame_time: f32,
    run_frame_time_acc: f32,
    aim_angle_index: u32,
}

impl PlayerAnimComponent {
    pub fn new() -> Self {
        Self {
            run_index: 0,
            is_crouching: false,
            flip_x: false,
            run_frame_time: 0.1,
            run_frame_time_acc: 0.0,
            aim_angle_index: 0,
        }
    }
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
    aim_right_button: VirtualButton,
    _aim_left_button: VirtualButton,
    jump_button: VirtualButton,
    camera: Camera,
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
                                                translation.x,
                                                translation.y,
                                            ));
                                            /*                                             voxels.push(rapier2d::math::Vector2::new(
                                                translation.x as f32,
                                                translation.y as f32,
                                            )); */

                                            physics_data.collider_set.insert(collider);
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
                        if entity_instance.identifier == "Player" {
                            player_start = Some(
                                grid_pos
                                    + Vec2F::new(
                                        entity_instance.px[0] as f32,
                                        entity_instance.px[1] as f32,
                                    ),
                            );
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

        let left_button = VirtualButton::new(&src, Key::A, GamepadButton::DPadLeft);
        let right_button = VirtualButton::new(&src, Key::D, GamepadButton::DPadRight);
        let down_button = VirtualButton::new(&src, Key::S, GamepadButton::DPadDown);

        let aim_left_button = VirtualButton::new(&src, Key::ArrowLeft, None);
        let aim_right_button = VirtualButton::new(&src, Key::ArrowRight, None);

        let jump_button = VirtualButton::new(&src, Key::Space, GamepadButton::South);

        let character_height = 15.0;

        let character_body = RigidBodyBuilder::kinematic_position_based().translation(
            rapier2d::math::Vec2::new(player_start.x, player_start.y - character_height - 2.),
        );
        let character_body = physics_data.rigid_body_set.insert(character_body);

        let character_collider = ColliderBuilder::capsule_y(character_height / 2., 1.0);
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

        let player_texture = ctx
            .graphics
            .load_png_from_file("assets/char-sheet-alpha.png", true)
            .unwrap();

        let window_size = ctx.window.size();
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
            _aim_left_button: aim_left_button,
            aim_right_button,
            jump_button,
            player_entity,
            player_texture,
            camera: Camera::new(
                vec2(player_start.x, player_start.y),
                window_size.x,
                window_size.y,
                1.,
            ),
        })
    }

    fn update(&mut self, ctx: &Context) -> Result<(), GameError> {
        let window_size = ctx.window.size();
        self.camera.width = window_size.x;
        self.camera.height = window_size.y;
        // perform your game logic here

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

        if let Ok(player) = self
            .world
            .query_one_mut::<&mut PlayerComponent>(self.player_entity)
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
        }

        let jump_height = 60.;
        let walk_speed = 2.0;

        for (player, character_controller) in self
            .world
            .query_mut::<(&mut PlayerComponent, &mut CharacterControllerComponent)>()
        {
            if self.jump_button.pressed() {
                //println!("jump pressed");

                if character_controller.is_grounded {
                    //println!("grounded");
                    player.desired_velocity.y = -self.physics_data.gravity.y
                        * (2.0 * jump_height / self.physics_data.gravity.y).sqrt();
                }
            } else {
                player.desired_velocity.y = 0.;
            }

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
            let scale = self.camera.zoom_amount * 2.0 * ctx.window.scale_factor();
            self.camera.position.x =
                translation.x + 24.0 / 2.0 - self.camera.width as f32 / scale / 2.;
            self.camera.position.y =
                translation.y + 32.0 / 2.0 - self.camera.height as f32 / scale / 2.; // + self.camera.height as f32;
        }

        step(&mut self.physics_data);

        for character_controller in self.world.query_mut::<&mut CharacterControllerComponent>() {
            character_controller.update_character_controller(ctx.dt(), &mut self.physics_data);
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

        for (player_anim, character_controller) in self
            .world
            .query_mut::<(&mut PlayerAnimComponent, &CharacterControllerComponent)>()
        {
            let body = &self.physics_data.rigid_body_set[character_controller.character_body];
            let translation = body.translation();
            /*             draw.circle(
                circle(Vec2F::new(translation.x, translation.y), 5.),
                Rgba8::RED,
                None,
            ); */
            let lin_vel = body.linvel();

            let mut animation_x = 0;

            if self.down_button.pressed() {
                player_anim.is_crouching = true;
            } else if self.down_button.released() {
                player_anim.is_crouching = false;
            }

            if self.aim_right_button.pressed() {
                player_anim.aim_angle_index =
                    inc_wrap_index(player_anim.aim_angle_index as usize, 48) as u32;
            }

            if lin_vel.x.abs() > 0. {
                player_anim.run_frame_time_acc += ctx.dt();
                if player_anim.run_frame_time_acc > player_anim.run_frame_time {
                    player_anim.run_index = inc_wrap_index(player_anim.run_index, 4);
                    player_anim.run_frame_time_acc = 0.;
                }
                animation_x = player_anim.run_index + 2;
            }
            if lin_vel.x > 0. {
                //player_anim.run_index = inc_wrap_index(player_anim.run_index, 4);
                //animation_x = player_anim.run_index + 2;
                player_anim.flip_x = false;
            } else if lin_vel.x < 0. {
                //player_anim.run_index = inc_wrap_index(player_anim.run_index, 4);
                //player_anim.flip_x = true;
                //animation_x = player_anim.run_index + 2;
                player_anim.flip_x = true;
            } else {
                player_anim.run_index = 0;

                //not running
                if player_anim.is_crouching {
                    animation_x = 1;
                }
            }

            let animation_y = player_anim.aim_angle_index * 46;

            let sub = self
                .player_texture
                .sub(rect(animation_x as u32 * 46, animation_y, 46, 46));

            draw.subtexture_at_flipped(
                sub,
                Vec2F::new(translation.x - 46., translation.y - 46.) - self.camera.position,
                Rgba8::WHITE,
                ColorMode::MULT,
                Vec2::new(player_anim.flip_x, false),
            );
        }

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

pub fn inc_wrap_index(index: usize, len: usize) -> usize {
    (index + 1) % len
}

pub struct MinMax {
    pub min: i64,
    pub max: i64,
    pub size: i64,
}
