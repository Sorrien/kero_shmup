use std::collections::HashMap;

use kero::{gfx::SubTexture, grid::GridBuf, math::Vec2F};

use crate::ldtk_json::{TileInstance};

pub struct TileGrid {
    pub int_grid_csv: Vec<i64>,
    pub width: i64,
    pub height: i64,
    pub pos: Vec2F,
    pub tile_set_id: i64,
    pub tiles: Vec<TileInstance>,
    pub opacity: f64,
}
pub struct Tileset {
    pub tiles: GridBuf<SubTexture>,
    pub tile_types: HashMap<i64, TileType>,
    pub grid_size: i64,
}

#[repr(u8)]
pub enum TileType {
    None = 0,
    Collider = 1,
}

impl From<u8> for TileType {
    fn from(value: u8) -> Self {
        match value {
            0 => TileType::None,
            1 => TileType::Collider,
            _ => TileType::None,
        }
    }
}