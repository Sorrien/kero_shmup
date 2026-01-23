use std::{
    fs::File,
    io::{BufReader, Read},
};

use kero::prelude::*;
use kero_shmup::{GameConfig, Shmup};

fn main() -> Result<(), GameError> {
    env_logger::init();
    let config_file = File::open("game_config.ron").unwrap();
    let mut config_file = BufReader::new(config_file);
    let mut config_string = String::new();
    config_file.read_to_string(&mut config_string).unwrap();
    let config = ron::from_str::<GameConfig>(&config_string).unwrap();
    // create a game, set some options, and then run it
    kero::new_game()
        .with_title(&config.window_title)
        .with_size(config.window_size.0, config.window_size.1)
        .run::<Shmup>(config)
}
