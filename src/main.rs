use kero::prelude::*;
use kero_shmup::Shmup;

fn main() -> Result<(), GameError> {
    env_logger::init();

    // create a game, set some options, and then run it
    kero::new_game()
        .with_title("Shmup")
        .with_size(1280, 720)
        .run::<Shmup>(())
}
