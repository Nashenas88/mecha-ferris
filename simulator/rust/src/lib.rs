mod game;
// mod multi_leg;
mod servo;

use gdnative::prelude::{godot_init, InitHandle};

// Function that registers all exposed classes to Godot
fn init(handle: InitHandle) {
    handle.add_class::<game::Game>();
    handle.add_class::<servo::Servo>();
}

// macros that create the entry-points of the dynamic library.
godot_init!(init);
