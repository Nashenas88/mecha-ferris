mod game;
mod servo;
mod walker;

use gdnative::prelude::{godot_init, InitHandle};

// Function that registers all exposed classes to Godot
fn init(handle: InitHandle) {
    handle.add_class::<game::Game>();
    handle.add_class::<servo::Servo>();
    handle.add_class::<walker::Walker>();
}

// macros that create the entry-points of the dynamic library.
godot_init!(init);
