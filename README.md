# Mecha Ferris

This is a project to build a robotic version of the friendly [Ferris] on top of a hexapod platform.
This is a work in progress and may not be fully functional.

[Ferris]: https://en.wikipedia.org/wiki/List_of_computing_mascots#F

## Directories

* free-cad: The FreeCAD files that the 3D printable parts are created from.
* [kinematics]: The forward and inverse kinematics code for the robot.
* [mecha-ferris]: The binary that's designed to run on the [pimorono-servo-2040 board]. It uses the kinematics crate to drive the robot motion.
* rust-avr: This was an older binary attempt to run the robot off of an arduino. Given the still-active bugs that prevent division from working, this is a dead folder and will be removed in a future commit.
* [simulator]: This is a simulation of the robot build into a godot project. It relies on godot-rust bindings so the same `kinematics` crate used in the `mecha-ferris` binary can also drive the simulation.

[kinematics]: ./kinematics/README.md
[mecha-ferris]: ./mecha-ferris/README.md
[simulator]: ./simulator/README.md

## License

The contents of this repository are dual-licensed under the _MIT OR Apache
2.0_ License. That means you can chose either the MIT licence or the
Apache-2.0 licence when you re-use this code. See `MIT` or `APACHE2.0` for more
information on each specific licence.

Any submissions to this project (e.g. as Pull Requests) must be made available
under these terms.
