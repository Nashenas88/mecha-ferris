use std::io::{self, Write};
use std::process::Command;

/// Runs a command line command.
///
/// ## Example:
/// ```
/// use cryptor_global::console::run_command;
///
/// let mut command_args = Vec::new();
/// command_args.push("ls");
/// command_args.push("a");
///
/// run_command("ls", &command_args.as_slice());
/// ```
pub fn run_command(comand: &str, args: &[&str]) {
    let mut command_with_args = Command::new(comand);

    for arg in args.iter() {
        command_with_args.arg(arg);
    }

    let output = command_with_args
        .output()
        .expect("Failed to execute command");

    io::stdout().write_all(&output.stdout).unwrap();
    io::stderr().write_all(&output.stderr).unwrap();
}
