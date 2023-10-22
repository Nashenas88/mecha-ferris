use manager::{add_android_targets_to_toolchain, create_android_targets_config_file, system};

fn main() {
    system::rerun_if_changed("build.rs");

    create_android_targets_config_file();
    add_android_targets_to_toolchain();
}
