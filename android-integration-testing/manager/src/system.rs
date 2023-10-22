use std::env;

pub fn is_release() -> bool {
    Ok("release".to_owned()) == env::var("PROFILE")
}

pub fn rerun_if_changed(file_name: &str) {
    println!("cargo:rerun-if-changed={}", file_name);
}
