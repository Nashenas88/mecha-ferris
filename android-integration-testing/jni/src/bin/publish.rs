///
/// This is a binary targets, which is an executable program
/// that can be run after crate compilation.
///
/// It will basically copy the 'release' version
/// of this crate to the corresponding android
/// project.
///
/// ## Examples
/// ```
/// $ cd cryptor_jni/
/// $ cargo run --bin publish
/// ```
///
/// For more information, refer to the official doc:
///  - https://doc.rust-lang.org/cargo/reference/cargo-targets.html#binaries
///
use anyhow::{anyhow, Context, Result};
use std::env;
use std::path::PathBuf;
use std::path::MAIN_SEPARATOR_STR;

// Represents the crate/lib file name generated
static JNI_LIB_FILE_NAME: &str = "librust_kotlin.so";

///
/// Returns the project directory path.
///
/// ## Examples
///
/// `$ rust-library/`
///
fn project_dir_path() -> String {
    let current_dir_path = env::current_dir().expect("Cannot read current directory");
    let target_dir_path = current_dir_path
        .parent()
        .expect("Cannot find/read 'rust-library' directory");

    target_dir_path
        .to_str()
        .expect("Cannot validate 'rust-library' directory")
        .to_owned()
}

///
/// Returns the jni directory path in the android project
/// where the release version of this crate should be
/// placed.
///
/// ## Arguments
///
/// * `android_jni_lib_folder` - A string slice that holds the name of the android target.
///
/// ## Examples
///
/// `$ rust-library/app/src/main/jniLibs`
///
fn android_jni_dir_path(android_jni_lib_folder: &str) -> String {
    let project_dir = PathBuf::from(project_dir_path());
    let android_project_dir_path = project_dir
        .parent()
        .expect("Cannot find/read 'rust-library' directory");

    let mut android_jni_file_path = android_project_dir_path
        .to_str()
        .expect("Cannot validate 'rust-library' directory")
        .to_owned();

    android_jni_file_path.push_str(MAIN_SEPARATOR_STR);
    android_jni_file_path.push_str("Android");
    android_jni_file_path.push_str(MAIN_SEPARATOR_STR);
    android_jni_file_path.push_str("app");
    android_jni_file_path.push_str(MAIN_SEPARATOR_STR);
    android_jni_file_path.push_str("src");
    android_jni_file_path.push_str(MAIN_SEPARATOR_STR);
    android_jni_file_path.push_str("main");
    android_jni_file_path.push_str(MAIN_SEPARATOR_STR);
    android_jni_file_path.push_str("jniLibs");
    android_jni_file_path.push_str(MAIN_SEPARATOR_STR);
    android_jni_file_path.push_str(android_jni_lib_folder);
    android_jni_file_path.push_str(MAIN_SEPARATOR_STR);
    android_jni_file_path.push_str(JNI_LIB_FILE_NAME);

    android_jni_file_path
}

#[cfg(feature = "host")]
fn crate_file_path_for_host(project_dir_path: &str) -> String {
    let mut crate_lib_file_path = project_dir_path.to_owned();

    crate_lib_file_path.push_str(MAIN_SEPARATOR_STR);
    crate_lib_file_path.push_str("target");
    crate_lib_file_path.push_str(MAIN_SEPARATOR_STR);
    crate_lib_file_path.push_str("release");
    crate_lib_file_path.push_str(MAIN_SEPARATOR_STR);
    crate_lib_file_path.push_str(JNI_LIB_FILE_NAME);

    crate_lib_file_path
}

///
/// Returns the file path where the
/// release version of this crate
/// is placed.
///
/// ## Arguments
///
/// * `project_dir_path` - A string slice that holds thsi project directory path.
/// * `android_target` - A string slice that holds the name of the android target.
///
/// ## Examples
///
/// `$ rust-library/target/x86_64-linux-android/release/JNI_LIB_FILE_NAME`
///
fn crate_file_path_for_target(project_dir_path: &str, android_target: &str) -> String {
    let mut crate_lib_file_path = project_dir_path.to_owned();

    crate_lib_file_path.push_str(MAIN_SEPARATOR_STR);
    crate_lib_file_path.push_str("target");
    crate_lib_file_path.push_str(MAIN_SEPARATOR_STR);
    crate_lib_file_path.push_str(android_target);
    crate_lib_file_path.push_str(MAIN_SEPARATOR_STR);
    crate_lib_file_path.push_str("release");
    crate_lib_file_path.push_str(MAIN_SEPARATOR_STR);
    crate_lib_file_path.push_str(JNI_LIB_FILE_NAME);

    crate_lib_file_path
}

///
/// Copy the release version of each android target to the corresponding
/// directory in the android project.
///
fn publish_jni_lib_to_android_project() -> Result<String> {
    let project_dir_path = project_dir_path();

    println!("publishing");
    #[cfg(feature = "host")]
    {
        let crate_lib_file_path = crate_file_path_for_host(&project_dir_path);
        let android_jni_lib_folder = "host"; // x86_64?
        let android_lib_file_path = android_jni_dir_path(android_jni_lib_folder);
        if PathBuf::from(&crate_lib_file_path).exists() {
            if !PathBuf::from(&android_lib_file_path)
                .parent()
                .unwrap()
                .exists()
            {
                return Err(anyhow!("Missing parent dir for {android_lib_file_path:?}"));
            }
            std::fs::copy(&crate_lib_file_path, &android_lib_file_path)
                .context("copying host lib")?;
        } else {
            return Err(anyhow!("Error copying library file from {crate_lib_file_path:?} to {android_lib_file_path:?}"));
        }
    }

    // we loop through all android targets
    for android_target in manager::ANDROID_TARGET_ABI_CONFIG.keys() {
        // get the path of the 'JNI_LIB_FILE_NAME' file.
        let crate_lib_file_path = crate_file_path_for_target(&project_dir_path, android_target);

        // get the jni android folder name to place our 'JNI_LIB_FILE_NAME' file.
        let android_jni_lib_folder = manager::ANDROID_TARGET_ABI_CONFIG
            .get(android_target)
            .expect("Cannot find 'jniLib' folder in 'rust-library' project.")
            .2;

        // build the entire jniLib based on the current android target
        let android_lib_file_path = android_jni_dir_path(android_jni_lib_folder);

        if PathBuf::from(&crate_lib_file_path).exists() {
            if !PathBuf::from(&android_lib_file_path)
                .parent()
                .unwrap()
                .exists()
            {
                return Err(anyhow!("Missing parent dir for {android_lib_file_path:?}"));
            }
            std::fs::copy(&crate_lib_file_path, &android_lib_file_path).context("copying lib")?;
        } else {
            return Err(anyhow!("Error copying library file from {crate_lib_file_path:?} to {android_lib_file_path:?}"));
        }
    }

    Ok("JNI Libs Succesfully Published to the Android Project!!!".to_owned())
}

fn main() {
    match publish_jni_lib_to_android_project() {
        Ok(success_message) => println!("{success_message}"),
        Err(error) => eprintln!("{error:?}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn android_jni_dir_path_includes_proper_folder_names() {
        let jni_folder_name = "arm64-v8a";
        let jni_dir = android_jni_dir_path(jni_folder_name);

        assert!(jni_dir.contains("rust-library"));
        assert!(jni_dir.contains("app"));
        assert!(jni_dir.contains("src"));
        assert!(jni_dir.contains("main"));
        assert!(jni_dir.contains("jniLibs"));
        assert!(jni_dir.contains(jni_folder_name));
    }

    #[test]
    fn crate_file_path_for_target_includes_proper_folder_names() {
        let project_dir_path = "fernando";
        let android_target = "android";
        let crate_file_path = crate_file_path_for_target(project_dir_path, android_target);

        assert!(crate_file_path.contains(project_dir_path));
        assert!(crate_file_path.contains("target"));
        assert!(crate_file_path.contains(android_target));
        assert!(crate_file_path.contains("release"));
    }

    #[test]
    fn proper_project_dir_path() {
        let project_dir_path = &project_dir_path();

        assert!(project_dir_path.contains("rust-library"));
    }
}
