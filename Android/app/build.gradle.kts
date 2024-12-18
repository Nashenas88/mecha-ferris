import com.google.protobuf.gradle.id

plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
    id("com.google.protobuf")
    id("com.google.devtools.ksp")
    id("kotlin-parcelize")
    id("idea")
}

val grpcVersion = "1.45.1"

android {
    namespace = "paulfaria.mechaferris"
    compileSdk = 34

    defaultConfig {
        applicationId = "paulfaria.mechaferris"
        minSdk = 33
        targetSdk = 33
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
        vectorDrawables.useSupportLibrary = true
    }

    buildTypes {
        getByName("release") {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }
    kotlinOptions {
        jvmTarget = "1.8"
    }
    buildFeatures {
        compose = true
        viewBinding = true
    }

    composeOptions {
        kotlinCompilerExtensionVersion = "1.5.4"
    }
    packaging {
        resources {
            excludes += "/META-INF/{AL2.0,LGPL2.1}"
        }
    }
}

protobuf {
    protoc {
        // The artifact spec for the Protobuf Compiler
        artifact = "com.google.protobuf:protoc:3.20.1"
    }
    generateProtoTasks {
        all().forEach { task ->
            task.builtins {
                id("java") {
                    option("lite")
                }
                id("kotlin") {
                    option("lite")
                }
            }
        }
    }
}

dependencies {
    implementation("androidx.core:core-ktx:1.12.0")
    val lifecycleVersion = "2.6.2"
    val navigationVersion = "2.7.5"
    val composeVersion = "1.5.4"
    val composeBom = platform("androidx.compose:compose-bom:2023.10.00")
    val kableVersion = "0.28.0-rc"
    val exerciseVersion = "0.11.1"
    implementation(composeBom)
    androidTestImplementation(composeBom)

    // Nordic Semiconductor BLE Library
//    implementation("no.nordicsemi.android:ble-ktx:2.6.1")
    // Ble integration with LiveData
//    implementation("no.nordicsemi.android:ble-livedata:2.6.1")

    implementation("androidx.lifecycle:lifecycle-runtime-ktx:$lifecycleVersion")
    // Lifecycle utilities for Compose
    implementation("androidx.lifecycle:lifecycle-runtime-compose:$lifecycleVersion")

    implementation(platform("androidx.compose:compose-bom:2022.10.00"))
    implementation("androidx.compose.ui:ui")
    implementation("androidx.compose.ui:ui-graphics")
    androidTestImplementation(platform("androidx.compose:compose-bom:2022.10.00"))

    // Coroutines
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-android:1.7.3")

    // Kable
    implementation("com.juul.kable:core-android:$kableVersion")
    // Exercise
    implementation("com.juul.exercise:annotations:$exerciseVersion")
    add("ksp", "com.juul.exercise:compile:$exerciseVersion")

    // Material Design 3
    implementation("androidx.compose.material3:material3:1.1.2")

    // Android Studio Preview support
    implementation("androidx.compose.ui:ui-tooling-preview:$composeVersion")
    debugImplementation("androidx.compose.ui:ui-tooling:$composeVersion")

    // UI Tests
    androidTestImplementation("androidx.compose.ui:ui-test-junit4:$composeVersion")
    debugImplementation("androidx.compose.ui:ui-test-manifest:$composeVersion")

    // Integration with activities
    implementation("androidx.activity:activity-compose:1.8.1")
    // Integration with ViewModels
    implementation("androidx.lifecycle:lifecycle-viewmodel-compose:$lifecycleVersion")
    // Integration with LiveData
    implementation("androidx.compose.runtime:runtime-livedata:$composeVersion")
//    // Optional - Integration with RxJava
//    implementation "androidx.compose.runtime:runtime-rxjava2"

    // Permissions
    implementation("com.google.accompanist:accompanist-permissions:0.32.0")

    // DataStore
    implementation("androidx.datastore:datastore:1.0.0")

    // Protobuf for DataStore
    val protobufVersion = "3.21.2"
    implementation("com.google.protobuf:protobuf-javalite:$protobufVersion")
    implementation("com.google.protobuf:protobuf-kotlin-lite:$protobufVersion")
    implementation("io.grpc:grpc-android:$grpcVersion")
    implementation("io.grpc:grpc-stub:$grpcVersion")
    implementation("io.grpc:grpc-protobuf-lite:$grpcVersion")

    implementation("androidx.core:core-ktx:1.12.0")
    implementation("androidx.appcompat:appcompat:1.6.1")
    implementation("com.google.android.material:material:1.10.0")
    implementation("androidx.constraintlayout:constraintlayout:2.1.4")
    implementation("androidx.lifecycle:lifecycle-livedata-ktx:$lifecycleVersion")
    implementation("androidx.lifecycle:lifecycle-viewmodel-ktx:$lifecycleVersion")
    implementation("androidx.navigation:navigation-fragment-ktx:$navigationVersion")
    implementation("androidx.navigation:navigation-ui-ktx:$navigationVersion")
    implementation("androidx.legacy:legacy-support-v4:1.0.0")
    testImplementation("junit:junit:4.13.2")
    androidTestImplementation("androidx.test.ext:junit:1.1.5")
    androidTestImplementation("androidx.test.espresso:espresso-core:3.5.1")
}

idea {
    module {
        sourceDirs.add(file("src/main/proto"))
        generatedSourceDirs.add(file("build/generated/source/proto/main"))
    }
}
