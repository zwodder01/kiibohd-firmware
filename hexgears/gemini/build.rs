// Copyright 2021 Jacob Alexander
// Copyright 2021 Zion Koyl
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
fn main() {
    // Setup memory map for linker
    if env::var_os("CARGO_FEATURE_RT").is_some() {
        let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
        File::create(out.join("memory.x"))
            .unwrap()
            .write_all(include_bytes!("memory.x"))
            .unwrap();
        println!("cargo:rustc-link-search={}", out.display());
        // TODO - Re-add once cmake is removed
        //println!("cargo:rerun-if-changed=memory.x");
    }
    //println!("cargo:rerun-if-changed=build.rs");

    let defmap_rslt: String;
    let partmap_rslt: String;
    match env::var_os("DefaultMapOverride") {
        Some(x) => defmap_rslt = x.into_string().unwrap(),
        None => defmap_rslt = String::from("geminiduskdawn/release.1 stdFuncMap"),
    }
    let defmap: &str = &defmap_rslt;

    match env::var_os("PartialMapsExpandedOverride") {
        Some(x) => partmap_rslt = x.into_string().unwrap(),
        None => partmap_rslt = String::from("geminiduskdawn/release.1.layer.1 stdFuncMap;geminiduskdawn/release.1.layer.2 stdFuncMap"),
    }
    let partmap: &str = &partmap_rslt;

    // Build controller static library
    // During the transition to rust, we'll need to link in the original kiibohd
    // controller codebase.
    let dst = cmake::Config::new("controller")
        .define("CHIP", "sam4s8b")
        .define("COMPILER", "gcc")
        .define("ScanModule", "Gemini_Dusk_Dawn")
        .define("MacroModule", "PixelMap")
        .define("OutputModule", "USB")
        .define("DebugModule", "full")
        .define("LayoutName", "")
        .define("BaseMap", "scancode_map")
        .define("DefaultMap", defmap)
        .define("PartialMaps", partmap)
        // TODO - Store VIDs in a config file
        .define("VENDOR_ID", 0x308F.to_string())
        .define("PRODUCT_ID", 0x0015.to_string())
        .define("BOOT_VENDOR_ID", 0x308F.to_string())
        .define("BOOT_PRODUCT_ID", 0x0014.to_string())
        .always_configure(false)
        .generator("Ninja")
        .build();
    println!("cargo:rustc-link-search=native={}/lib", dst.display());
    println!("cargo:rustc-link-lib=static=kiibohd_static");
}
