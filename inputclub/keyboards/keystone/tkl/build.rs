// Copyright 2021 Jacob Alexander
// Copyright 2021 Zion Koyl
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

use std::env;
use std::fs::File;
use std::io::prelude::*;
use std::io::Write;
use std::path::PathBuf;
fn main() {
    // Setup memory map for linker
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!(
            "../../../../common/memory/atsam4s8b_nobootloader.x"
        ))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=project.env");

    // Read PID and HID from project.env
    let mut projectfile = match File::open("project.env") {
        Err(e) => panic!("Can't read from project.env, err {}", e),
        Ok(f) => f,
    };
    let mut projectread = String::new();
    match projectfile.read_to_string(&mut projectread) {
        Err(e) => panic!("File read failed with err {}", e),
        Ok(f) => f,
    };

    // Identify and store data from file into variables
    let mut bvid_rslt = "";
    let mut bpid_rslt = "";
    let mut vid_rslt = "";
    let mut pid_rslt = "";
    let mut usb_manufacturer = "XXXX";
    let mut usb_product = "YYYY";
    let mut usb_serial_chip = "ZZZZ";
    for line in projectread.lines() {
        let curline = &line;
        if curline.contains("BOOT_VID=\"") {
            let strt = curline
                .find("BOOT_VID=\"")
                .unwrap()
                .checked_add(10)
                .unwrap();
            let end = curline.rfind('\"').unwrap();
            bvid_rslt = &curline[strt..end];
        } else if curline.contains("BOOT_PID=\"") {
            let strt = curline
                .find("BOOT_PID=\"")
                .unwrap()
                .checked_add(10)
                .unwrap();
            let end = curline.rfind('\"').unwrap();
            bpid_rslt = &curline[strt..end];
        } else if curline.contains("DEVICE_VID=\"") {
            let strt = curline
                .find("DEVICE_VID=\"")
                .unwrap()
                .checked_add(12)
                .unwrap();
            let end = curline.rfind('\"').unwrap();
            vid_rslt = &curline[strt..end];
        } else if curline.contains("DEVICE_PID=\"") {
            let strt = curline
                .find("DEVICE_PID=\"")
                .unwrap()
                .checked_add(12)
                .unwrap();
            let end = curline.rfind('\"').unwrap();
            pid_rslt = &curline[strt..end];
        } else if curline.contains("USB_MANUFACTURER=\"") {
            let strt = curline
                .find("USB_MANUFACTURER=\"")
                .unwrap()
                .checked_add(18)
                .unwrap();
            let end = curline.rfind('\"').unwrap();
            usb_manufacturer = &curline[strt..end];
        } else if curline.contains("USB_PRODUCT=\"") {
            let strt = curline
                .find("USB_PRODUCT=\"")
                .unwrap()
                .checked_add(13)
                .unwrap();
            let end = curline.rfind('\"').unwrap();
            usb_product = &curline[strt..end];
        } else if curline.contains("USB_SERIAL_CHIP=\"") {
            let strt = curline
                .find("USB_SERIAL_CHIP=\"")
                .unwrap()
                .checked_add(17)
                .unwrap();
            let end = curline.rfind('\"').unwrap();
            usb_serial_chip = &curline[strt..end];
        }
    }
    println!("cargo:rustc-env=BOOT_VID={}", bvid_rslt);
    println!("cargo:rustc-env=BOOT_PID={}", bpid_rslt);
    println!("cargo:rustc-env=VID={}", vid_rslt);
    println!("cargo:rustc-env=PID={}", pid_rslt);
    println!("cargo:rustc-env=USB_MANUFACTURER={}", usb_manufacturer);
    println!("cargo:rustc-env=USB_PRODUCT={}", usb_product);
    println!("cargo:rustc-env=USB_SERIAL_CHIP={}", usb_serial_chip);
}