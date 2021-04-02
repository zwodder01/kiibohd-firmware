# Hexgears Gemini Dusk/Dawn Firmware

## Building

### Initial Setup

```bash
rustup install nightly
rustup target add thumbv7em-none-eabi
rustup component add llvm-tools-preview
cargo install cargo-binutils
pipenv install
```


### First Build

pipenv is needed to run the Python kll compiler that's used in the kiibohd/controller repo.

```bash
pipenv run cargo +nightly build
```


## Use Bootloader

Uses dfu-util to flash the keyboard.
Make sure the keyboard is in flash mode (press the button on the back of the keyboard).

```bash
pipenv run cargo +nightly bin
dfu-util -a 0 -D target/gemini.dfu.bin
```


## GDB Testing

Requires a Segger JLink or OpenOCD compatible debugger attached to SWD.

```bash
JLinkGDBServer -if SWD -device ATSAM4S8B
# In a different terminal
cargo run
```


## License

Licensed under either of

 * Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.


### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
