rust-aobench
============

![aobench](https://raw.githubusercontent.com/sharow/rust-aobench/master/target/imgs/image.png)

[aobench](http://code.google.com/p/aobench/) @ Rust

## Cargo dependencies
rand = "0.3.8"


## Build & Run
```
$ rustc --version
rustc 1.0.0-beta.5 (built 2015-05-14)

$ cargo update
$ cargo run --release
Compiling libc v0.1.7
Compiling rand v0.3.8
Compiling aobench v0.0.2 (file:///....)
Running `target/release/aobench`

$ convert image.ppm image.png

```



