# Packet tracer

Packet Tracer is an embedded device designed to scan networks. This project is implemented using Rust and targets Pico W to provide network scanning capabilities.

## Installation
### Prerequisites
#### Hardware
* [Pico W](https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html)
* Debug Probe [example](https://www.raspberrypi.com/products/debug-probe/)

* Oled display with ssd1306

* Tactile buttons x5

* Led Diode 

#### Software
* [Cargo and Rust](https://www.rust-lang.org/tools/install)
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```
* [Probe-rs](https://probe.rs/) read more about it features
```bash
cargo install probe-rs-tools
```
* [Embassy](https://github.com/embassy-rs/embassy)
```bash
git clone https://github.com/embassy-rs/embassy
```
### Connecting Hardware
soon :)
### Running
1. Clone my repo and cd into it
```bash
git clone https://github.com/KacperJarocki/packet_tracer
cd packet_tracer
```
  
2. Edit Cargo.toml and adjust the path to embassy repo cloned on you machine
```toml
[dependencies]
embassy-embedded-hal = { version = "0.1.0", path = "../../embassy/embassy-embedded-hal", features = ["defmt"] }
embassy-sync = { version = "0.6.0", path = "../../embassy/embassy-sync", features = ["defmt"] }
embassy-executor = { version = "0.5.0", path = "../../embassy/embassy-executor", features = ["task-arena-size-98304", "arch-cortex-m", "executor-thread", "executor-interrupt", "defmt", "integrated-timers"] }
embassy-time = { version = "0.3.1", path = "../../embassy/embassy-time", features = ["defmt", "defmt-timestamp-uptime"] }
embassy-rp = { version = "0.1.0", path = "../../embassy/embassy-rp", features = ["defmt", "unstable-pac", "time-driver", "critical-section-impl"] }
embassy-usb = { version = "0.2.0", path = "../../embassy/embassy-usb", features = ["defmt"] }
embassy-net = { version = "0.4.0", path = "../../embassy/embassy-net", features = ["defmt", "tcp", "udp", "raw", "dhcpv4", "medium-ethernet", "dns"] }
embassy-net-wiznet = { version = "0.1.0", path = "../../embassy/embassy-net-wiznet", features = ["defmt"] }
embassy-usb-logger = { version = "0.2.0", path = "../../embassy/embassy-usb-logger" }
cyw43 = { version = "0.1.0", path = "../../embassy/cyw43", features = ["defmt", "firmware-logs"] }
cyw43-pio = { version = "0.1.0", path = "../../embassy/cyw43-pio", features = ["defmt", "overclock"] }
```
3. Run 
```bash
cargo run --release
```

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.
