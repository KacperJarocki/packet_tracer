#![no_std]
#![no_main]

use core::str;
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::Stack;
use embassy_rp::peripherals::{DMA_CH0, PIO0, UART0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::uart::{self, Blocking};
use embassy_rp::{bind_interrupts, gpio};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use gpio::{Input, Level, Output, Pull};
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});
use embassy_rp::i2c::{self, Config};

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("set up led");
    let led = Output::new(p.PIN_17, Level::High);
    let led_button = Input::new(p.PIN_16, Pull::Up);
    spawner.spawn(button_task(led_button, led)).unwrap();
    info!("network set up");
    let network_button = Input::new(p.PIN_18, Pull::Up);
    let fw = include_bytes!("../../../embassy/cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../../embassy/cyw43-firmware/43439A0_clm.bin");
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    info!("set up i2c ");
    let sda = p.PIN_20;
    let scl = p.PIN_21;
    let mut i2c = i2c::I2c::new_blocking(p.I2C0, scl, sda, Config::default());
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    display.flush().unwrap();
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(wifi_task(runner)));
    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let config = uart::Config::default();
    let uart =
        uart::Uart::new_with_rtscts_blocking(p.UART0, p.PIN_0, p.PIN_1, p.PIN_3, p.PIN_2, config);
    unwrap!(spawner.spawn(scan_networks_task(network_button, control, uart)));
}

#[embassy_executor::task]
async fn button_task(mut button: Input<'static>, mut led: Output<'static>) {
    loop {
        button.wait_for_falling_edge().await;
        led.toggle();
        info!("i am in another task");
    }
}
#[embassy_executor::task]
async fn scan_networks_task(
    mut button: Input<'static>,
    mut control: cyw43::Control<'static>,
    mut uart: uart::Uart<'static, UART0, Blocking>,
) {
    loop {
        button.wait_for_falling_edge().await;
        info!("print networks");
        let mut scanner = control.scan(Default::default()).await;
        while let Some(bss) = scanner.next().await {
            if let Ok(ssid_str) = str::from_utf8(&bss.ssid) {
                info!("scanned {} == {:x}", ssid_str, bss.bssid);
                uart.blocking_write("Network: ".as_bytes()).unwrap();
                uart.blocking_write(ssid_str.as_bytes()).unwrap();
                uart.blocking_write(" bssid: ".as_bytes()).unwrap();
                uart.blocking_write(&bss.bssid).unwrap();
                uart.blocking_write("\n\r".as_bytes()).unwrap();
            }
        }
    }
}
