#![no_std]
#![no_main]

use core::str;
use cyw43::BssInfo;
use cyw43_pio::PioSpi;
use defmt::{info, *};
use embassy_executor::Spawner;
use embassy_net::Stack;
use embassy_rp::peripherals::{DMA_CH0, I2C0, PIO0, UART0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::uart::{self, Blocking};
use embassy_rp::{bind_interrupts, gpio};
use embassy_time::Timer;
use embedded_graphics::{
    mono_font::{ascii::FONT_5X7, MonoTextStyleBuilder},
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
use embassy_executor::Executor;
use embassy_rp::i2c::{self, Config, I2c};
use embassy_rp::multicore::{self, spawn_core1};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use heapless::Vec;

static mut CORE1_STACK: multicore::Stack<4096> = multicore::Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static CHANNEL: Channel<CriticalSectionRawMutex, Vec<cyw43::BssInfo, 10>, 1> = Channel::new();

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
    let led_button = Input::new(p.PIN_16, Pull::Up);
    let led = Output::new(p.PIN_17, Level::High);
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
    let config = uart::Config::default();
    let uart =
        uart::Uart::new_with_rtscts_blocking(p.UART0, p.PIN_0, p.PIN_1, p.PIN_3, p.PIN_2, config);
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, control, runner) = cyw43::new(state, pwr, spi, fw).await;
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            info!("set up i2c ");
            let sda = p.PIN_20;
            let scl = p.PIN_21;
            let i2c = i2c::I2c::new_blocking(p.I2C0, scl, sda, Config::default());
            let interface = I2CDisplayInterface::new(i2c);
            let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
                .into_buffered_graphics_mode();
            if display.init().is_err() {
                info!("display init failed");
            } else {
                info!("display init successful");
            }
            info!("executcor should run");
            executor1.run(|spawner| {
                if spawner
                    .spawn(change_display_output(display, "hello form core 1"))
                    .is_err()
                {
                    info!("fail to spawn  change_display_output");
                } else {
                    info!("spawned change_display_output");
                }
                if spawner.spawn(button_task(led_button, led)).is_err() {
                    info!("fail to spawn button_task");
                } else {
                    info!("spawned button_task");
                };
            });
        },
    );
    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        info!("spawning tasks");
        unwrap!(spawner.spawn(wifi_task(runner)));
        unwrap!(spawner.spawn(scan_networks_task(control, uart, clm, network_button)));
    });
}
#[embassy_executor::task]
async fn change_display_output(
    mut display: Ssd1306<
        I2CInterface<I2c<'static, I2C0, i2c::Blocking>>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    >,
    mess: &'static str,
) {
    loop {
        Timer::after_millis(50).await;
        info!("clearing display");
        if display.clear(BinaryColor::Off).is_err() {
            info!("clearing failed");
        } else {
            info!("clearing successful");
        }
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_5X7)
            .text_color(BinaryColor::On)
            .build();
        info!("displaying mess {}", mess);
        Text::with_baseline(mess, Point::zero(), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        info!("waitnig for recv");
        let mut bss_vec = CHANNEL.receive().await;
        info!("recv channel to display");
        for i in 0..10 {
            let bss = bss_vec.pop().unwrap();
            let x = 6 * i;
            if let Ok(ssid_str) = str::from_utf8(&bss.ssid) {
                info!("will display {} == {:x}", ssid_str, bss.bssid);
                Text::with_baseline(ssid_str, Point::new(0, x), text_style, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();
            }
        }

        if display.flush().is_err() {
            info!("flushing failed");
        } else {
            info!("flushing successful");
        }
        info!("displayed");
    }
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
    mut control: cyw43::Control<'static>,
    mut uart: uart::Uart<'static, UART0, Blocking>,
    clm: &'static [u8],
    mut network_button: Input<'static>,
) {
    info!("waitnig for load");
    control.init(clm).await;
    info!("set power management");
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;
    info!("waiting for 2 seconds");
    Timer::after_secs(2).await;
    loop {
        info!("waiting for button");
        network_button.wait_for_falling_edge().await;
        info!("create a new vec");
        let mut bss_vec: Vec<BssInfo, 10> = Vec::new();
        info!("print networks");
        let mut scanner = control.scan(Default::default()).await;
        info!("scanning");
        while let Some(bss) = scanner.next().await {
            info!("add BssInfo to vec");
            if bss_vec.push(bss).is_err() {
                info!("Vec is overflow")
            } else {
                info!("Added bss to Vec")
            }
            if let Ok(ssid_str) = str::from_utf8(&bss.ssid) {
                info!("scanned {} == {:x}", ssid_str, bss.bssid);
                uart.blocking_write("Network: ".as_bytes()).unwrap();
                uart.blocking_write(ssid_str.as_bytes()).unwrap();
                uart.blocking_write(" bssid: ".as_bytes()).unwrap();
                uart.blocking_write(&bss.bssid).unwrap();
                uart.blocking_write("\n\r".as_bytes()).unwrap();
            }
        }

        info!("sending bss_vec to channel");
        CHANNEL.send(bss_vec).await;
    }
}
