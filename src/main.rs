#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt::Write;
use core::str;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use cyw43::BssInfo;
use cyw43_pio::PioSpi;
use defmt::{debug, info, unwrap};
use embassy_executor::Spawner;
use embassy_net::Stack;
use embassy_rp::peripherals::{DMA_CH0, I2C0, PIO0, UART0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::uart::{self, Blocking};
use embassy_rp::{bind_interrupts, gpio};
use embassy_sync::blocking_mutex;
use embassy_time::Timer;
use embedded_graphics::mono_font::iso_8859_7::FONT_6X12;
use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
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
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::channel::Channel;
use heapless::Vec;

static mut CORE1_STACK: multicore::Stack<8192> = multicore::Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static CHANNEL: Channel<CriticalSectionRawMutex, cyw43::BssInfo, 1> = Channel::new();
static NEW_INFO_SEND: AtomicBool = AtomicBool::new(false);
static VIEW_MORE_INFO: AtomicBool = AtomicBool::new(false);
static BSS_VEC_MUTEX: blocking_mutex::Mutex<ThreadModeRawMutex, RefCell<Vec<BssInfo, 25>>> =
    blocking_mutex::Mutex::new(RefCell::new(Vec::new()));
static INDEX_NETWORKS: AtomicUsize = AtomicUsize::new(0);

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
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
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
            debug!("set up led");
            let _led_button = Input::new(p.PIN_16, Pull::Up);
            let _led = Output::new(p.PIN_17, Level::High);
            let up_button = Input::new(p.PIN_15, Pull::Up);
            let down_button = Input::new(p.PIN_14, Pull::Up);
            let in_out_button = Input::new(p.PIN_13, Pull::Up);
            debug!("set up i2c ");
            let sda = p.PIN_20;
            let scl = p.PIN_21;
            let i2c = i2c::I2c::new_blocking(p.I2C0, scl, sda, Config::default());
            let interface = I2CDisplayInterface::new(i2c);
            let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
                .into_buffered_graphics_mode();
            if display.init().is_err() {
                debug!("display init failed");
            } else {
                debug!("display init successful");
            }
            debug!("executcor should run");
            executor1.run(|spawner| {
                unwrap!(spawner.spawn(update_networks_task()));
                if spawner.spawn(change_display_output(display)).is_err() {
                    debug!("fail to spawn change_display_output");
                } else {
                    debug!("spawned change_display_output");
                }
                unwrap!(spawner.spawn(in_out_button_task(in_out_button)));
                unwrap!(spawner.spawn(down_button_task(down_button)));
                unwrap!(spawner.spawn(up_button_task(up_button)));
            });
        },
    );
    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        debug!("spawning tasks");
        unwrap!(spawner.spawn(wifi_task(runner)));
        unwrap!(spawner.spawn(scan_networks_task(control, uart, clm, network_button)));
    });
}
#[embassy_executor::task]
async fn up_button_task(mut button: Input<'static>) {
    loop {
        button.wait_for_falling_edge().await;
        let mut index = INDEX_NETWORKS.load(Ordering::Relaxed);
        index += 1;
        INDEX_NETWORKS.store(index, Ordering::Relaxed);
    }
}
#[embassy_executor::task]
async fn down_button_task(mut button: Input<'static>) {
    loop {
        button.wait_for_falling_edge().await;
        let index = INDEX_NETWORKS.load(Ordering::Relaxed);
        let new_index = index.saturating_sub(1);
        INDEX_NETWORKS.store(new_index, Ordering::Relaxed);
    }
}
#[embassy_executor::task]
async fn in_out_button_task(mut button: Input<'static>) {
    loop {
        button.wait_for_falling_edge().await;
        let stored = VIEW_MORE_INFO.load(Ordering::Relaxed);
        let new = !stored;
        VIEW_MORE_INFO.store(new, Ordering::Relaxed);
    }
}
#[embassy_executor::task]
async fn update_networks_task() {
    loop {
        debug!("waitnig for recv");
        let bss = CHANNEL.receive().await;
        info!("adding to mutex");
        BSS_VEC_MUTEX.lock(|bss_vec| {
            if bss_vec.borrow_mut().push(bss).is_err() {
                info!("Vec is overflow")
            } else {
                info!("Added bss to Vec")
            }
        })
    }
}
#[embassy_executor::task]
async fn change_display_output(
    mut display: Ssd1306<
        I2CInterface<I2c<'static, I2C0, i2c::Blocking>>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    >,
) {
    info!("clearing display");
    if display.clear(BinaryColor::Off).is_err() {
        info!("clearing failed");
    } else {
        info!("clearing successful");
    }
    if display.flush().is_err() {
        info!("flushing failed");
    } else {
        info!("flushing successful");
    }
    loop {
        Timer::after_millis(10).await;
        info!("clearing display");
        if display.clear(BinaryColor::Off).is_err() {
            info!("clearing failed");
        } else {
            info!("clearing successful");
        }
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X12)
            .text_color(BinaryColor::On)
            .build();
        let index = INDEX_NETWORKS.load(Ordering::Relaxed);
        BSS_VEC_MUTEX.lock(|vec| {
            let bss_vec = vec.borrow_mut();
            let end = match index + 5 > bss_vec.len() {
                true => bss_vec.len(),
                false => index + 5,
            };
            info!("index {}, end {}", index, end);
            let mut y: i32 = 1;
            let go_in_or_go_out = VIEW_MORE_INFO.load(Ordering::Relaxed);
            if !go_in_or_go_out {
                let mut buffer: heapless::String<30> = heapless::String::new();
                write!(buffer, "Networks:       {}/{}", index, bss_vec.len()).ok();
                Text::with_baseline(buffer.as_str(), Point::new(0, 0), text_style, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();
                for i in index..end {
                    let bss: BssInfo = bss_vec[i];
                    let x = 12 * y;
                    if let Ok(ssid_str) = str::from_utf8(&bss.ssid) {
                        let (mut ssid_str, _) = ssid_str.split_at(bss.ssid_len.into());
                        if ssid_str.is_empty() {
                            ssid_str = "Unknown ssid";
                        }
                        let postions = x;
                        Text::with_baseline(
                            ssid_str.trim(),
                            Point::new(0, postions),
                            text_style,
                            Baseline::Top,
                        )
                        .draw(&mut display)
                        .unwrap();
                    }
                    y += 1;
                }
            } else {
                let mut buffer: heapless::String<64> = heapless::String::new();
                let bss = bss_vec[index];
                if let Ok(ssid_str) = str::from_utf8(&bss.ssid) {
                    let (mut ssid_str, _useless) = ssid_str.split_at(bss.ssid_len.into());
                    if ssid_str.is_empty() {
                        ssid_str = "Unknown ssid";
                    }
                    write!(buffer, "SSID: {}\n", ssid_str).ok();
                    write!(buffer, "Channel: {}\n", bss.chanspec).ok();
                    write!(buffer, "RSSI: {}\n", bss.rssi).ok();
                    write!(buffer, "BSSID: {:02X?}\n", bss.bssid).ok();
                    write!(buffer, "SNR: {}\n", bss.snr).ok();
                    let buffer = buffer.as_str();

                    Text::with_baseline(buffer, Point::zero(), text_style, Baseline::Top)
                        .draw(&mut display)
                        .unwrap();
                }
            }
            if display.flush().is_err() {
                debug!("flushing failed");
            } else {
                debug!("flushing successful");
            }
            debug!("displayed");
        });

        let info = NEW_INFO_SEND.load(Ordering::Relaxed);
        if info {
            debug!("creating new vector");
            BSS_VEC_MUTEX.lock(|bss_vec_cell| bss_vec_cell.take().clear());
            NEW_INFO_SEND.store(false, Ordering::Relaxed);
        }
    }
}

#[embassy_executor::task]
async fn scan_networks_task(
    mut control: cyw43::Control<'static>,
    mut uart: uart::Uart<'static, UART0, Blocking>,
    clm: &'static [u8],
    mut button: Input<'static>,
) {
    info!("waitnig for load");
    control.init(clm).await;
    info!("set power management");
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;
    info!("waiting for 2 seconds");
    Timer::after_secs(1).await;
    loop {
        info!("print networks");
        let mut scanner = control.scan(Default::default()).await;
        info!("scanning");
        while let Some(bss) = scanner.next().await {
            CHANNEL.send(bss).await;
            if let Ok(ssid_str) = str::from_utf8(&bss.ssid) {
                info!("scanned {} == {:x}", ssid_str, bss.bssid);
                uart.blocking_write("Network: ".as_bytes()).unwrap();
                uart.blocking_write(ssid_str.as_bytes()).unwrap();
                uart.blocking_write(" bssid: ".as_bytes()).unwrap();
                uart.blocking_write(&bss.bssid).unwrap();
                uart.blocking_write("\n\r".as_bytes()).unwrap();
            }
        }

        info!("waiting for button");
        button.wait_for_falling_edge().await;
        info!("should delete all info in other task");
        NEW_INFO_SEND.store(true, Ordering::Relaxed)
    }
}
