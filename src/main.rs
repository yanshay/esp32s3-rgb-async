#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]
#![feature(box_as_ptr)]

use esp_hal::clock::CpuClock;
use esp_hal::dma::{CHUNK_SIZE, DmaDescriptor, DmaTxBuf};
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::lcd_cam::LcdCam;
use esp_hal::peripherals::{self, Peripherals};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;

use log::{error, info};

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};

use alloc::boxed::Box;
use embedded_graphics_core::{
    pixelcolor::Rgb565,
    prelude::{IntoStorage, RgbColor},
};

use esp_backtrace as _;

#[unsafe(link_section = ".rwtext")]
pub unsafe fn cache_writeback_addr(addr: u32, size: u32) {
    unsafe extern "C" {
        fn rom_Cache_WriteBack_Addr(addr: u32, size: u32);
        fn Cache_Suspend_DCache_Autoload() -> u32;
        fn Cache_Resume_DCache_Autoload(value: u32);
    }
    // suspend autoload, avoid load cachelines being written back
    unsafe {
        let autoload = Cache_Suspend_DCache_Autoload();
        rom_Cache_WriteBack_Addr(addr, size);
        Cache_Resume_DCache_Autoload(autoload);
    }
}

extern crate alloc;

fn init_psram_heap(start: *mut u8, size: usize) {
    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            start,
            size,
            esp_alloc::MemoryCapability::External.into(),
        ));
    }
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

// === Display constants ===
const LCD_H_RES: u16 = 800;
const LCD_V_RES: u16 = 480;
const FRAME_BYTES: usize = (LCD_H_RES as usize * LCD_V_RES as usize) * 2;

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let psram_config = esp_hal::psram::PsramConfig {
        ram_frequency: esp_hal::psram::SpiRamFreq::Freq80m,
        ..Default::default()
    };

    let config = esp_hal::Config::default()
        .with_cpu_clock(CpuClock::max())
        .with_psram(psram_config);
    let peripherals = esp_hal::init(config);

    let (start, size) = esp_hal::psram::psram_raw_parts(&peripherals.PSRAM);

    // IMPORTANT: PSRAM need to be initialized first, so 'Normal' allocations will use the region
    info!("Before init_psram_heap");
    init_psram_heap(start, size);
    esp_alloc::heap_allocator!(size: 300 * 1024);
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    // let sw_int =
    //     esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    //
    // static EXECUTOR_CORE_0: static_cell::StaticCell<esp_rtos::embassy::InterruptExecutor<2>> =
    //     static_cell::StaticCell::new();
    // let executor_core0 = esp_rtos::embassy::InterruptExecutor::new(sw_int.software_interrupt2);
    // let executor_core0 = EXECUTOR_CORE_0.init(executor_core0);
    //
    // let core0_spawner = executor_core0.start(esp_hal::interrupt::Priority::Priority1);

    info!("Embassy initialized!");

    // Done with esp-hal initialization /////////////////////////////////////////////////////////////

    // Prepare shared memory for drive_display and app tasks ////////////////////////////////////////

    // Allocate a PSRAM-backed DMA buffer for the frame
    let original_buf_box: Box<[u8; FRAME_BYTES]> = Box::new([0; FRAME_BYTES]);
    let ptr = Box::into_raw(original_buf_box);
    let buf_box: Box<[u8; FRAME_BYTES]> = unsafe { Box::from_raw(ptr) };
    let buf_box2: Box<[u8; FRAME_BYTES]> = unsafe { Box::from_raw(ptr) };

    // core0_spawner.spawn(drive_display(buf_box)).ok();
    spawner.spawn(drive_display(buf_box)).ok();
    spawner.spawn(app(buf_box2)).ok();

    // Done with UI stuff initialization /////////////////////////////////////////////////////////////

    loop {
        info!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v~1.0/examples
}

#[embassy_executor::task]
pub async fn drive_display(buf_box: Box<[u8; FRAME_BYTES]>) {
    let ptr = Box::into_raw(buf_box);
    let buf_box: Box<[u8; FRAME_BYTES]> = unsafe { Box::from_raw(ptr) };
    let buf_box3: Box<[u8; FRAME_BYTES]> = unsafe { Box::from_raw(ptr) };

    let peripherals = unsafe { Peripherals::steal() };

    // turn on backlight
    let _ = Output::new(peripherals.GPIO2, Level::High, OutputConfig::default());

    let tx_channel = peripherals.DMA_CH2; // TODO: should be received externally
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    // // ST7262 IPS LCD 800x480
    //  Arduino_RPi_DPI_RGBPanel *gfx = new Arduino_RPi_DPI_RGBPanel(
    //    bus,
    //    800 /* width */, 0 /* hsync_polarity */, 8 /* hsync_front_porch */, 4 /* hsync_pulse_width */, 8 /* hsync_back_porch */,
    //    480 /* height */, 0 /* vsync_polarity */, 8 /* vsync_front_porch */, 4 /* vsync_pulse_width */, 8 /* vsync_back_porch */,
    //    1 /* pclk_active_neg */, 16000000 /* prefer_speed */, true /* auto_flush */);

    let config = esp_hal::lcd_cam::lcd::dpi::Config::default()
        .with_clock_mode(esp_hal::lcd_cam::lcd::ClockMode {
            polarity: esp_hal::lcd_cam::lcd::Polarity::IdleLow,
            phase: esp_hal::lcd_cam::lcd::Phase::ShiftHigh, // High, not Low
        })
        .with_frequency(Rate::from_mhz(16))
        .with_format(esp_hal::lcd_cam::lcd::dpi::Format {
            enable_2byte_mode: true,
            ..Default::default()
        })
        .with_timing(esp_hal::lcd_cam::lcd::dpi::FrameTiming {
            horizontal_active_width: 800,
            horizontal_total_width: 808,
            horizontal_blank_front_porch: 8,

            vertical_active_height: 480,
            vertical_total_height: 488,
            vertical_blank_front_porch: 8,

            hsync_width: 4,
            vsync_width: 4,

            hsync_position: 8,
        })
        .with_vsync_idle_level(Level::High)
        .with_hsync_idle_level(Level::High)
        .with_de_idle_level(Level::Low)
        .with_disable_black_region(false);

    // Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    //     GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
    //     40 /* DE */, 41 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
    //     45 /* R0 */, 48 /* R1 */, 47 /* R2 */, 21 /* R3 */, 14 /* R4 */,
    //     5 /* G0 */, 6 /* G1 */, 7 /* G2 */, 15 /* G3 */, 16 /* G4 */, 4 /* G5 */,
    //     8 /* B0 */, 3 /* B1 */, 46 /* B2 */, 9 /* B3 */, 1 /* B4 */
    // );

    let mut dpi = esp_hal::lcd_cam::lcd::dpi::Dpi::new(lcd_cam.lcd, tx_channel, config)
        .unwrap()
        .with_vsync(peripherals.GPIO41) // 41
        .with_hsync(peripherals.GPIO39)
        .with_de(peripherals.GPIO40)
        .with_pclk(peripherals.GPIO42)
        // Blue
        .with_data0(peripherals.GPIO8)
        .with_data1(peripherals.GPIO3)
        .with_data2(peripherals.GPIO46)
        .with_data3(peripherals.GPIO9)
        .with_data4(peripherals.GPIO1)
        // Green
        .with_data5(peripherals.GPIO5)
        .with_data6(peripherals.GPIO6)
        .with_data7(peripherals.GPIO7)
        .with_data8(peripherals.GPIO15)
        .with_data9(peripherals.GPIO16)
        .with_data10(peripherals.GPIO4)
        // Red
        .with_data11(peripherals.GPIO45)
        .with_data12(peripherals.GPIO48)
        .with_data13(peripherals.GPIO47)
        .with_data14(peripherals.GPIO21)
        .with_data15(peripherals.GPIO14);

    let psram_buf: &'static mut [u8] = Box::leak(buf_box);

    #[allow(clippy::manual_div_ceil)]
    const NUM_DMA_DESC: usize = (FRAME_BYTES + CHUNK_SIZE - 1) / CHUNK_SIZE;
    info!("NUM_DMA_DESC: {NUM_DMA_DESC}");
    #[unsafe(link_section = ".dma")]
    static mut TX_DESCRIPTORS: [DmaDescriptor; NUM_DMA_DESC] = [DmaDescriptor::EMPTY; NUM_DMA_DESC];
    #[allow(static_mut_refs)]
    let mut dma_tx: DmaTxBuf = unsafe { DmaTxBuf::new(&mut TX_DESCRIPTORS, psram_buf).unwrap() };
    unsafe {
        TX_DESCRIPTORS[NUM_DMA_DESC-1].next = &mut TX_DESCRIPTORS[0];
        TX_DESCRIPTORS[NUM_DMA_DESC-1].set_suc_eof(true);
    }

    info!("Transfering");
    Timer::after_secs(1).await;

    let xfer_holder = match dpi.send(true, dma_tx) {
        Ok(xfer) => {
            // Strange behavior, to see display need to check is_done and wait and after wait it stops even though the next lines are also wait
            if !xfer.is_done() {
                info!("Tested Transfer is done");
            }
            Some(xfer)
        }
        Err((e, _, _)) => {
            error!("Initial DMA send error: {:?}", e);
            None
        }
    };

    info!("buf_box3_len {}", buf_box3.len());
    loop {
        Timer::after_millis(100).await;
        unsafe {cache_writeback_addr(buf_box3.as_ptr() as u32, buf_box3.len() as u32); }
    }
    loop {
        let _start = esp_hal::time::Instant::now();
        match dpi.send(false, dma_tx) {
            Ok(xfer) => {
                let start = Instant::now();
                Timer::after_micros(20000).await; // 26438 is optimal w/o other tasks running
                let mut yield_count = 0;
                // loop {
                //     if !xfer.is_done() {
                //         embassy_futures::yield_now().await;
                //         yield_count += 1;
                //     } else {
                let (_res, dpi2, tx2) = xfer.wait();
                dpi = dpi2;
                dma_tx = tx2;
                // break;
                //  }
                // }
                let took = start.elapsed();
                // info!("took: {}, yield_count={yield_count}", took.as_micros());
            }
            Err((e, dpi2, tx2)) => {
                error!("Initial DMA send error: {:?}", e);
                dpi = dpi2;
                dma_tx = tx2;
            }
        }
        // info!("{}", start.elapsed().as_micros());
    }
}

#[embassy_executor::task]
pub async fn app(mut buf_box2: Box<[u8; FRAME_BYTES]>) {
    const FRAME_PIXELS: usize = (LCD_H_RES as usize) * (LCD_V_RES as usize);

    // Clear screen to blue for sanity check
    let [blue_lo, blue_hi] = Rgb565::BLUE.into_storage().to_le_bytes();
    {
        for pixel in 0..FRAME_PIXELS {
            buf_box2[2 * pixel] = blue_lo;
            buf_box2[2 * pixel + 1] = blue_hi;
        }
    }

    let [yellow_lo, yellow_hi] = Rgb565::YELLOW.into_storage().to_le_bytes();
    for y in 0..479  {
        for x in [0, 400, 799] {
            buf_box2[(y * LCD_H_RES as usize + x) * 2] = yellow_lo;
            buf_box2[(y * LCD_H_RES as usize + x) * 2 + 1] = yellow_hi;
        }
    }
    for x in 0..799 {
        for y in [0, 240, 479] {
            buf_box2[(y * LCD_H_RES as usize + x) * 2] = yellow_lo;
            buf_box2[(y * LCD_H_RES as usize + x) * 2 + 1] = yellow_hi;
        }
    }

    let [red_lo, red_hi] = Rgb565::RED.into_storage().to_le_bytes();
    let mut x = 0;
    let mut y = 0;
    let mut dx: isize = 1;
    let mut dy: isize = 1;
    loop {
        let idx = (y * LCD_H_RES as usize + x) * 2;

        buf_box2[idx] = red_lo;
        buf_box2[idx+1] = red_hi;
        if x >= 799 {
            dx = -1;
        };
        if x == 0 {
            dx = 1;
            info!("Drawing");
        };
        if y >= 479 {
            dy = -1;
        };
        if y == 0 {
            dy = 1;
        };
        x = (x as isize + dx) as usize;
        y = (y as isize + dy) as usize;

        // unsafe {cache_writeback_addr(buf_box2.as_ptr().add(idx) as u32, 2); }
        Timer::after_micros(10000).await;
        // let start = Instant::now();
        // loop {
        //     if start.elapsed() > Duration::from_millis(100) { break; }
        // }
    }
}
