#![no_std]
#![no_main]

use panic_halt as _;

use core::sync::atomic::{AtomicUsize, Ordering};

use riscv_rt::entry;
use gd32vf103xx_hal::prelude::*;
use gd32vf103xx_hal::pac::{self, Interrupt, ECLIC};
use gd32vf103xx_hal::eclic::*;

use example_longan_nano_board::{USB, UsbBus};
use usb_device::class::UsbClass;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbVidPid};

use usbd_hid::descriptor::{KeyboardReport, KeyboardUsage, SerializedDescriptor};
use usbd_hid::hid_class::HIDClass;

const HELLO: [(u8, u8); 5] = [
    (
        KeyboardUsage::KeyboardLeftShift as u8,
        KeyboardUsage::KeyboardHh as u8,
    ),
    (0x00, KeyboardUsage::KeyboardEe as u8),
    (0x00, KeyboardUsage::KeyboardLl as u8),
    (0x00, KeyboardUsage::KeyboardLl as u8),
    (0x00, KeyboardUsage::KeyboardOo as u8),
];

static INDEX: AtomicUsize = AtomicUsize::new(0);

fn index() -> usize {
    INDEX.load(Ordering::Relaxed)
}

fn increment_index() -> usize {
    let index = (INDEX.load(Ordering::Relaxed) + 1) % HELLO.len();
    INDEX.store(index, Ordering::SeqCst);
    index
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Configure clocks
    let mut rcu = dp
        .RCU
        .configure()
        .ext_hf_clock(8.mhz())
        .sysclk(96.mhz())
        .freeze();

    assert!(rcu.clocks.usbclk_valid());

    let gpioa = dp.GPIOA.split(&mut rcu);
    let usb = USB {
        usb_global: dp.USBFS_GLOBAL,
        usb_device: dp.USBFS_DEVICE,
        usb_pwrclk: dp.USBFS_PWRCLK,
        pin_dm: gpioa.pa11,
        pin_dp: gpioa.pa12,
        hclk: rcu.clocks.hclk()
    };

    static mut EP_MEMORY: [u32; 1024] = [0; 1024];

    unsafe {
        let usb_bus = &*USB_BUS.insert(UsbBus::new(usb, &mut EP_MEMORY));
        HID_CLASS = Some(HIDClass::new(usb_bus, KeyboardReport::desc(), 1));
        USB_DEV = Some(UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1208, 0x1337))
        .manufacturer("Longboi")
        .product("Keeb")
        .build());
    }

    ECLIC::reset();
    ECLIC::set_threshold_level(Level::L0);
    ECLIC::set_level_priority_bits(LevelPriorityBits::L3P1);

    // setup the USBFS interrupt
    ECLIC::setup(
        Interrupt::USBFS,
        TriggerType::Level,
        Level::L1,
        Priority::P1,
    );

    // setup the USBFS_WKUP interrupt
    ECLIC::setup(
        Interrupt::USBFS_WKUP,
        TriggerType::Level,
        Level::L1,
        Priority::P2,
    );

    unsafe { 
        riscv::interrupt::enable();
    };

    loop {
        send_report();
    }
}

static mut HID_CLASS: Option<HIDClass<UsbBus<USB>>> = None;
static mut USB_BUS: Option<UsbBusAllocator<UsbBus<USB>>> = None;
static mut USB_DEV: Option<UsbDevice<UsbBus<USB>>> = None;

fn send_report() {
    let idx = index();
    let key = HELLO[idx];

    let report = KeyboardReport {
        modifier: key.0,
        reserved: 0,
        leds: 0,
        keycodes: [key.1, 0x00, 0x00, 0x00, 0x00, 0x00],
    };

    unsafe {
        let hid = HID_CLASS.as_mut().unwrap();
        if hid.push_input(&report).is_ok() {
            increment_index();
        }
    }
}

fn poll() {
    unsafe {
        let hid = HID_CLASS.as_mut().unwrap();

        if let Some(usb) = USB_DEV.as_mut() {
            if usb.poll(&mut [hid]) {
                hid.poll();
            }
        }
    }
}

#[allow(non_snake_case)]
#[no_mangle]
fn USBFS() {
    poll();
}

#[allow(non_snake_case)]
#[no_mangle]
fn USBFS_WKUP() {
    poll();
}
