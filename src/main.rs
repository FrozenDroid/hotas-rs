#![feature(assoc_int_consts)]
#![no_std]
#![no_main]

extern crate panic_semihosting;
use stm32l4xx_hal::stm32 as stm;

use crate::descriptor::JoystickReport;

use core::fmt::Write;
use cortex_m::asm;
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use stm::interrupt;
use stm32_usbd::UsbBus;
use stm32l4xx_hal::adc::Adc;
use stm32l4xx_hal::prelude::*;
use stm32l4xx_hal::serial::{Config, Serial};
use stm32l4xx_hal::stm32;
use stm32l4xx_hal::usb::Peripheral;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usb_device::prelude::{UsbDeviceBuilder, UsbVidPid};

use usbd_hid::descriptor::SerializedDescriptor;
use usbd_hid::hid_class::HIDClass;
use usbd_serial::SerialPort;

mod descriptor;

fn enable_crs() {
    let rcc = unsafe { &(*stm32::RCC::ptr()) };
    rcc.apb1enr1.modify(|_, w| w.crsen().set_bit());
    let crs = unsafe { &(*stm32::CRS::ptr()) };
    // Initialize clock recovery
    // Set autotrim enabled.
    crs.cr.modify(|_, w| w.autotrimen().set_bit());
    // Enable CR
    crs.cr.modify(|_, w| w.cen().set_bit());
}

/// Enables VddUSB power supply
fn enable_usb_pwr() {
    // Enable PWR peripheral
    let rcc = unsafe { &(*stm32::RCC::ptr()) };
    rcc.apb1enr1.modify(|_, w| w.pwren().set_bit());

    // Enable VddUSB
    let pwr = unsafe { &*stm32::PWR::ptr() };
    pwr.cr2.modify(|_, w| w.usv().set_bit());
}

#[entry]
fn main() -> ! {
    asm::nop();

    let dp = stm::Peripherals::take().unwrap();
    let mut cp = stm::CorePeripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .hsi48(true)
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .pclk2(24.mhz())
        .lsi(true)
        .freeze(&mut flash.acr);

    enable_crs();

    enable_usb_pwr();

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);

    let pa2 = gpioa.pa2.into_af7(&mut gpioa.moder, &mut gpioa.afrl);
    let pa3 = gpioa.pa3.into_af7(&mut gpioa.moder, &mut gpioa.afrl);

    let mut usart2 = Serial::usart2(
        dp.USART2,
        (pa2, pa3),
        Config::default(),
        clocks,
        &mut rcc.apb1r1,
    );

    let usb = stm32l4xx_hal::usb::Peripheral {
        usb: dp.USB,
        pin_dm: gpioa.pa11.into_af10(&mut gpioa.moder, &mut gpioa.afrh),
        pin_dp: gpioa.pa12.into_af10(&mut gpioa.moder, &mut gpioa.afrh),
    };

    let bus_allocator = unsafe {
        USB_ALLOCATOR = Some(UsbBus::new(usb));
        USB_ALLOCATOR.as_ref().unwrap()
    };

    unsafe {
        USB_HID = Some(HIDClass::new(&bus_allocator, JoystickReport::desc(), 10));

        USB_BUS = Some(
            UsbDeviceBuilder::new(&bus_allocator, UsbVidPid(0x16c0, 0x27dd))
                .product("hotas-rs")
                .device_class(0xEF)
                .build(),
        );

        cp.NVIC.set_priority(stm::interrupt::USB, 1);
        NVIC::unmask(interrupt::USB);
    }

    let mut pa0 = gpioa.pa0.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    let mut pa1 = gpioa.pa1.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

    let mut adc = Adc::adc1(dp.ADC, &mut rcc.ahb2);

    loop {
        let x: u16 = adc.read(&mut pa0).unwrap();
        let y: u16 = adc.read(&mut pa1).unwrap();
        unsafe {
            writeln!(usart2, "x: \t{:?}, y: \t{:?}\r", x, y);
            USB_HID
                .as_mut()
                .unwrap()
                .push_input(&core::mem::transmute::<_, [u8; 4]>(JoystickReport {
                    x: (x as i16) - i16::MAX,
                    y: (y as i16) - i16::MAX,
                }));
        }
    }
}

static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus<Peripheral>>> = None;
static mut USB_BUS: Option<UsbDevice<UsbBus<Peripheral>>> = None;
static mut USB_HID: Option<HIDClass<UsbBus<Peripheral>>> = None;

#[interrupt]
fn USB() {
    unsafe {
        USB_BUS.as_mut().map(|usb_dev| {
            USB_HID.as_mut().map(|hid| {
                usb_dev.poll(&mut [hid]);
            });
        });
    };
}
