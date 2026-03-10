#![no_std]
#![no_main]

mod protocol;
use protocol::*;

use embassy_executor::Spawner;
use embassy_stm32::peripherals::*;
use embassy_stm32::{Config, bind_interrupts, can, can::filter::*, flash, rcc};
use embassy_stm32::can::config::{
    ClockDivider, DataBitTiming, FdCanConfig, FrameTransmissionConfig,
    NominalBitTiming, TxBufferMode,
};
use embassy_time::{Duration, with_timeout};
use core::num::{NonZeroU16, NonZeroU8};
use embedded_can::Id;
use panic_halt as _;

macro_rules! log {
    ($($arg:tt)*) => {{
        #[cfg(debug_assertions)]
        rtt_target::rprintln!($($arg)*);
    }};
}

bind_interrupts!(struct Irqs {
    FDCAN2_IT0 => can::IT0InterruptHandler<FDCAN2>;
    FDCAN2_IT1 => can::IT1InterruptHandler<FDCAN2>;
});

unsafe fn jump_to_app() -> ! {
    unsafe {
        let mut p = cortex_m::Peripherals::steal();

        cortex_m::interrupt::disable();

        // Disable SysTick
        p.SYST.disable_counter();
        p.SYST.disable_interrupt();

        for i in 0..16 {
            p.NVIC.icer[i].write(0xFFFF_FFFF);
            p.NVIC.icpr[i].write(0xFFFF_FFFF);
        }

        let rcc = embassy_stm32::pac::RCC;
        rcc.cr().modify(|w| w.set_hsion(true));
        while !rcc.cr().read().hsirdy() {}
        rcc.cfgr().modify(|w| w.set_sw(embassy_stm32::pac::rcc::vals::Sw::HSI));
        while rcc.cfgr().read().sws() != embassy_stm32::pac::rcc::vals::Sw::HSI {}
        rcc.cr().modify(|w| w.set_pllon(0, false));

        p.SCB.invalidate_icache();
        p.SCB.vtor.write(0x0800_8000);

        cortex_m::asm::bootload(0x0800_8000 as *const u32);
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    #[cfg(debug_assertions)]
    rtt_target::rtt_init_print!();

    let mut config = Config::default();

    // 1. Configure HSI (64MHz) as the PLL source
    config.rcc.hsi = Some(rcc::HSIPrescaler::DIV1);

    // 2. Configure PLL1: HSI(64)/M(4) * N(30) = 480MHz VCO
    config.rcc.pll1 = Some(rcc::Pll {
            source: rcc::PllSource::HSI,
            prediv: rcc::PllPreDiv::DIV4,   // 64 / 4 = 16MHz input
            mul: rcc::PllMul::MUL30,        // 16 * 30 = 480MHz VCO
            divp: Some(rcc::PllDiv::DIV2),  // 480 / 2 = 240MHz (System Core)
            divq: Some(rcc::PllDiv::DIV12), // 480 / 12 = 40MHz (FDCAN)
            divr: None,
    });

    // 3. Set System Clock and FDCAN Mux
    config.rcc.sys = rcc::Sysclk::PLL1_P;
    config.rcc.mux.fdcan12sel = rcc::mux::Fdcansel::PLL1_Q;

    let peripherals = embassy_stm32::init(config);

    let uid_base = 0x08FFF800 as *const u32;
    let (w0, w1, w2) = unsafe {
        (
            core::ptr::read_volatile(uid_base),
            core::ptr::read_volatile(uid_base.add(1)),
            core::ptr::read_volatile(uid_base.add(2)),
        )
    };

    let this_node = match (w0, w1, w2) {
        (0x001E005F, 0x33335101, 0x32313831) => {
            log!("Detected Nuc1 UID");
            CanDevices::Nuc1
        }
        (0x00450024, 0x33335101, 0x32313831) => {
            log!("Detected Nuc2 UID");
            CanDevices::Nuc2
        }
        _ => {
            log!("Unknown UID, assuming Nuc1");
            CanDevices::UNKNOWN
        }
    };

    let target_id_mask = 0x1F << 21;
    let filter_all = ExtendedFilter {
        filter: FilterType::BitMask {
            filter: 0x01 << 21,
            mask: target_id_mask,
        },
        action: can::filter::Action::StoreInFifo0,
    };
    let filter_this_node = ExtendedFilter {
        filter: FilterType::BitMask {
            filter: (this_node as u32) << 21,
            mask: target_id_mask,
        },
        action: can::filter::Action::StoreInFifo0,
    };
    let mut can =
        can::CanConfigurator::new(peripherals.FDCAN2, peripherals.PB12, peripherals.PB13, Irqs);
    can.properties()
        .set_extended_filter(ExtendedFilterSlot::_0, filter_all);
    can.properties()
        .set_extended_filter(ExtendedFilterSlot::_1, filter_this_node);
    let config = FdCanConfig::default()
        .set_clock_divider(ClockDivider::_1)
        .set_frame_transmit(FrameTransmissionConfig::AllowFdCanAndBRS)
        .set_automatic_retransmit(false)
        .set_transmit_pause(false)
        .set_protocol_exception_handling(false)
        .set_tx_buffer_mode(TxBufferMode::Fifo)
        .set_nominal_bit_timing(NominalBitTiming {
            prescaler:        NonZeroU16::new(1).unwrap(),
            sync_jump_width:  NonZeroU8::new(6).unwrap(),
            seg1:             NonZeroU8::new(33).unwrap(),
            seg2:             NonZeroU8::new(6).unwrap(),
        })
        .set_data_bit_timing(DataBitTiming {
            transceiver_delay_compensation: true,
            prescaler:       NonZeroU16::new(1).unwrap(),
            seg1:            NonZeroU8::new(5).unwrap(),
            seg2:            NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(2).unwrap(),
        });

    can.set_config(config);

    #[allow(unused_mut)]
    let mut can = can.into_normal_mode();
    let (mut tx, mut rx, _props) = can.split();

    let this_node_id = this_node as u16;
    let host_id = CanDevices::RaspberryPi as u16;

    log!("CAN up, sending FirmwareUpdateQuery to host");

    // Send FirmwareUpdateQuery to host
    let query_id = DfrCanId::new(
        1,
        host_id,
        BootloaderCommand::FirmwareUpdateQuery.into(),
        this_node_id,
    ).unwrap();
    let query_frame = embassy_stm32::can::frame::FdFrame::new_extended(query_id.to_raw_id(), &[]).unwrap();
    tx.write_fd(&query_frame).await;

    // Wait 100ms for FirmwareUpdateResponse
    let stay_in_bootloader = match with_timeout(Duration::from_millis(100), async {
        loop {
            if let Ok(message) = rx.read_fd().await {
                let (rx_frame, _ts) = message.parts();
                if let Id::Extended(id) = rx_frame.id() {
                    let can_msg = parse_can_id(id.as_raw());
                    if can_msg.target == this_node_id {
                        if let Ok(BootloaderCommand::FirmwareUpdateResponse) = BootloaderCommand::try_from(can_msg.command) {
                            let data = rx_frame.data();
                            return data.first().copied().unwrap_or(0) == 1;
                        }
                    }
                }
            }
        }
    }).await {
        Ok(result) => result,
        Err(_) => false, // Timeout
    };

    if !stay_in_bootloader {
        log!("No firmware update, jumping to application");
        unsafe { jump_to_app(); }
    }

    log!("Firmware update requested, entering bootloader mode");

    #[allow(unused_mut)]
    let mut flash = flash::Flash::new_blocking(peripherals.FLASH).into_blocking_regions();
    let mut chunk_address: u32 = 0;
    let mut chunk_size: u8 = 0;
    let mut f = flash.bank1_region;

    loop {
        match rx.read_fd().await {
            Ok(message) => {
                let (rx_frame, _ts) = message.parts();

                if let Id::Extended(id) = rx_frame.id() {
                    let raw_id = id.as_raw();
                    let can_msg = parse_can_id(raw_id);
                    log!("CAN msg: target={:X} command={:X}", can_msg.target, can_msg.command);
                    if can_msg.target == this_node_id {
                        let data = rx_frame.data();

                        if let Ok(command) = BootloaderCommand::try_from(can_msg.command) {
                            match command {
                                BootloaderCommand::Ping => {
                                    log!("Command Ping received, replying with bootloader status");
                                    let reply_id = DfrCanId::new(
                                        1,
                                        can_msg.source,
                                        BootloaderCommand::Ping.into(),
                                        this_node_id,
                                    ).unwrap();
                                    let tx_frame = embassy_stm32::can::frame::FdFrame::new_extended(reply_id.to_raw_id(), &[0u8]).unwrap();
                                    tx.write_fd(&tx_frame).await;
                                }
                                BootloaderCommand::Erase => {
                                    f.blocking_erase(0x8000, 0x80000).unwrap();
                                    log!("Erase Complete");
                                    let reply_id = DfrCanId::new(
                                        1,
                                        can_msg.source,
                                        BootloaderCommand::EraseOk.into(),
                                        this_node_id,
                                    ).unwrap();
                                    let tx_frame = embassy_stm32::can::frame::FdFrame::new_extended(reply_id.to_raw_id(), &[]).unwrap();
                                    tx.write_fd(&tx_frame).await;
                                    log!("Sent EraseOk ACK");
                                }
                                BootloaderCommand::AddressAndSize => {
                                    if data.len() >= 5 {
                                        chunk_address = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
                                        chunk_size = data[4];
                                        log!("Prepared for Write at 0x{:08X} (Size: {})", chunk_address, chunk_size);
                                    }
                                }
                                BootloaderCommand::Write => {
                                    if chunk_address >= 0x0800_8000 {
                                        let offset = chunk_address - 0x0800_0000;

                                        let mut write_buf = [0xFF; 64];

                                        let aligned_len = (data.len() + 15) & !15;

                                        if aligned_len <= 64 {

                                            write_buf[..data.len()].copy_from_slice(data);

                                            f.blocking_write(offset, &write_buf[..aligned_len]).unwrap();
                                            log!("Wrote {} bytes (padded to {}) to {:x}", data.len(), aligned_len, chunk_address);

                                            let mut payload = [0u8; 5];
                                            payload[0..4].copy_from_slice(&chunk_address.to_be_bytes());
                                            payload[4] = chunk_size;

                                            let reply_id = DfrCanId::new(
                                                1,
                                                can_msg.source,
                                                BootloaderCommand::WriteOk.into(),
                                                this_node_id
                                            ).unwrap();

                                            let tx_frame = embassy_stm32::can::frame::FdFrame::new_extended(reply_id.to_raw_id(), &payload).unwrap();
                                            tx.write_fd(&tx_frame).await;

                                            log!("Sent WriteOk ACK for 0x{:08X}", chunk_address);

                                            chunk_address += data.len() as u32;
                                        }
                                    }
                                }
                                BootloaderCommand::Jump => {
                                    log!("Jumping to application at 0x{:08X}", 0x0800_8000u32);
                                    unsafe { jump_to_app(); }
                                }
                                _ => log!("Unhandled command 0x{:04X}", can_msg.command),
                            }
                        } else {
                            log!("Unknown command ID 0x{:x}", raw_id);
                        }
                    }
                }
            }
            Err(e) => log!("CAN read error: {:?}", e),
        }
    }
}
