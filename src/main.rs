#![no_std]
#![no_main]

mod protocol;
use protocol::*;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::peripherals::*;
use embassy_stm32::{Config, bind_interrupts, can, can::filter::*, flash, rcc};
use embassy_stm32::can::config::{
    ClockDivider, DataBitTiming, FdCanConfig, FrameTransmissionConfig,
    NominalBitTiming, TxBufferMode,
};
use core::num::{NonZeroU16, NonZeroU8};
use embedded_can::Id;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
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

    // Use these words for matching
    let this_node = match (w0, w1, w2) {
        (0x001E005F, 0x33335101, 0x32313831) => {
            info!("Detected Nuc1 UID");
            CanDevices::Nuc1
        }
        (0x00450024, 0x33335101, 0x32313831) => {
            info!("Detected Nuc2 UID");
            CanDevices::Nuc2
        }
        _ => {
            info!("Unknown UID, assuming Nuc1");
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
        can::CanConfigurator::new(peripherals.FDCAN1, peripherals.PA11, peripherals.PA12, Irqs);
    can.properties()
        .set_extended_filter(ExtendedFilterSlot::_0, filter_all);
    can.properties()
        .set_extended_filter(ExtendedFilterSlot::_1, filter_this_node);
    let config = FdCanConfig::default()
        .set_clock_divider(ClockDivider::_1)                  // FDCAN_CLOCK_DIV1
        .set_frame_transmit(FrameTransmissionConfig::AllowFdCanAndBRS) // FDCAN_FRAME_FD_BRS
        .set_automatic_retransmit(false)                       // AutoRetransmission = DISABLE
        .set_transmit_pause(false)                             // TransmitPause = DISABLE
        .set_protocol_exception_handling(false)                // ProtocolException = DISABLE
        .set_tx_buffer_mode(TxBufferMode::Fifo)                // FDCAN_TX_FIFO_OPERATION
        .set_nominal_bit_timing(NominalBitTiming {
            prescaler:        NonZeroU16::new(1).unwrap(),     // NominalPrescaler = 1
            sync_jump_width:  NonZeroU8::new(6).unwrap(),      // NominalSyncJumpWidth = 6
            seg1:             NonZeroU8::new(33).unwrap(),     // NominalTimeSeg1 = 33
            seg2:             NonZeroU8::new(6).unwrap(),      // NominalTimeSeg2 = 6
        })
        .set_data_bit_timing(DataBitTiming {
            transceiver_delay_compensation: true,              // TDC required at 5Mbps
            prescaler:       NonZeroU16::new(1).unwrap(),      // tq = 1/40MHz = 25ns
            seg1:            NonZeroU8::new(5).unwrap(),       // 5 tq
            seg2:            NonZeroU8::new(2).unwrap(),       // 2 tq → 1+5+2 = 8 tq = 5Mbps
            sync_jump_width: NonZeroU8::new(2).unwrap(),
        });

    can.set_config(config);

    //let mut can = can.into_internal_loopback_mode();
    #[allow(unused_mut)]
    let mut can = can.into_normal_mode();
    let (mut tx, mut rx, _props) = can.split();
    info!("CAN up and running, waiting for messages");
    #[allow(unused_mut)]
    let mut flash = flash::Flash::new_blocking(peripherals.FLASH).into_blocking_regions();

    let this_node_id = this_node as u16;

    let mut chunk_address: u32 = 0;
    let mut chunk_size: u8;

    let mut f = flash.bank1_region;

    loop {
        match rx.read_fd().await {
            Ok(message) => {
                let (rx_frame, _ts) = message.parts();

                if let Id::Extended(id) = rx_frame.id() {
                    let raw_id = id.as_raw();
                    let can_msg = parse_can_id(raw_id);
                    info!("CAN msg: target={:X} command={:X}", can_msg.target, can_msg.command);
                    if can_msg.target == this_node_id {
                        /* this is a command for us! */
                        let data = rx_frame.data();

                        if let Ok(command) = BootloaderCommand::try_from(can_msg.command) {
                            match command {
                                BootloaderCommand::Ping => {
                                    info!("Command Ping {:X} received", BootloaderCommand::Ping as u16);
                                }
                                BootloaderCommand::Erase => {
                                    unwrap!(f.blocking_erase(0x8000, 0x80000));
                                    info!("Erase Complete");
                                }
                                BootloaderCommand::AddressAndSize => {
                                    if data.len() >= 5 {
                                        chunk_address = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
                                        chunk_size = data[4];
                                        info!("Prepared for Write at 0x{:08X} (Size: {})", chunk_address, chunk_size);
                                    }
                                }
                                BootloaderCommand::Write => {
                                    if data.len() % 16 != 0 {
                                        error!("Data length {} is not 16-byte aligned!", data.len());
                                    } else if chunk_address >= 0x0800_8000 {
                                        let offset = chunk_address - 0x0800_0000;
                                        // 1. Write to flash
                                        unwrap!(f.blocking_write(offset, data));
                                        info!("Wrote {} bytes to {:x}", data.len(), chunk_address);

                                        // 2. Prepare the WriteOk payload [Addr0, Addr1, Addr2, Addr3, Size]
                                        let mut payload = [0u8; 5];
                                        payload[0..4].copy_from_slice(&chunk_address.to_be_bytes());
                                        payload[4] = data.len() as u8;

                                        let reply_id = DfrCanId::new(
                                            1,
                                            can_msg.source,
                                            BootloaderCommand::WriteOk.into(),
                                            this_node_id
                                        ).unwrap();

                                        // 4. Create the FdFrame and send it
                                        let tx_frame = embassy_stm32::can::frame::FdFrame::new_extended(reply_id.to_raw_id(), &payload).unwrap();
                                        tx.write_fd(&tx_frame).await;

                                        info!("Sent WriteOk ACK for 0x{:08X}", chunk_address);

                                        chunk_address += data.len() as u32;
                                    }
                                }
                                BootloaderCommand::Jump => {
                                    info!("Jumping to application...");
                                }
                                _ => warn!("Unhandled command 0x{:04X}", can_msg.command),
                            }
                        } else {
                            warn!("Unknown command ID 0x{:x}", raw_id);
                        }
                    }
                }
            }
            Err(e) => error!("CAN read error: {}", e),
        }
    }
}
