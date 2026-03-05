#![no_std]
#![no_main]

mod protocol;
use protocol::*;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::peripherals::*;
use embassy_stm32::{Config, bind_interrupts, can, can::filter::*, flash, rcc, uid};
use embedded_can::Id;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    config.rcc.hse = Some(rcc::Hse {
        freq: embassy_stm32::time::Hertz(25_000_000),
        mode: rcc::HseMode::Oscillator,
    });
    config.rcc.mux.fdcan12sel = rcc::mux::Fdcansel::HSE;
    let peripherals = embassy_stm32::init(config);

    let this_node = match uid::uid_hex() {
        "001E005F3333510132313831" => CanDevices::Nuc1,
        "004500243333510132313831" => CanDevices::Nuc2,
        _ => CanDevices::UNKNOWN,
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
    can.set_bitrate(1_000_000);
    can.set_fd_data_bitrate(5_000_000, true);

    let mut can = can.into_internal_loopback_mode();
    //let mut can = can.into_normal_mode();
    let (mut tx, mut rx, _props) = can.split();
    info!("CAN up and running, waiting for messages");

    let mut flash = flash::Flash::new_blocking(peripherals.FLASH).into_blocking_regions();

    let this_node_id = this_node as u16;

    #[allow(non_snake_case)]
    let mut chunkAddress: u32 = 0;
    #[allow(non_snake_case)]
    let mut chunkSize: u8 = 0;

    let mut f = flash.bank1_region;

    loop {
        match rx.read_fd().await {
            Ok(message) => {
                let (rx_frame, _ts) = message.parts();

                if let Id::Extended(id) = rx_frame.id() {
                    let raw_id = id.as_raw();
                    let can_msg = parse_can_id(raw_id);

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
                                        chunkAddress = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
                                        chunkSize = data[4];
                                        info!("Prepared for Write at 0x{:08X} (Size: {})", chunkAddress, chunkSize);
                                    }
                                }
                                BootloaderCommand::Write => {
                                    let offset = chunkAddress - 0x0800_0000;

                                    if data.len() % 16 != 0 {
                                        error!("Data length {} is not 16-byte aligned!", data.len());
                                    } else if chunkAddress >= 0x0800_8000 {
                                        // 1. Write to flash
                                        unwrap!(f.blocking_write(offset, data));
                                        info!("Wrote {} bytes to {:x}", data.len(), chunkAddress);

                                        // 2. Prepare the WriteOk payload [Addr0, Addr1, Addr2, Addr3, Size]
                                        let mut payload = [0u8; 5];
                                        payload[0..4].copy_from_slice(&chunkAddress.to_be_bytes());
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

                                        info!("Sent WriteOk ACK for 0x{:08X}", chunkAddress);

                                        chunkAddress += data.len() as u32;
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
