#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use command::{Command, CommandUpdate, StateManager};
use communication::{I2cRequest, Serialize, COMMS_ADDR};
use core::cell::RefCell;
use core::mem;
use defmt::unwrap;
use defmt_rtt as _; // global logger
use embassy_executor::Spawner;
use embassy_nrf::interrupt::{self, Interrupt, InterruptExt, Priority};
use embassy_nrf::peripherals::TWISPI0;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{self as _, bind_interrupts}; // embassy-time impl
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver};
use embassy_sync::mutex::Mutex;
use nrf_softdevice as _; // critical section impl
use nrf_softdevice::ble::{peripheral, Connection};
use nrf_softdevice::{raw, Softdevice};
use panic_probe as _;
use server::Server;
use state::RobotState;
use static_cell::StaticCell;

use crate::services::{
    BatteryService, BatteryServiceEvent, ControllerService, ControllerServiceEvent,
};

mod command;
mod consts;
mod log;
mod server;
mod services;
mod wrappers;

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

#[embassy_executor::task]
async fn robot_comm(
    mut i2c: Twim<'static, TWISPI0>,
    rx: Receiver<'static, CriticalSectionRawMutex, I2cRequest, 1>,
) {
    let mut buffer = [0; 16];
    loop {
        let i2c_message = rx.recv().await;
        let written = i2c_message.serialize(&mut buffer);
        buffer[written..].fill(0);
        if let Err(e) = i2c.write(COMMS_ADDR as u8, &buffer).await {
            log::error!("Unable to write message to MCU: {:?}", e);
        }
    }
}

bind_interrupts!(struct Irqs {
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => twim::InterruptHandler<TWISPI0>;
});

static CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, I2cRequest, 1>> = StaticCell::new();
static STATE_MANAGER: StaticCell<RefCell<StateManager>> = StaticCell::new();
static ROBOT_STATE: Mutex<CriticalSectionRawMutex, RobotState> = Mutex::new(RobotState::new());
static SERVER: StaticCell<Server> = StaticCell::new();
static CONNECTION: StaticCell<RefCell<Option<Connection>>> = StaticCell::new();
static COMMAND_UPDATE: StaticCell<RefCell<CommandUpdate>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);
    let channel = CHANNEL.init(Channel::new());

    let config = nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 16,
            rc_temp_ctiv: 2,
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 6,
            event_length: 24,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 256 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: 32768,
        }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 3,
            central_role_count: 3,
            central_sec_count: 0,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: b"MechaFerris" as *const u8 as _,
            current_len: 11,
            max_len: 11,
            write_perm: unsafe { mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                raw::BLE_GATTS_VLOC_STACK as u8,
            ),
        }),
        ..Default::default()
    };

    let sd = Softdevice::enable(&config);
    let server = unwrap!(Server::new(sd));
    unwrap!(spawner.spawn(softdevice_task(sd)));
    let server = &*SERVER.init(server);
    let connection = CONNECTION.init(RefCell::new(None));
    let state_manager = STATE_MANAGER.init(RefCell::new(StateManager::new()));
    let command_update = COMMAND_UPDATE.init(RefCell::new(CommandUpdate::new()));

    #[rustfmt::skip]
    let adv_data = &[
        // Size, Type, Data...
        // 2 bytes, raw::BLE_GAP_AD_TYPE_FLAGS as u8, LowEnergyOnlyGeneralDiscoveryMode
        0x02, 0x01, raw::BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE as u8,
        // // 3 bytes, raw::BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE as u8,
        // // -- Little Endian of 0x1809 - Health Thermometer Service
        // 0x03, raw::BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE as u8, 0x09, 0x18,
        // 17 bytes , 128-bit uuid complete, customer service a6772cc6-302c-4de3-abcd-ce702762881e
        // 0x11, raw::BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE as u8, 0x1e, 0x88, 0x62, 0x27, 0x70, 0xce, 0xcd, 0xab, 0xe3, 0x4d, 0x2c, 0x30, 0xc6, 0x2c, 0x77, 0xa6,
        // 12 bytes, raw::BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME as u8, b"MechaFerris"
        0x0c, raw::BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME as u8, b'M', b'e', b'c', b'h', b'a', b'F', b'e', b'r', b'r', b'i', b's'
    ];
    #[rustfmt::skip]
    let scan_data = &[
        // 17 bytes, 128-bit uuid complete, customer service a6772cc6-302c-4de3-abcd-ce702762881e
        0x11, raw::BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE as u8, 0x1e, 0x88, 0x62, 0x27, 0x70, 0xce, 0xcd, 0xab, 0xe3, 0x4d, 0x2c, 0x30, 0xc6, 0x2c, 0x77, 0xa6,
    ];

    let config = twim::Config::default();
    {
        let irq = unsafe { interrupt::SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0::steal() };
        irq.set_priority(interrupt::Priority::P3);
    }
    let i2c = Twim::new(p.TWISPI0, Irqs, p.P0_16, p.P0_14, config);

    let robot_comms_token = robot_comm(i2c, channel.receiver());
    if let Err(e) = spawner.spawn(robot_comms_token) {
        log::error!("Failed to spawn robot comms task: {:?}", e);
    }

    loop {
        let config = peripheral::Config::default();
        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data,
            scan_data,
        };
        let conn = unwrap!(peripheral::advertise_connectable(sd, adv, &config).await);
        loop {
            if let Ok(mut connection) = connection.try_borrow_mut() {
                *connection = Some(conn);
                break;
            }
        }

        log::info!("advertising done!");

        // Run the GATT server on the connection. This returns when the connection gets disconnected.
        //
        // Event enums (ServerEvents) are generated by nrf_softdevice::gatt_server
        // proc macro when applied to the Server struct above

        let res = server
            .run(connection, state_manager, command_update, channel, spawner)
            .await;
        loop {
            if let Ok(mut connection) = connection.try_borrow_mut() {
                *connection = None;
                break;
            }
        }

        if let Err(e) = res {
            log::error!("gatt_server run exited with error: {:?}", e);
        }
    }
}
