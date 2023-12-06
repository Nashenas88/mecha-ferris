#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[cfg(feature = "pairing")]
use crate::bonder::Bonder;
use bluetooth_comms::log;
use command::{Command, CommandUpdate, StateManager};
use communication::{I2cRequest, I2cResponse, COMMS_ADDR};
use core::cell::RefCell;
use core::mem;
#[cfg(feature = "defmt")]
use defmt_rtt as _; // global logger
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Input, Pull};
use embassy_nrf::gpiote::{InputChannel, InputChannelPolarity};
use embassy_nrf::interrupt::{self, Interrupt, InterruptExt, Priority};
use embassy_nrf::peripherals::TWISPI0;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{self as _, bind_interrupts}; // embassy-time impl
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::mutex::Mutex;
use embassy_time::{with_timeout, Duration, TimeoutError};
use futures::Future;
use nrf_softdevice as _; // critical section impl
use nrf_softdevice::ble::{peripheral, Connection};
use nrf_softdevice::{raw, Softdevice};
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use panic_probe as _;
use server::Server;
use state::{CalibrationDto, JointDto};
use static_cell::StaticCell;

use crate::services::{
    BatteryService, BatteryServiceEvent, ControllerService, ControllerServiceEvent,
};

pub(crate) type RobotState = state::RobotState<JointDto, NUM_SERVOS_PER_LEG, NUM_LEGS>;

#[cfg(feature = "pairing")]
mod bonder;
mod command;
mod consts;
mod server;
mod services;

pub(crate) const NUM_SERVOS_PER_LEG: usize = 3;
pub(crate) const NUM_LEGS: usize = 6;

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

type CriticalChannel<T> = Channel<CriticalSectionRawMutex, T, 1>;
type CriticalReceiver<T> = Receiver<'static, CriticalSectionRawMutex, T, 1>;
type CriticalSender<T> = Sender<'static, CriticalSectionRawMutex, T, 1>;

trait TimeoutExt: Future {
    type Success;
    type Error;
    async fn timeout(self, timeout: Duration) -> Result<Self::Success, Self::Error>;
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum CommsError {
    Timeout(TimeoutError),
    I2c(twim::Error),
    Postcard(postcard::Error),
}

impl From<TimeoutError> for CommsError {
    fn from(e: TimeoutError) -> Self {
        Self::Timeout(e)
    }
}

impl From<twim::Error> for CommsError {
    fn from(e: twim::Error) -> Self {
        Self::I2c(e)
    }
}

impl From<postcard::Error> for CommsError {
    fn from(e: postcard::Error) -> Self {
        Self::Postcard(e)
    }
}

impl<T, U, E> TimeoutExt for T
where
    T: Future<Output = Result<U, E>>,
    E: Into<CommsError>,
{
    type Success = U;
    type Error = CommsError;
    async fn timeout(self, timeout: Duration) -> Result<U, CommsError> {
        with_timeout(timeout, self)
            .await
            .map_err(|e| e.into())
            .and_then(|r| r.map_err(|e| e.into()))
    }
}

const I2C_TIMEOUT_DURATION: Duration = Duration::from_millis(10_000);
const BUF_LEN: usize = 32;
#[embassy_executor::task]
async fn robot_comm(
    mut i2c: Twim<'static, TWISPI0>,
    request_rx: CriticalReceiver<I2cRequest>,
    response_tx: CriticalSender<Result<I2cResponse, CommsError>>,
) {
    let mut buffer = [0; BUF_LEN];
    loop {
        let i2c_message = request_rx.receive().await;
        if let Some(result) = handle_message(i2c_message, &mut i2c, &mut buffer)
            .await
            .transpose()
        {
            response_tx.send(result).await;
        }
    }
}

async fn handle_message(
    i2c_message: I2cRequest,
    i2c: &mut Twim<'static, TWISPI0>,
    buffer: &mut [u8],
) -> Result<Option<I2cResponse>, CommsError> {
    {
        let buf = postcard::to_slice(&i2c_message, buffer).unwrap();
        if buf.len() > BUF_LEN {
            log::panic!(
                "Message too large to send to MCU: {} for req: {:?}",
                buf.len(),
                i2c_message
            );
        }
        // Write the size of the message first.
        if let Err(e) = i2c
            .write(COMMS_ADDR as u8, &[buf.len() as u8])
            .timeout(I2C_TIMEOUT_DURATION)
            .await
        {
            log::error!(
                "Unable to write message size to MCU {} for {:?}: {:?}",
                buf.len(),
                i2c_message,
                e
            );
            return Err(e);
        }
        // Then send the message.
        if let Err(e) = i2c
            .write(COMMS_ADDR as u8, buf)
            .timeout(I2C_TIMEOUT_DURATION)
            .await
        {
            log::error!("Unable to write message to MCU {:?}: {:?}", i2c_message, e);
            return Err(e);
        }
    }

    if i2c_message.is_get() {
        // Read size of response first.
        if let Err(e) = i2c
            .read(COMMS_ADDR as u8, &mut buffer[..1])
            .timeout(I2C_TIMEOUT_DURATION)
            .await
        {
            log::error!(
                "Unable to read message response size from MCU for {:?}: {:?}",
                i2c_message,
                e
            );
            return Err(e);
        }
        let size = buffer[0] as usize;
        log::info!("Response size is {}", size);
        // Limit buffer size for read and deserialization.
        let buffer = &mut buffer[..size];
        if let Err(e) = i2c
            .read(COMMS_ADDR as u8, buffer)
            .timeout(I2C_TIMEOUT_DURATION)
            .await
        {
            log::error!("Unable to read message from MCU: {:?}", e);
            return Err(e);
        }
        log::info!("Received message from MCU: {:?}", buffer);
        let result = match postcard::from_bytes::<I2cResponse>(buffer) {
            Ok(result) => result,
            Err(e) => {
                log::error!(
                    "Unable to deserialize message from MCU for {:?}: {:?}",
                    i2c_message,
                    e
                );
                return Err(e.into());
            }
        };
        Ok(Some(result))
    } else {
        Ok(None)
    }
}

bind_interrupts!(struct Irqs {
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => twim::InterruptHandler<TWISPI0>;
});

const fn make_default_joints() -> [[state::JointDto; NUM_SERVOS_PER_LEG]; NUM_LEGS] {
    let mut joints = [[state::JointDto {
        servo_idx: 0,
        cal: [CalibrationDto {
            kind: state::CalibrationKind::Home,
            pulse_ms: 1500.0,
        }; 4],
    }; NUM_SERVOS_PER_LEG]; NUM_LEGS];

    let mut leg = 0;
    while leg < NUM_LEGS {
        let mut joint = 0;
        while joint < NUM_SERVOS_PER_LEG {
            joints[leg][joint].servo_idx = (leg * NUM_SERVOS_PER_LEG + joint) as u8;
            let mut kind = 0;
            while kind < (if joint == 2 { 4 } else { 3 }) {
                joints[leg][joint].cal[kind].kind = match kind {
                    0 => state::CalibrationKind::Min,
                    1 => state::CalibrationKind::Max,
                    2 => state::CalibrationKind::Mid,
                    3 => state::CalibrationKind::Home,
                    _ => unreachable!(),
                };
                kind += 1;
            }
            joint += 1;
        }
        leg += 1;
    }

    joints
}

static REQUEST_CHANNEL: StaticCell<CriticalChannel<I2cRequest>> = StaticCell::new();
static RESPONSE_CHANNEL: StaticCell<CriticalChannel<Result<I2cResponse, CommsError>>> =
    StaticCell::new();
static STATE_MANAGER: StaticCell<RefCell<StateManager>> = StaticCell::new();
static ROBOT_STATE: Mutex<CriticalSectionRawMutex, RobotState> =
    Mutex::new(RobotState::new(make_default_joints()));
static SERVER: StaticCell<Server> = StaticCell::new();
static CONNECTION: StaticCell<RefCell<Option<Connection>>> = StaticCell::new();
static COMMAND_UPDATE: StaticCell<RefCell<CommandUpdate>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    log::info!("Started");
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);
    let request_channel = REQUEST_CHANNEL.init(Channel::new());
    let response_channel = RESPONSE_CHANNEL.init(Channel::new());

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
        common_vs_uuid: Some(raw::ble_common_cfg_vs_uuid_t { vs_uuid_count: 15 }),
        ..Default::default()
    };

    log::info!("Enabling softdevice");
    let sd = Softdevice::enable(&config);
    log::info!("constructing server");
    let server = log::unwrap!(Server::new(sd));
    log::info!("spawning softdevice task");
    log::unwrap!(spawner.spawn(softdevice_task(sd)));
    let server = &*SERVER.init(server);
    let connection = CONNECTION.init(RefCell::new(None));
    let state_manager = STATE_MANAGER.init(RefCell::new(StateManager::new()));
    let command_update = COMMAND_UPDATE.init(RefCell::new(CommandUpdate::new()));

    #[rustfmt::skip]
    let adv_data = &[
        // Size, Type, Data...
        // 2 bytes, raw::BLE_GAP_AD_TYPE_FLAGS as u8, LowEnergyOnlyGeneralDiscoveryMode
        0x02, 0x01, raw::BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE as u8,
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

    #[cfg(feature = "pairing")]
    static BONDER: StaticCell<Bonder> = StaticCell::new();
    #[cfg(feature = "pairing")]
    let bonder = BONDER.init(Bonder::default());

    log::info!("Setting up i2c");
    let config = twim::Config::default();
    Interrupt::SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0.set_priority(interrupt::Priority::P3);
    let i2c = Twim::new(p.TWISPI0, Irqs, p.P0_16, p.P0_14, config);

    log::info!("Spawning robot comms task");
    let robot_comms_token = robot_comm(i2c, request_channel.receiver(), response_channel.sender());
    if let Err(e) = spawner.spawn(robot_comms_token) {
        log::error!("Failed to spawn robot comms task: {:?}", e);
    }

    let ble_loop = async {
        loop {
            let config = peripheral::Config::default();
            let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
                adv_data,
                scan_data,
            };
            log::info!("Advertising...");
            #[cfg(feature = "pairing")]
            let conn = log::unwrap!(peripheral::advertise_pairable(sd, adv, &config, bonder).await);
            #[cfg(not(feature = "pairing"))]
            let conn = log::unwrap!(peripheral::advertise_connectable(sd, adv, &config).await);
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
                .run(
                    connection,
                    state_manager,
                    command_update,
                    request_channel,
                    response_channel,
                    spawner,
                )
                .await;
            loop {
                if let Ok(mut connection) = connection.try_borrow_mut() {
                    *connection = None;
                    break;
                }
            }

            log::error!("gatt_server run exited with error: {:?}", res);
        }
    };

    let event_ch = InputChannel::new(
        p.GPIOTE_CH0,
        Input::new(p.P0_27, Pull::Up),
        InputChannelPolarity::HiToLo,
    );
    let robot_irq = async {
        loop {
            event_ch.wait().await;
            log::info!("Robot IRQ");
        }
    };

    futures::join!(ble_loop, robot_irq);
}
