#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;
use core::mem;

use communication::{I2cRequest, I2cRequestField, I2cRequestOp, Serialize, COMMS_ADDR};
use defmt::unwrap;
use defmt_rtt as _; // global logger
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::peripherals::TWISPI0;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{self as _, bind_interrupts};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::mutex::Mutex;
use nalgebra::{Quaternion, Translation3, UnitQuaternion, Vector3};
// embassy-time impl
use nrf_softdevice as _;
use nrf_softdevice::ble::gatt_server::{IndicateValueError, NotifyValueError};
// critical section impl
use nrf_softdevice::ble::{gatt_server, peripheral, Connection, FixedGattValue};
use nrf_softdevice::{raw, Softdevice};
use panic_probe as _;
use scapegoat::SgSet;
use state::{RobotState, StateMachine};
use static_cell::StaticCell;

const MIN_SPEED: f32 = 0.0;
const MAX_SPEED: f32 = 100.0;
const MIN_X: f32 = 0.0;
const MAX_X: f32 = 1000.0;
const MIN_Y: f32 = 0.0;
const MAX_Y: f32 = 1000.0;
const MIN_Z: f32 = 0.0;
const MAX_Z: f32 = 1000.0;
const MIN_YAW: f32 = 0.0;
const MAX_YAW: f32 = 1000.0;
const MIN_PITCH: f32 = 0.0;
const MAX_PITCH: f32 = 1000.0;
const MIN_ROLL: f32 = 0.0;
const MAX_ROLL: f32 = 1000.0;
const MIN_AVEL: f32 = -100.0;
const MAX_AVEL: f32 = -100.0;
const MIN_LRAD: f32 = 75.0;
const MAX_LRAD: f32 = 200.0;
const MIN_UPIVAL: u32 = 0;
const MAX_UPIVAL: u32 = 10000;

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

#[nrf_softdevice::gatt_service(uuid = "180f")]
struct BatteryService {
    #[characteristic(uuid = "2a19", read, notify)]
    battery_level: u8,
}

#[derive(Copy, Clone)]
struct Translation(Translation3<f32>);

#[derive(Copy, Clone)]
struct Vector(Vector3<f32>);

#[derive(Copy, Clone)]
struct UQuaternion(UnitQuaternion<f32>);

impl FixedGattValue for Translation {
    const SIZE: usize = core::mem::size_of::<f32>() * 3;

    fn from_gatt(data: &[u8]) -> Self {
        let data: &[f32] = unsafe { core::slice::from_raw_parts(data.as_ptr() as *const f32, 3) };
        Self(Translation3::new(data[0], data[1], data[2]))
    }

    fn to_gatt(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                self.0.vector.as_slice().as_ptr() as *const u8,
                core::mem::size_of::<f32>() * 3,
            )
        }
    }
}

impl FixedGattValue for Vector {
    const SIZE: usize = core::mem::size_of::<f32>() * 3;

    fn from_gatt(data: &[u8]) -> Self {
        let data: &[f32] = unsafe { core::slice::from_raw_parts(data.as_ptr() as *const f32, 3) };
        Self(Vector3::new(data[0], data[1], data[2]))
    }

    fn to_gatt(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                self.0.data.as_slice().as_ptr() as *const u8,
                core::mem::size_of::<f32>() * 3,
            )
        }
    }
}

impl FixedGattValue for UQuaternion {
    const SIZE: usize = core::mem::size_of::<f32>() * 4;

    fn from_gatt(data: &[u8]) -> Self {
        let data: &[f32] = unsafe { core::slice::from_raw_parts(data.as_ptr() as *const f32, 4) };
        Self(UnitQuaternion::new_normalize(Quaternion::new(
            data[0], data[1], data[2], data[3],
        )))
    }

    fn to_gatt(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                self.0.as_ref().coords.as_slice().as_ptr() as *const u8,
                core::mem::size_of::<f32>() * 4,
            )
        }
    }
}

struct SM(StateMachine);
impl FixedGattValue for SM {
    const SIZE: usize = 1;

    fn from_gatt(data: &[u8]) -> Self {
        let Ok(sm) = u8::from_gatt(data).try_into() else {
            panic!("bad data for StateMachine");
        };
        SM(sm)
    }

    fn to_gatt(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self as *const Self as *const u8, Self::SIZE) }
    }
}

#[nrf_softdevice::gatt_service(uuid = "77144c32-0ed7-469b-a3d3-0f2c9157333a")]
struct ControllerService {
    #[characteristic(uuid = "b0115f52-c6fd-4ea4-94cf-21626b9e3469", write)]
    sync: bool,
    #[characteristic(
        uuid = "f125c904-a0e2-4885-817c-55d7463630db",
        read,
        write,
        notify,
        indicate
    )]
    state: SM,
    #[characteristic(
        uuid = "7566bd93-3712-4850-8868-88ce30f6acc1",
        read,
        write,
        notify,
        indicate
    )]
    speed: f32,
    #[characteristic(
        uuid = "6be80625-e69b-418e-bf01-9abc617cdd9f",
        read,
        write,
        notify,
        indicate
    )]
    angular_velocity: f32,
    #[characteristic(
        uuid = "77d6f220-7057-4dc6-8746-8a23b06e53d6",
        read,
        write,
        notify,
        indicate
    )]
    motion_vector: Vector,
    #[characteristic(
        uuid = "cde4ce10-edc2-44af-bd14-60865d30f2b6",
        read,
        write,
        notify,
        indicate
    )]
    body_translation: Translation,
    #[characteristic(
        uuid = "ccfb948a-3421-47ed-a0c0-b84f7d307027",
        read,
        write,
        notify,
        indicate
    )]
    body_rotation: UQuaternion,
    #[characteristic(
        uuid = "2735f1d0-b944-4efe-960c-10380d061052",
        read,
        write,
        notify,
        indicate
    )]
    leg_radius: f32,
    #[characteristic(
        uuid = "d007632f-10e5-427a-b158-482aeb48b90e",
        read,
        write,
        notify,
        indicate
    )]
    battery_update_interval_ms: u32,
}

#[nrf_softdevice::gatt_server]
struct Server {
    bas: BatteryService,
    controller: ControllerService,
}

struct StateManager {
    should_update: SgSet<I2cRequestOp, 16>,
}

impl StateManager {
    fn new() -> Self {
        Self {
            should_update: SgSet::new(),
        }
    }
}

async fn handle_command(
    robot_state: &mut RobotState,
    state_manager: &mut StateManager,
    sender: Sender<'static, CriticalSectionRawMutex, I2cRequest, 1>,
    command: Command,
) -> Result<(), CommandErr> {
    match command {
        Command::Sync => {
            for op in state_manager.should_update.iter() {
                let i2c_message = match op {
                    I2cRequestOp::ChangeState => I2cRequest::ChangeState(robot_state.state_machine),
                    I2cRequestOp::Set(field) => match field {
                        I2cRequestField::Speed => I2cRequest::SetSpeed(robot_state.speed),
                        I2cRequestField::AngularVelocity => {
                            I2cRequest::SetAngularVelocity(robot_state.angular_velocity)
                        }
                        I2cRequestField::MoveVector => I2cRequest::SetMoveVector(
                            robot_state.motion_vector.x,
                            robot_state.motion_vector.y,
                            robot_state.motion_vector.z,
                        ),
                        I2cRequestField::BodyTranslation => I2cRequest::SetBodyTranslation(
                            robot_state.body_translation.x,
                            robot_state.body_translation.y,
                            robot_state.body_translation.z,
                        ),
                        I2cRequestField::BodyRotation => {
                            let (r, p, y) = robot_state.body_rotation.euler_angles();
                            I2cRequest::SetBodyRotation(r, p, y)
                        }
                        I2cRequestField::LegRadius => {
                            I2cRequest::SetLegRadius(robot_state.leg_radius)
                        }
                    },
                    I2cRequestOp::SetBatteryUpdateInterval => {
                        I2cRequest::SetBatteryUpdateInterval(robot_state.battery_update_interval_ms)
                    }
                    _ => unreachable!(),
                };
                sender.send(i2c_message).await;
            }
            // I2cRequestOp::Get(field) => match field {
            //     I2cRequestField::Speed => todo!(),
            //     I2cRequestField::AngularVelocity => todo!(),
            //     I2cRequestField::MoveVector => todo!(),
            //     I2cRequestField::BodyTranslation => todo!(),
            //     I2cRequestField::BodyRotation => todo!(),
            //     I2cRequestField::LegRadius => todo!(),
            // },
            // I2cRequestOp::GetBatteryLevel => todo!(),

            state_manager.should_update.clear();
        }
        Command::ChangeState(sm) => {
            robot_state.state_machine = sm;
            let _ = state_manager
                .should_update
                .insert(I2cRequestOp::ChangeState);
        }
        Command::SetSpeed(val) => {
            if (MIN_SPEED..=MAX_SPEED).contains(&val) {
                robot_state.speed = val;
                let _ = state_manager
                    .should_update
                    .insert(I2cRequestOp::Set(I2cRequestField::Speed));
            } else {
                return Err(CommandErr::InvalidFloat);
            }
        }
        Command::SetBodyTranslation(val) => {
            if (MIN_X..=MAX_X).contains(&val.x)
                && (MIN_Y..=MAX_Y).contains(&val.y)
                && (MIN_Z..=MAX_Z).contains(&val.z)
            {
                robot_state.body_translation = val;
                let _ = state_manager
                    .should_update
                    .insert(I2cRequestOp::Set(I2cRequestField::BodyTranslation));
            } else {
                return Err(CommandErr::InvalidFloat);
            }
        }
        Command::SetBodyRotation(val) => {
            let (roll, pitch, yaw) = val.euler_angles();
            if (MIN_ROLL..=MAX_ROLL).contains(&roll)
                && (MIN_PITCH..=MAX_PITCH).contains(&pitch)
                && (MIN_YAW..=MAX_YAW).contains(&yaw)
            {
                robot_state.body_rotation = val;
                let _ = state_manager
                    .should_update
                    .insert(I2cRequestOp::Set(I2cRequestField::BodyRotation));
            } else {
                return Err(CommandErr::InvalidFloat);
            }
        }
        Command::SetMotionVector(val) => {
            if (MIN_X..=MAX_X).contains(&val.x)
                && (MIN_Y..=MAX_Y).contains(&val.y)
                && (MIN_Z..=MAX_Z).contains(&val.z)
            {
                robot_state.motion_vector = val;
                let _ = state_manager
                    .should_update
                    .insert(I2cRequestOp::Set(I2cRequestField::MoveVector));
            } else {
                return Err(CommandErr::InvalidFloat);
            }
        }
        Command::SetAngularVelocity(val) => {
            if (MIN_AVEL..=MAX_AVEL).contains(&val) {
                robot_state.angular_velocity = val;
                let _ = state_manager
                    .should_update
                    .insert(I2cRequestOp::Set(I2cRequestField::AngularVelocity));
            } else {
                return Err(CommandErr::InvalidFloat);
            }
        }
        Command::SetLegRadius(val) => {
            if (MIN_LRAD..=MAX_LRAD).contains(&val) {
                robot_state.leg_radius = val;
                let _ = state_manager
                    .should_update
                    .insert(I2cRequestOp::Set(I2cRequestField::LegRadius));
            } else {
                return Err(CommandErr::InvalidFloat);
            }
        }
        Command::SetBatteryUpdateInterval(val) => {
            if (MIN_UPIVAL..=MAX_UPIVAL).contains(&val) {
                robot_state.battery_update_interval_ms = val;
                let _ = state_manager
                    .should_update
                    .insert(I2cRequestOp::SetBatteryUpdateInterval);
            } else {
                return Err(CommandErr::InvalidFloat);
            }
        }
    }

    Ok(())
}

enum CommandErr {
    InvalidFloat,
}

#[derive(Copy, Clone)]
enum Command {
    Sync,
    ChangeState(StateMachine),
    SetSpeed(f32),
    SetBodyTranslation(Translation3<f32>),
    SetBodyRotation(UnitQuaternion<f32>),
    SetMotionVector(Vector3<f32>),
    SetAngularVelocity(f32),
    SetLegRadius(f32),
    SetBatteryUpdateInterval(u32),
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
            defmt::warn!("Unable to write message to MCU: {:?}", e);
        }
    }
}

struct CommandUpdate {
    state: UpdateState,
    speed: UpdateState,
    body_translation: UpdateState,
    body_rotation: UpdateState,
    motion_vector: UpdateState,
    angular_velocity: UpdateState,
    leg_radius: UpdateState,
    battery_interval: UpdateState,
}

impl CommandUpdate {
    fn new() -> Self {
        Self {
            state: UpdateState::new(),
            speed: UpdateState::new(),
            body_translation: UpdateState::new(),
            body_rotation: UpdateState::new(),
            motion_vector: UpdateState::new(),
            angular_velocity: UpdateState::new(),
            leg_radius: UpdateState::new(),
            battery_interval: UpdateState::new(),
        }
    }
}

#[derive(Copy, Clone)]
struct UpdateState {
    kind: Option<UpdateKind>,
}

impl UpdateState {
    fn new() -> Self {
        Self { kind: None }
    }
}

#[derive(Copy, Clone)]
enum UpdateKind {
    Notify,
    Indicate,
}

impl UpdateKind {
    fn from_cccd(notification: bool, indicate: bool) -> Option<UpdateKind> {
        match (notification, indicate) {
            (true, _) => Some(UpdateKind::Notify),
            (_, true) => Some(UpdateKind::Indicate),
            _ => None,
        }
    }
}

impl CommandUpdate {
    fn get(&self, command: Command) -> Option<UpdateKind> {
        match command {
            Command::Sync => None,
            Command::ChangeState(_) => self.state.kind,
            Command::SetSpeed(_) => self.speed.kind,
            Command::SetBodyTranslation(_) => self.body_translation.kind,
            Command::SetBodyRotation(_) => self.body_rotation.kind,
            Command::SetMotionVector(_) => self.motion_vector.kind,
            Command::SetAngularVelocity(_) => self.angular_velocity.kind,
            Command::SetLegRadius(_) => self.leg_radius.kind,
            Command::SetBatteryUpdateInterval(_) => self.battery_interval.kind,
        }
    }
}

#[derive(defmt::Format)]
enum UpdateError {
    Notify(NotifyValueError),
    Indicate(IndicateValueError),
}

impl From<NotifyValueError> for UpdateError {
    fn from(err: NotifyValueError) -> Self {
        Self::Notify(err)
    }
}

impl From<IndicateValueError> for UpdateError {
    fn from(err: IndicateValueError) -> Self {
        Self::Indicate(err)
    }
}

fn update(
    server: &'static Server,
    conn: &Connection,
    command_update: &CommandUpdate,
    command: Command,
    robot_state: &RobotState,
) -> Result<(), UpdateError> {
    match (command, command_update.get(command)) {
        (Command::Sync, _) => {}
        (Command::ChangeState(_), Some(UpdateKind::Notify)) => server
            .controller
            .state_notify(conn, &SM(robot_state.state_machine))?,
        (Command::ChangeState(_), Some(UpdateKind::Indicate)) => server
            .controller
            .state_indicate(conn, &SM(robot_state.state_machine))?,
        (Command::SetSpeed(_), Some(UpdateKind::Notify)) => {
            server.controller.speed_notify(conn, &robot_state.speed)?
        }
        (Command::SetSpeed(_), Some(UpdateKind::Indicate)) => {
            server.controller.speed_indicate(conn, &robot_state.speed)?
        }
        (Command::SetBodyTranslation(_), Some(UpdateKind::Notify)) => server
            .controller
            .body_translation_notify(conn, &Translation(robot_state.body_translation))?,
        (Command::SetBodyTranslation(_), Some(UpdateKind::Indicate)) => server
            .controller
            .body_translation_indicate(conn, &Translation(robot_state.body_translation))?,
        (Command::SetBodyRotation(_), Some(UpdateKind::Notify)) => server
            .controller
            .body_rotation_notify(conn, &UQuaternion(robot_state.body_rotation))?,
        (Command::SetBodyRotation(_), Some(UpdateKind::Indicate)) => server
            .controller
            .body_rotation_indicate(conn, &UQuaternion(robot_state.body_rotation))?,
        (Command::SetMotionVector(_), Some(UpdateKind::Notify)) => server
            .controller
            .motion_vector_notify(conn, &Vector(robot_state.motion_vector))?,
        (Command::SetMotionVector(_), Some(UpdateKind::Indicate)) => server
            .controller
            .motion_vector_indicate(conn, &Vector(robot_state.motion_vector))?,
        (Command::SetAngularVelocity(_), Some(UpdateKind::Notify)) => server
            .controller
            .angular_velocity_notify(conn, &robot_state.angular_velocity)?,
        (Command::SetAngularVelocity(_), Some(UpdateKind::Indicate)) => server
            .controller
            .angular_velocity_indicate(conn, &robot_state.angular_velocity)?,
        (Command::SetLegRadius(_), Some(UpdateKind::Notify)) => server
            .controller
            .leg_radius_notify(conn, &robot_state.leg_radius)?,
        (Command::SetLegRadius(_), Some(UpdateKind::Indicate)) => server
            .controller
            .leg_radius_indicate(conn, &robot_state.leg_radius)?,
        _ => todo!(),
    }
    Ok(())
}

#[embassy_executor::task]
async fn command_task(
    server: &'static Server,
    conn: &'static RefCell<Option<Connection>>,
    state_manager: &'static RefCell<StateManager>,
    sender: Sender<'static, CriticalSectionRawMutex, I2cRequest, 1>,
    command: Command,
    command_update: &'static RefCell<CommandUpdate>,
) {
    let mut robot_state = ROBOT_STATE.lock().await;
    let res = handle_command(
        &mut robot_state,
        &mut state_manager.borrow_mut(),
        sender,
        command,
    )
    .await;
    let robot_state = &*robot_state;
    let mut word = [0; 8];
    let _ = Serializable(res).serialize(&mut word);
    if let Ok(conn) = conn.try_borrow() {
        if let Some(conn) = &*conn {
            let res: Result<(), UpdateError> =
                update(server, conn, &command_update.borrow(), command, robot_state);
            if let Err(e) = res {
                defmt::info!("send notification error: {:?}", e);
            }
        }
    }
}

struct Serializable(Result<(), CommandErr>);

impl Serialize for Serializable {
    fn serialize(self, buffer: &mut [u8]) -> usize {
        buffer[0] = match self.0 {
            Ok(()) => 0,
            Err(e) => match e {
                CommandErr::InvalidFloat => 1,
            },
        };
        1
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
    // fn main() {
    defmt::info!("Hello world!");
    let p = embassy_nrf::init(Default::default());

    let mut led = Output::new(p.P0_06, Level::High, OutputDrive::Standard);
    led.set_high();

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
            p_value: b"HelloRust" as *const u8 as _,
            current_len: 9,
            max_len: 9,
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
        0x11, raw::BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE as u8, 0x1e, 0x88, 0x62, 0x27, 0x70, 0xce, 0xcd, 0xab, 0xe3, 0x4d, 0x2c, 0x30, 0xc6, 0x2c, 0x77, 0xa6,
        // 10 bytes, raw::BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME as u8, b"MechaFerris"
        0x0c, raw::BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME as u8, b'M', b'e', b'c', b'h', b'a', b'F', b'e', b'r', b'r', b'i', b's'
    ];
    #[rustfmt::skip]
    let scan_data = &[
        // 17 bytes, 128-bit uuid complete, customer service a6772cc6-302c-4de3-abcd-ce702762881e
        0x11, raw::BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE as u8, 0x1e, 0x88, 0x62, 0x27, 0x70, 0xce, 0xcd, 0xab, 0xe3, 0x4d, 0x2c, 0x30, 0xc6, 0x2c, 0x77, 0xa6,
    ];

    let config = twim::Config::default();
    let i2c = Twim::new(p.TWISPI0, Irqs, p.P0_16, p.P0_24, config);

    let robot_comms_token = robot_comm(i2c, channel.receiver());
    if let Err(e) = spawner.spawn(robot_comms_token) {
        defmt::error!("Failed to spawn robot comms task: {:?}", e);
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

        defmt::info!("advertising done!");

        // Run the GATT server on the connection. This returns when the connection gets disconnected.
        //
        // Event enums (ServerEvents) are generated by nrf_softdevice::gatt_server
        // proc macro when applied to the Server struct above

        let res = gatt_server::run(connection.borrow().as_ref().unwrap(), server, |e| match e {
            ServerEvent::Bas(e) => match e {
                BatteryServiceEvent::BatteryLevelCccdWrite { notifications } => {
                    defmt::info!("battery notifications: {}", notifications);
                }
            },
            ServerEvent::Controller(e) => {
                let command = match e {
                    ControllerServiceEvent::SyncWrite(sync) if sync => Some(Command::Sync),
                    ControllerServiceEvent::SyncWrite(_) => None,
                    ControllerServiceEvent::StateWrite(SM(state_machine)) => {
                        Some(Command::ChangeState(state_machine))
                    }
                    ControllerServiceEvent::StateCccdWrite {
                        indications,
                        notifications,
                    } => {
                        command_update.borrow_mut().state.kind =
                            UpdateKind::from_cccd(notifications, indications);
                        None
                    }
                    ControllerServiceEvent::SpeedWrite(speed) => Some(Command::SetSpeed(speed)),
                    ControllerServiceEvent::SpeedCccdWrite {
                        indications,
                        notifications,
                    } => {
                        command_update.borrow_mut().speed.kind =
                            UpdateKind::from_cccd(notifications, indications);
                        None
                    }
                    ControllerServiceEvent::AngularVelocityWrite(angular_velocity) => {
                        Some(Command::SetAngularVelocity(angular_velocity))
                    }
                    ControllerServiceEvent::AngularVelocityCccdWrite {
                        indications,
                        notifications,
                    } => {
                        command_update.borrow_mut().angular_velocity.kind =
                            UpdateKind::from_cccd(notifications, indications);
                        None
                    }
                    ControllerServiceEvent::MotionVectorWrite(Vector(motion_vector)) => {
                        Some(Command::SetMotionVector(motion_vector))
                    }
                    ControllerServiceEvent::MotionVectorCccdWrite {
                        indications,
                        notifications,
                    } => {
                        command_update.borrow_mut().motion_vector.kind =
                            UpdateKind::from_cccd(notifications, indications);
                        None
                    }
                    ControllerServiceEvent::BodyTranslationWrite(Translation(body_translation)) => {
                        Some(Command::SetBodyTranslation(body_translation))
                    }
                    ControllerServiceEvent::BodyTranslationCccdWrite {
                        indications,
                        notifications,
                    } => {
                        command_update.borrow_mut().body_translation.kind =
                            UpdateKind::from_cccd(notifications, indications);
                        None
                    }
                    ControllerServiceEvent::BodyRotationWrite(UQuaternion(body_rotation)) => {
                        Some(Command::SetBodyRotation(body_rotation))
                    }
                    ControllerServiceEvent::BodyRotationCccdWrite {
                        indications,
                        notifications,
                    } => {
                        command_update.borrow_mut().body_rotation.kind =
                            UpdateKind::from_cccd(notifications, indications);
                        None
                    }
                    ControllerServiceEvent::LegRadiusWrite(leg_radius) => {
                        Some(Command::SetLegRadius(leg_radius))
                    }
                    ControllerServiceEvent::LegRadiusCccdWrite {
                        indications,
                        notifications,
                    } => {
                        command_update.borrow_mut().leg_radius.kind =
                            UpdateKind::from_cccd(notifications, indications);
                        None
                    }
                    ControllerServiceEvent::BatteryUpdateIntervalMsWrite(
                        battery_update_interval,
                    ) => Some(Command::SetBatteryUpdateInterval(battery_update_interval)),
                    ControllerServiceEvent::BatteryUpdateIntervalMsCccdWrite {
                        indications,
                        notifications,
                    } => {
                        command_update.borrow_mut().battery_interval.kind =
                            UpdateKind::from_cccd(notifications, indications);
                        None
                    }
                };

                if let Some(command) = command {
                    let command_task_token = command_task(
                        server,
                        &*connection,
                        state_manager,
                        channel.sender(),
                        command,
                        command_update,
                    );
                    if let Err(e) = spawner.spawn(command_task_token) {
                        defmt::error!("Failed to spawn command task: {:?}", e);
                    }
                }
            }
        })
        .await;
        loop {
            if let Ok(mut connection) = connection.try_borrow_mut() {
                *connection = None;
                break;
            }
        }

        if let Err(e) = res {
            defmt::error!("gatt_server run exited with error: {:?}", e);
        }
    }
}
