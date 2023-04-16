use core::cell::RefCell;

use communication::I2cRequest;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Sender};
use nalgebra::{Translation3, UnitQuaternion, Vector3};
use nrf_softdevice::ble::gatt_server::{RegisterError, RunError};
use nrf_softdevice::ble::{gatt_server, Connection};
use nrf_softdevice::Softdevice;
use state::StateMachine;

use crate::command::{handle_command, CommandUpdate, StateManager, UpdateKind};
use crate::services::{update, UpdateError};
use crate::wrappers::{Translation, UQuaternion, Vector, SM};
use crate::{
    BatteryService, BatteryServiceEvent, Command, ControllerService, ControllerServiceEvent,
    ROBOT_STATE,
};

#[nrf_softdevice::gatt_server]
pub(crate) struct ServerInner {
    pub(crate) bas: BatteryService,
    pub(crate) controller: ControllerService,
}

pub(crate) struct Server(ServerInner);

impl Server {
    pub(crate) fn new(sd: &mut Softdevice) -> Result<Self, RegisterError> {
        let server = ServerInner::new(sd)?;
        server.bas.battery_level_set(&0).unwrap();
        server.controller.sync_set(&false).unwrap();
        server
            .controller
            .state_set(&SM(StateMachine::Paused))
            .unwrap();
        server.controller.speed_set(&0.0).unwrap();
        server.controller.angular_velocity_set(&0.0).unwrap();
        server
            .controller
            .motion_vector_set(&Vector(Vector3::new(0.0, 0.0, 0.0)))
            .unwrap();
        server
            .controller
            .body_translation_set(&Translation(Translation3::new(0.0, 0.0, 0.0)))
            .unwrap();
        server
            .controller
            .body_rotation_set(&UQuaternion(UnitQuaternion::from_euler_angles(
                0.0, 0.0, 0.0,
            )))
            .unwrap();
        server.controller.leg_radius_set(&0.0).unwrap();
        server
            .controller
            .battery_update_interval_ms_set(&0)
            .unwrap();
        Ok(Server(server))
    }

    pub(crate) fn controller(&self) -> &ControllerService {
        &self.0.controller
    }

    pub(crate) fn bas(&self) -> &BatteryService {
        &self.0.bas
    }

    pub(crate) async fn run(
        &'static self,
        conn: &'static RefCell<Option<Connection>>,
        state_manager: &'static RefCell<StateManager>,
        command_update: &'static RefCell<CommandUpdate>,
        channel: &'static Channel<CriticalSectionRawMutex, I2cRequest, 1>,
        spawner: Spawner,
    ) -> Result<(), RunError> {
        gatt_server::run(conn.borrow().as_ref().unwrap(), &self.0, |e| match e {
            ServerInnerEvent::Bas(e) => match e {
                BatteryServiceEvent::BatteryLevelCccdWrite { notifications } => {
                    defmt::info!("battery notifications: {}", notifications);
                }
            },
            ServerInnerEvent::Controller(e) => {
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
                        command_update.borrow_mut().state =
                            UpdateKind::from_cccd(notifications, indications);
                        None
                    }
                    ControllerServiceEvent::SpeedWrite(speed) => Some(Command::SetSpeed(speed)),
                    ControllerServiceEvent::SpeedCccdWrite {
                        indications,
                        notifications,
                    } => {
                        command_update.borrow_mut().speed =
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
                        command_update.borrow_mut().angular_velocity =
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
                        command_update.borrow_mut().motion_vector =
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
                        command_update.borrow_mut().body_translation =
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
                        command_update.borrow_mut().body_rotation =
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
                        command_update.borrow_mut().leg_radius =
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
                        command_update.borrow_mut().battery_interval =
                            UpdateKind::from_cccd(notifications, indications);
                        None
                    }
                };

                if let Some(command) = command {
                    let command_task_token = command_task(
                        self,
                        conn,
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
        .await
    }
}

#[embassy_executor::task]
// Allowed because state_manager is only used as mut in handle command, and not
// used anywhere else.
#[allow(clippy::await_holding_refcell_ref)]
async fn command_task(
    server: &'static Server,
    conn: &'static RefCell<Option<Connection>>,
    state_manager: &'static RefCell<StateManager>,
    sender: Sender<'static, CriticalSectionRawMutex, I2cRequest, 1>,
    command: Command,
    command_update: &'static RefCell<CommandUpdate>,
) {
    defmt::info!("Executing command: {}", command);
    let mut robot_state = ROBOT_STATE.lock().await;
    let res = handle_command(
        &mut robot_state,
        &mut state_manager.borrow_mut(),
        sender,
        command,
    )
    .await;
    if let Err(e) = res {
        defmt::error!("Failed to process command {}: {}", command, e);
    }
    let robot_state = &*robot_state;
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
