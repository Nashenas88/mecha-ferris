use core::cell::RefCell;

use crate::command::{handle_command, CommandUpdate, StateManager, UpdateKind};
use crate::services::{update, UpdateError, Wrapper};
use crate::{log, CommsError, CriticalChannel, CriticalReceiver, CriticalSender};
use crate::{
    BatteryService, BatteryServiceEvent, Command, ControllerService, ControllerServiceEvent,
    ROBOT_STATE,
};
use bluetooth_comms::wrappers::{Translation, UQuaternion, Vector, SM};
use bluetooth_comms::{CalibrationDatum, CalibrationIndex, SetRes, SetResult};
use communication::{I2cRequest, I2cResponse};
use embassy_executor::Spawner;
use nalgebra::{Translation3, UnitQuaternion, Vector3};
use nrf_softdevice::ble::gatt_server::RegisterError;
use nrf_softdevice::ble::{gatt_server, Connection, DisconnectedError};
use nrf_softdevice::Softdevice;
use state::StateMachine;

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
        server.controller.notify_all_set(&false).unwrap();
        server.controller.sync_set(&false).unwrap();
        server
            .controller
            .state_set(&Wrapper(SM(StateMachine::Paused)))
            .unwrap();
        server
            .controller
            .animation_factor_set(&Wrapper(1.0))
            .unwrap();
        server
            .controller
            .angular_velocity_set(&Wrapper(0.0))
            .unwrap();
        server
            .controller
            .motion_vector_set(&Wrapper(Vector(Vector3::new(0.0, 0.0, 0.0))))
            .unwrap();
        server
            .controller
            .body_translation_set(&Wrapper(Translation(Translation3::new(0.0, 0.0, 0.0))))
            .unwrap();
        server
            .controller
            .body_rotation_set(&Wrapper(UQuaternion(UnitQuaternion::from_euler_angles(
                0.0, 0.0, 0.0,
            ))))
            .unwrap();
        server.controller.leg_radius_set(&Wrapper(0.0)).unwrap();
        server
            .controller
            .battery_update_interval_ms_set(&0)
            .unwrap();
        server
            .controller
            .get_calibration_result_set(&Wrapper(CalibrationDatum::new(
                0.0,
                CalibrationIndex {
                    leg: 0,
                    joint: 0,
                    kind: 0,
                },
            )))
            .unwrap();
        server
            .controller
            .set_calibration_result_set(&Wrapper(SetResult {
                index: CalibrationIndex {
                    leg: 0,
                    joint: 0,
                    kind: 0,
                },
                result: SetRes::InvalidIndex,
            }))
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
        request_channel: &'static CriticalChannel<I2cRequest>,
        response_channel: &'static CriticalChannel<Result<I2cResponse, CommsError>>,
        spawner: Spawner,
    ) -> DisconnectedError {
        gatt_server::run(conn.borrow().as_ref().unwrap(), &self.0, |e| match e {
            ServerInnerEvent::Bas(e) => match e {
                BatteryServiceEvent::BatteryLevelCccdWrite { notifications } => {
                    log::info!("battery notifications: {}", notifications);
                    command_update.borrow_mut().battery_level =
                        UpdateKind::from_cccd(notifications, false);
                }
            },
            ServerInnerEvent::Controller(e) => {
                log::info!("Processing command");
                let command = match e {
                    ControllerServiceEvent::NotifyAllWrite(_) => Some(Command::NotifyAll),
                    ControllerServiceEvent::SyncWrite(_) => Some(Command::Sync),
                    ControllerServiceEvent::StateWrite(Wrapper(SM(state_machine))) => {
                        Some(Command::ChangeState(state_machine))
                    }
                    ControllerServiceEvent::StateCccdWrite {
                        indications,
                        notifications,
                    } => {
                        log::info!("state notifications: {}", notifications);
                        command_update.borrow_mut().state =
                            UpdateKind::from_cccd(notifications, indications);
                        None
                    }
                    ControllerServiceEvent::AnimationFactorWrite(animation_factor) => {
                        Some(Command::SetAnimationFactor(animation_factor.0))
                    }
                    ControllerServiceEvent::AnimationFactorCccdWrite {
                        indications,
                        notifications,
                    } => {
                        log::info!("animation_factor notifications: {}", notifications);
                        command_update.borrow_mut().animation_factor =
                            UpdateKind::from_cccd(notifications, indications);
                        None
                    }
                    ControllerServiceEvent::AngularVelocityWrite(angular_velocity) => {
                        Some(Command::SetAngularVelocity(angular_velocity.0))
                    }
                    ControllerServiceEvent::AngularVelocityCccdWrite {
                        indications,
                        notifications,
                    } => {
                        log::info!("angular velocity notifications: {}", notifications);
                        command_update.borrow_mut().angular_velocity =
                            UpdateKind::from_cccd(notifications, indications);
                        None
                    }
                    ControllerServiceEvent::MotionVectorWrite(Wrapper(Vector(motion_vector))) => {
                        Some(Command::SetMotionVector(motion_vector))
                    }
                    ControllerServiceEvent::MotionVectorCccdWrite {
                        indications,
                        notifications,
                    } => {
                        log::info!("motion vector notifications: {}", notifications);
                        command_update.borrow_mut().motion_vector =
                            UpdateKind::from_cccd(notifications, indications);
                        None
                    }
                    ControllerServiceEvent::BodyTranslationWrite(Wrapper(Translation(
                        body_translation,
                    ))) => Some(Command::SetBodyTranslation(body_translation)),
                    ControllerServiceEvent::BodyTranslationCccdWrite {
                        indications,
                        notifications,
                    } => {
                        log::info!("body translation notifications: {}", notifications);
                        command_update.borrow_mut().body_translation =
                            UpdateKind::from_cccd(notifications, indications);
                        None
                    }
                    ControllerServiceEvent::BodyRotationWrite(Wrapper(UQuaternion(
                        body_rotation,
                    ))) => Some(Command::SetBodyRotation(body_rotation)),
                    ControllerServiceEvent::BodyRotationCccdWrite {
                        indications,
                        notifications,
                    } => {
                        log::info!("body rotation notifications: {}", notifications);
                        command_update.borrow_mut().body_rotation =
                            UpdateKind::from_cccd(notifications, indications);
                        None
                    }
                    ControllerServiceEvent::LegRadiusWrite(leg_radius) => {
                        Some(Command::SetLegRadius(leg_radius.0))
                    }
                    ControllerServiceEvent::LegRadiusCccdWrite {
                        indications,
                        notifications,
                    } => {
                        log::info!("leg radius notifications: {}", notifications);
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
                        log::info!("battery interval notifications: {}", notifications);
                        command_update.borrow_mut().battery_interval =
                            UpdateKind::from_cccd(notifications, indications);
                        None
                    }
                    ControllerServiceEvent::GetCalibationForWrite(calibration_index) => {
                        Some(Command::GetCalibrationFor(calibration_index.0))
                    }
                    ControllerServiceEvent::GetCalibrationResultCccdWrite { notifications } => {
                        log::info!("get calibration result notifications: {}", notifications);
                        command_update.borrow_mut().get_calibration_result =
                            UpdateKind::from_cccd(notifications, false);
                        None
                    }
                    ControllerServiceEvent::SetCalibrationDatumWrite(datum) => Some(
                        Command::SetCalibrationDatum(datum.0.value(), datum.0.index()),
                    ),
                    ControllerServiceEvent::SetCalibrationResultCccdWrite { notifications } => {
                        log::info!("calibration result notifications: {}", notifications);
                        command_update.borrow_mut().set_calibration_result =
                            UpdateKind::from_cccd(notifications, false);
                        None
                    }
                };

                if let Some(command) = command {
                    let command_task_token = command_task(
                        self,
                        conn,
                        state_manager,
                        request_channel.sender(),
                        response_channel.receiver(),
                        command,
                        command_update,
                    );
                    if let Err(e) = spawner.spawn(command_task_token) {
                        log::error!("Failed to spawn command task: {:?}", e);
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
    request_tx: CriticalSender<I2cRequest>,
    response_rx: CriticalReceiver<Result<I2cResponse, CommsError>>,
    command: Command,
    command_update: &'static RefCell<CommandUpdate>,
) {
    log::info!("Executing command: {}", command);
    let mut robot_state = ROBOT_STATE.lock().await;
    let res = handle_command(
        &mut robot_state,
        &mut state_manager.borrow_mut(),
        request_tx,
        response_rx,
        command,
    )
    .await;
    let robot_state = &*robot_state;
    let command_results = command.to_results(robot_state, res);
    if let Ok(conn) = conn.try_borrow() {
        if let Some(conn) = &*conn {
            for res in command_results {
                let res: Result<(), UpdateError> =
                    update(server, conn, &command_update.borrow(), res);
                if let Err(e) = res {
                    log::info!("send notification error: {:?}", e);
                }
            }
        }
    }
    log::info!("Done executing command: {}", command);
}
